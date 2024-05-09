#include "hwacc/window_manager.hh"
#include "base/trace.hh"
#include "debug/WindowManager.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "sim/system.hh"
#include <algorithm>

#include <string>

WindowManager::WindowManager(const WindowManagerParams &p)
    : BasicPioDevice(p, p.pio_size), tickEvent(this), deviceName(p.devicename),
      debugEnabled(p.debug_enabled), clockPeriod(p.clock_period),
      masterId(p.system->getRequestorId(this, name())), io_size(p.pio_size),
      io_addr(p.pio_addr), endian(p.system->getGuestByteOrder()) {
  processingDelay = 1000 * clockPeriod;

  signalCurrentFreeSPMAddr = spmAddr;

  mmr = new uint8_t[io_size];
  std::fill(mmr, mmr + io_size, 0);
}

void WindowManager::recvPacket(PacketPtr pkt) {
  if (pkt->isRead()) {
    MemoryRequest *read_req = findMemRequest(pkt, activeReadRequests);
    RequestPort *carrier_port = read_req->getCarrierPort();
    pkt->writeData(read_req->getBuffer() +
                   (pkt->req->getPaddr() - read_req->getBeginAddr()));

    if (debug())
      DPRINTF(WindowManager,
              "Done with a read. addr: 0x%x, size: %d, data: %s\n",
              pkt->req->getPaddr(), pkt->getSize(), read_req->printBuffer());

    if (PEPort *pe_port = dynamic_cast<PEPort *>(carrier_port)) {
      if (isSignalMode()) {
        if (*((uint64_t *)read_req->getBuffer()) == endToken) {
          finishedPEs.push_back(true);
          removeRequest(read_req, activeReadRequests);
          scheduleEvent();
          delete pkt;
          return;
        }

        handleSignalPEResponse(read_req, pe_port);
      } else if (isTimeseriesMode()) {
        handleTimeseriesPEResponse(read_req, pe_port);
      }
    } else if (GlobalPort *global_port =
                   dynamic_cast<GlobalPort *>(carrier_port)) {
      if (isSignalMode()) {
        handleSignalMemoryResponse(pkt, read_req);
      } else if (isTimeseriesMode()) {
        handleTimeseriesMemoryResponse(pkt, read_req);
      }
    } else if (SPMPort *spm_port = dynamic_cast<SPMPort *>(carrier_port)) {
      handleTimeseriesMemoryResponse(pkt, read_req);
    } else {
      panic("Unknown port receiving the read response!\n");
    }

    removeTimeseriesWindowRequest(read_req);
    removeRequest(read_req, activeReadRequests);
    delete read_req;
  } else if (pkt->isWrite()) {
    MemoryRequest *write_req = findMemRequest(pkt, activeWriteRequests);
    RequestPort *carrier_port = write_req->getCarrierPort();

    if (debug())
      DPRINTF(WindowManager, "Done with a write. addr: 0x%x, size: %d\n",
              pkt->req->getPaddr(), pkt->getSize());

    if (SPMPort *port = dynamic_cast<SPMPort *>(carrier_port)) {
      removeSignalWindowRequest(write_req);
    } else if (PEPort *port = dynamic_cast<PEPort *>(carrier_port)) {
    } else if (GlobalPort *port = dynamic_cast<GlobalPort *>(carrier_port)) {
      handleTimeseriesMemoryResponse(pkt, write_req);
    } else {
      panic("Unknown port receiving the write response!\n");
    }

    removeTimeseriesWindowRequest(write_req);
    removeRequest(write_req, activeWriteRequests);
    delete write_req;
  } else {
    panic("Received packet that is neither read nor write!\n");
  }

  scheduleEvent();
  delete pkt;
}

/* Timeseries-related functions */
WindowManager::TimeseriesWindow::TimeseriesWindow(WindowManager *owner,
                                                  PEPort *pe_port)
    : state(none), windowName("TimeseriesWindow"), owner(owner),
      correspondingPEPort(pe_port), numReceivedChildren(0), currentCoreAddr(0),
      currentComputationCoreAddr(0), numCheckedNodes(0),
      maxCacheIndex((size_t)(owner->spmSize / cacheEntrySize - 1)),
      saveCacheCoreAddr(0), checkCacheCoreAddr(0),
      minCacheEntryAccessTick(UINT64_MAX), minCacheEntryIndex(0) {
  if (debug())
    DPRINTF(WindowManager, "maxCacheIndex %d\n", maxCacheIndex);
}

void WindowManager::TimeseriesWindow::firstScanTick() {
  if (state == requestCoreStart) {
    if (debug())
      DPRINTF(WindowManager, "State: requestCoreStart\n");

    if (coreQueue.empty()) {
      state = firstScanDone;
      return;
    }

    currentCoreAddr = coreQueue.front();
    Addr coreRangeStartAddr = currentCoreAddr + coreRangeStartOffset;

    if (debug())
      DPRINTF(WindowManager, "Core Start Addr: 0x%016x\n", currentCoreAddr);

    if (GlobalPort *memory_port =
            owner->getValidGlobalPort(coreRangeStartAddr, true)) {
      MemoryRequest *read_req = owner->readFromPort(
          memory_port, coreRangeStartAddr, owner->tsDataSize);
      owner->activeTimeseriesWindowRequests[this].push_back(read_req);
      state = waitingCoreStart;
    } else {
      if (debug())
        DPRINTF(WindowManager,
                "Did not find a global port to read %d bytes from 0x%016x\n",
                owner->tsDataSize, coreRangeStartAddr);
      currentCoreAddr = 0;
    }
  } else if (state == requestCoreEnd) {
    if (debug())
      DPRINTF(WindowManager, "State: requestCoreEnd\n");

    Addr coreRangeEndAddr = currentCoreAddr + coreRangeEndOffset;

    if (debug())
      DPRINTF(WindowManager, "Core End Addr: 0x%016x\n", coreRangeEndAddr);

    if (GlobalPort *memory_port =
            owner->getValidGlobalPort(coreRangeEndAddr, true)) {
      MemoryRequest *read_req =
          owner->readFromPort(memory_port, coreRangeEndAddr, owner->tsDataSize);
      owner->activeTimeseriesWindowRequests[this].push_back(read_req);
      state = waitingCoreEnd;
    } else {
      if (debug())
        DPRINTF(WindowManager,
                "Did not find a global port to read %d bytes from 0x%016x\n",
                owner->tsDataSize, coreRangeEndAddr);
    }
  } else if (state == checkRange) {
    if (debug())
      DPRINTF(WindowManager, "State: checkRange\n");

    uint64_t inp_start = peRequest.startTimestamp;
    uint64_t inp_end = peRequest.endTimestamp;
    double node_start, node_end;
    memcpy(&node_start, &currentCoreStart, sizeof(node_start));
    memcpy(&node_end, &currentCoreEnd, sizeof(node_end));

    assert(node_start < node_end);

    if (debug())
      DPRINTF(WindowManager,
              "Checking range inp_start:%d inp_end:%d against node_start:%f "
              "node_end:%f\n",
              inp_start, inp_end, node_start, node_end);

    bool is_leaf = isLeafNode(currentCoreAddr);
    bool case_skip = (node_start > inp_end || node_end < inp_start);
    bool case_finished = (inp_start == node_start && node_end == inp_end);
    bool case_add_to_list = (inp_start <= node_start && node_end <= inp_end);

    if (debug())
      DPRINTF(WindowManager, "is leaf %d skip %d finished %d add %d \n",
              is_leaf, case_skip, case_finished, case_add_to_list);

    if (case_skip) {
      state = requestCoreStart;
    } else if (case_finished) {
      computationCores.push(currentCoreAddr);
      state = firstScanDone;
    } else if (case_add_to_list) {
      computationCores.push(currentCoreAddr);
      state = requestCoreStart;
    } else { // this means either leaf/partial or non-leaf/children
      if (!is_leaf) {
        if (debug())
          DPRINTF(WindowManager, "Request children\n");

        state = requestChildAddr;
      } else {
        double partial_start = fmax(inp_start, node_start);
        double partial_diff = fabs(partial_start - fmin(inp_end, node_end));

        Addr partial_start_addr =
            currentCoreAddr +
            ((uint64_t)(partial_start - node_start)) * coreStatOffset;
        uint64_t partial_length = (uint64_t)partial_diff;

        if (debug())
          DPRINTF(WindowManager, "Added to partials 0x%016x, %d\n",
                  partial_start_addr, partial_length);

        partials.push(std::make_pair(partial_start_addr, partial_length));
        state = requestCoreStart;
      }
    }

    if (!coreQueue.empty())
      coreQueue.pop();
  } else if (state == requestChildAddr) {
    if (debug())
      DPRINTF(WindowManager, "State: requestChildAddr\n");

    assert(numReceivedChildren < numChildren);

    Addr childAddr =
        currentCoreAddr + (numReceivedChildren + 3) * owner->tsDataSize;

    if (debug())
      DPRINTF(WindowManager,
              "Requesting address for Child number %d which is inside address "
              "0x%016x\n",
              numReceivedChildren, childAddr);

    if (GlobalPort *memory_port = owner->getValidGlobalPort(childAddr, true)) {
      MemoryRequest *read_req =
          owner->readFromPort(memory_port, childAddr, owner->tsDataSize);
      owner->activeTimeseriesWindowRequests[this].push_back(read_req);
      state = waitingChildAddr;
    } else {
      if (debug())
        DPRINTF(WindowManager,
                "Did not find a global port to read %d bytes from 0x%016x\n",
                owner->tsDataSize, childAddr);
    }
  } else if (state == firstScanDone) {
    state = initTraverse;
  }
}

void WindowManager::TimeseriesWindow::traverseTick() {
  if (state == initTraverse) {
    if (debug())
      DPRINTF(WindowManager, "State: initTraverse\n");

    assert(!(computationCores.empty() && !traverseStack.empty()));

    if (computationCores.empty() && traverseStack.empty()) {
      if (debug())
        DPRINTF(WindowManager, "Computation cores are done!\n");

      if (partials.empty()) {
        if (debug())
          DPRINTF(WindowManager, "We are done!\n");

        state = done;
        uint64_t result = 0;
        for (auto val : endResults)
          result += val;
        result /= endResults.size();

        if (debug())
          DPRINTF(WindowManager, "END RESULT IS 0x%016x\n", result);

        return;
      } else {
        assert(coresDone());

        if (debug())
          DPRINTF(WindowManager, "Still have partials to do\n");

        currentPartial = partials.front();
        partials.pop();
        state = computeStat;
      }
    } else {
      if (debug())
        DPRINTF(WindowManager, "Going over this computation core: 0x%016x\n",
                computationCores.front());

      traverseStack.push(std::make_pair(computationCores.front(), false));
      currentComputationCoreAddr = computationCores.front();
      computationCores.pop();

      // alternate cache/no cache
      if (useCache()) {
        state = checkCache;
      } else {
        state = requestStat;
      }
    }
  }

  else if (state == checkCache) {
    if (debug())
      DPRINTF(WindowManager, "State: checkCache\n");

    currentCoreHash = 0;
    checkCacheCoreAddr = currentComputationCoreAddr;
    state = checkCache_RequestCoreStart;
    currentCacheEntryIndex = 0;
  } else if (state == checkCache_RequestCoreStart) {
    if (debug())
      DPRINTF(WindowManager, "State: checkCache_RequestCoreStart\n");

    assert(checkCacheCoreAddr != 0);
    Addr core_start = checkCacheCoreAddr + coreRangeStartOffset;

    if (GlobalPort *memory_port = owner->getValidGlobalPort(core_start, true)) {
      MemoryRequest *read_req =
          owner->readFromPort(memory_port, core_start, owner->tsDataSize);
      owner->activeTimeseriesWindowRequests[this].push_back(read_req);
      state = checkCache_WaitingCoreStart;
    } else {
      if (debug())
        DPRINTF(WindowManager,
                "Did not find a global port to read %d bytes from 0x%016x\n",
                owner->tsDataSize, core_start);
    }
  } else if (state == checkCache_RequestCoreEnd) {
    if (debug())
      DPRINTF(WindowManager, "State: checkCache_RequestCoreEnd\n");

    assert(checkCacheCoreAddr != 0);
    Addr core_end = checkCacheCoreAddr + coreRangeEndOffset;

    if (GlobalPort *memory_port = owner->getValidGlobalPort(core_end, true)) {
      MemoryRequest *read_req =
          owner->readFromPort(memory_port, core_end, owner->tsDataSize);
      owner->activeTimeseriesWindowRequests[this].push_back(read_req);
      state = checkCache_WaitingCoreEnd;
    } else {
      if (debug())
        DPRINTF(WindowManager,
                "Did not find a global port to read %d bytes from 0x%016x\n",
                owner->tsDataSize, core_end);
    }
  } else if (state == checkCache_RequestScanCacheEntry) {
    if (debug())
      DPRINTF(WindowManager, "State: checkCache_RequestScanCacheEntry %d\n",
              currentCacheEntryIndex);

    assert(currentCoreHash != 0);

    Addr cache_entry_hash_addr = owner->spmAddr +
                                 currentCacheEntryIndex * cacheEntrySize +
                                 owner->tsDataSize;

    if (SPMPort *spm_port = owner->getValidSPMPort(cache_entry_hash_addr)) {
      MemoryRequest *read_req = owner->readFromPort(
          spm_port, cache_entry_hash_addr, owner->tsDataSize);
      owner->activeTimeseriesWindowRequests[this].push_back(read_req);

      state = checkCache_WaitingScanCacheEntry;
    } else {
      if (debug())
        DPRINTF(WindowManager,
                "Did not find a spm port to read %d bytes from 0x%016x\n",
                owner->tsDataSize, cache_entry_hash_addr);
    }
  } else if (state == checkCache_RequestFoundEntryStat) {
    if (debug())
      DPRINTF(WindowManager, "State: checkCache_RequestFoundEntryStat\n");

    Addr cache_entry_result_addr = owner->spmAddr +
                                   currentCacheEntryIndex * cacheEntrySize +
                                   coreStatOffset;

    if (SPMPort *spm_port = owner->getValidSPMPort(cache_entry_result_addr)) {
      MemoryRequest *read_req = owner->readFromPort(
          spm_port, cache_entry_result_addr, owner->tsDataSize);
      owner->activeTimeseriesWindowRequests[this].push_back(read_req);

      state = checkCache_WaitingFoundEntryStat;
    } else {
      if (debug())
        DPRINTF(WindowManager,
                "Did not find a spm port to read %d bytes from 0x%016x\n",
                owner->tsDataSize, cache_entry_result_addr);
    }
  } else if (state == checkCache_UpdateFoundEntryAccessCycle) {
    if (debug())
      DPRINTF(WindowManager, "State: checkCache_UpdateFoundEntryAccessCycle\n");

    Addr cache_entry_meta_info_addr =
        owner->spmAddr + currentCacheEntryIndex * cacheEntrySize;

    if (SPMPort *spm_port =
            owner->getValidSPMPort(cache_entry_meta_info_addr, false)) {
      MemoryRequest *write_req = owner->writeToPort(
          spm_port, (uint64_t)curTick(), cache_entry_meta_info_addr);
      owner->activeTimeseriesWindowRequests[this].push_back(write_req);

      if (debug())
        DPRINTF(WindowManager,
                "done one computation core with cache hit 0x%016x 0x%016x\n",
                currentCoreHash, currentComputationCoreAddr);

      currentCacheEntryIndex = 0;
      currentCoreHash = 0;
      currentComputationCoreAddr = 0;

      traverseStack.pop();
      assert(traverseStack.empty());

      state = initTraverse;
    } else {
      if (debug())
        DPRINTF(WindowManager,
                "Did not find a global port to write %d bytes to 0x%016x\n",
                owner->tsDataSize, cache_entry_meta_info_addr);
    }
  } else if (state == checkCache_NotFound) {
    if (debug())
      DPRINTF(WindowManager, "State: checkCache_NotFound\n");

    currentCacheEntryIndex = 0;
    currentCoreHash = 0;
    state = requestStat;
  }

  else if (state == requestStat) {
    if (debug())
      DPRINTF(WindowManager, "State: requestStat\n");

    if (traverseStack.top()
            .second) { // visited nodes do not need initial stat check
      if (debug())
        DPRINTF(WindowManager, "no need for stat check\n");

      state = startTraverse;
      return;
    }

    Addr statAddr = traverseStack.top().first + coreStatOffset;

    if (debug())
      DPRINTF(WindowManager, "stat addr: 0x%016x\n", statAddr);

    if (GlobalPort *memory_port = owner->getValidGlobalPort(statAddr, true)) {
      MemoryRequest *read_req =
          owner->readFromPort(memory_port, statAddr, owner->tsDataSize);
      owner->activeTimeseriesWindowRequests[this].push_back(read_req);
      numCheckedNodes++;

      state = waitingStat;
    } else {
      if (debug())
        DPRINTF(WindowManager,
                "Did not find a global port to read %d bytes from 0x%016x\n",
                owner->tsDataSize, statAddr);
    }
  } else if (state == startTraverse) {
    if (debug())
      DPRINTF(WindowManager, "State: startTraverse with current size of %d\n",
              traverseStack.size());

    traverseStackHead = traverseStack.top();

    if (debug())
      DPRINTF(WindowManager, "Head addr and visited: 0x%016x, %d\n",
              traverseStackHead.first, traverseStackHead.second);

    if (!traverseStackHead.second) { // not visited -> add children if not leaf
      traverseStack.top().second = true;

      if (isLeafNode(traverseStackHead.first)) { // is leaf -> nothing special
        state = startTraverse;
      } else { // is not leaf -> add children to stack
        state = requestTraverseChildren;
      }
    } else { // visited -> comp stat -> pop
      state = computeStat;
    }
  } else if (state == requestTraverseChildren) {
    if (debug())
      DPRINTF(WindowManager, "State: requestTraverseChildren\n");

    assert(numReceivedChildren >= 0 && numReceivedChildren < numChildren);

    // adding to stack from end to beginning
    Addr childAddr =
        traverseStackHead.first +
        ((numChildren - numReceivedChildren - 1) + 3) * owner->tsDataSize;

    if (debug())
      DPRINTF(WindowManager,
              "Requesting address for Child number %d which is inside address "
              "0x%016x\n",
              numReceivedChildren, childAddr);

    if (GlobalPort *memory_port = owner->getValidGlobalPort(childAddr, true)) {
      MemoryRequest *read_req =
          owner->readFromPort(memory_port, childAddr, owner->tsDataSize);
      owner->activeTimeseriesWindowRequests[this].push_back(read_req);
      state = waitingTraverseChildren;
    } else {
      if (debug())
        DPRINTF(WindowManager,
                "Did not find a global port to read %d bytes from 0x%016x\n",
                owner->tsDataSize, childAddr);
    }
  }

  else if (state == computeStat) {
    if (debug())
      DPRINTF(WindowManager, "State: computeStat\n");

    openPEPort = findPEPortForComputeState();
    Addr port_addr = openPEPort->getStartAddr();

    if (coresDone() &&
        currentPartial != std::pair<Addr, uint64_t>()) { // partial calc
      if (debug())
        DPRINTF(WindowManager, "calculating partial\n");

      MemoryRequest *write_req =
          owner->writeToPort(openPEPort, (uint64_t)1, port_addr);
      owner->activeTimeseriesWindowRequests[this].push_back(write_req);

      write_req =
          owner->writeToPort(openPEPort, currentPartial.second, port_addr);
      owner->activeTimeseriesWindowRequests[this].push_back(write_req);

      write_req =
          owner->writeToPort(openPEPort, currentPartial.first, port_addr);
      owner->activeTimeseriesWindowRequests[this].push_back(write_req);

      state = computeStat_RequestResult;
    } else if (isLeafNode(traverseStackHead.first)) { // leaf node
      if (debug())
        DPRINTF(WindowManager, "calculating leaf node\n");

      MemoryRequest *write_req =
          owner->writeToPort(openPEPort, (uint64_t)1, port_addr);
      owner->activeTimeseriesWindowRequests[this].push_back(write_req);

      write_req = owner->writeToPort(
          openPEPort, (uint64_t)(batchSize * numChildren), port_addr);
      owner->activeTimeseriesWindowRequests[this].push_back(write_req);

      state = computeStat_RequestLeafValuesStartAddr;
    } else { // non-leaf node
      if (debug())
        DPRINTF(WindowManager, "calculating core node\n");

      MemoryRequest *write_req =
          owner->writeToPort(openPEPort, (uint64_t)2, port_addr);
      owner->activeTimeseriesWindowRequests[this].push_back(write_req);

      write_req =
          owner->writeToPort(openPEPort, (uint64_t)numChildren, port_addr);
      owner->activeTimeseriesWindowRequests[this].push_back(write_req);

      state = computeStat_RequestCoreChildAddr;
    }
  } else if (state == computeStat_RequestLeafValuesStartAddr) {
    if (debug())
      DPRINTF(WindowManager, "State: computeStat_RequestLeafValuesStartAddr\n");

    // TODO: leaf's children might not be sequential
    Addr childStartAddr = traverseStackHead.first + 3 * owner->tsDataSize;

    if (GlobalPort *memory_port =
            owner->getValidGlobalPort(childStartAddr, true)) {
      MemoryRequest *read_req =
          owner->readFromPort(memory_port, childStartAddr, owner->tsDataSize);
      owner->activeTimeseriesWindowRequests[this].push_back(read_req);
      state = computeStat_WaitingLeafValuesStartAddr;
    } else {
      if (debug())
        DPRINTF(WindowManager,
                "Did not find a global port to read %d bytes from 0x%016x\n",
                owner->tsDataSize, childStartAddr);
    }
  } else if (state == computeStat_RequestCoreChildAddr) {
    if (debug())
      DPRINTF(WindowManager, "State: computeStat_RequestCoreChildAddr\n");

    Addr childStartAddr =
        traverseStackHead.first + (numReceivedChildren + 3) * owner->tsDataSize;

    if (GlobalPort *memory_port =
            owner->getValidGlobalPort(childStartAddr, true)) {
      MemoryRequest *read_req =
          owner->readFromPort(memory_port, childStartAddr, owner->tsDataSize);
      owner->activeTimeseriesWindowRequests[this].push_back(read_req);

      state = computeStat_WaitingCoreChildAddr;
    } else {
      if (debug())
        DPRINTF(WindowManager,
                "Did not find a global port to read %d bytes from 0x%016x\n",
                owner->tsDataSize, childStartAddr);
    }
  } else if (state == computeStat_RequestCoreChildStat) {
    if (debug())
      DPRINTF(WindowManager, "State: computeStat_RequestCoreChildStat\n");

    Addr childStatAddr = computeStatChildAddr + coreStatOffset;

    if (GlobalPort *memory_port =
            owner->getValidGlobalPort(childStatAddr, true)) {
      MemoryRequest *read_req =
          owner->readFromPort(memory_port, childStatAddr, owner->tsDataSize);
      owner->activeTimeseriesWindowRequests[this].push_back(read_req);
      state = computeStat_WaitingCoreChildStat;
    } else {
      if (debug())
        DPRINTF(WindowManager,
                "Did not find a global port to read %d bytes from 0x%016x\n",
                owner->tsDataSize, childStatAddr);
    }
  } else if (state == computeStat_RequestResult) {
    if (debug())
      DPRINTF(WindowManager, "State: computeStat_RequestResult\n");

    assert(openPEPort != nullptr);
    int port_idx = std::distance(owner->peResponseStreamPorts.begin(),
                                 std::find(owner->peResponseStreamPorts.begin(),
                                           owner->peResponseStreamPorts.end(),
                                           openPEPort));
    assert(port_idx < owner->peRequestStreamPorts.size());
    PEPort *pe_req_port = owner->peRequestStreamPorts[port_idx];
    assert(pe_req_port != nullptr);

    if (owner->checkPort(pe_req_port, owner->peRequestLength, true)) {
      MemoryRequest *read_req = owner->readFromPort(
          pe_req_port, pe_req_port->getStartAddr(), owner->peRequestLength);
      owner->activeTimeseriesWindowRequests[this].push_back(read_req);
      state = computeStat_WaitingResult;
    }
  } else if (state == computeStat_SaveResult) {
    if (debug())
      DPRINTF(WindowManager, "State: computeStat_SaveResult\n");

    Addr currentCoreStatAddr = traverseStackHead.first + coreStatOffset;

    if (GlobalPort *memory_port =
            owner->getValidGlobalPort(currentCoreStatAddr, true)) {
      MemoryRequest *write_req =
          owner->writeToPort(memory_port, computedResult, currentCoreStatAddr);
      owner->activeTimeseriesWindowRequests[this].push_back(write_req);
      state = computeStat_WaitingSaveResult;
    } else {
      if (debug())
        DPRINTF(WindowManager,
                "Did not find a global port to read %d bytes from 0x%016x\n",
                owner->tsDataSize, currentCoreStatAddr);
    }
  }

  else if (state == saveCache) {
    if (debug())
      DPRINTF(WindowManager, "State: saveCache\n");

    currentCoreHash = 0;
    saveCacheCoreAddr = currentComputationCoreAddr;
    currentCacheEntryIndex = 0;
    saveCacheEntryStat = computedResult;

    state = saveCache_RequestCoreStart;
  } else if (state == saveCache_RequestCoreStart) {
    if (debug())
      DPRINTF(WindowManager, "State: saveCache_RequestCoreStart\n");

    assert(saveCacheCoreAddr != 0);
    Addr core_start = saveCacheCoreAddr + 0;

    if (GlobalPort *memory_port = owner->getValidGlobalPort(core_start, true)) {
      MemoryRequest *read_req =
          owner->readFromPort(memory_port, core_start, owner->tsDataSize);
      owner->activeTimeseriesWindowRequests[this].push_back(read_req);
      state = saveCache_WaitingCoreStart;
    } else {
      if (debug())
        DPRINTF(WindowManager,
                "Did not find a global port to read %d bytes from 0x%016x\n",
                owner->tsDataSize, core_start);
    }
  } else if (state == saveCache_RequestCoreEnd) {
    if (debug())
      DPRINTF(WindowManager, "State: saveCache_RequestCoreEnd\n");

    assert(saveCacheCoreAddr != 0);
    Addr core_end = saveCacheCoreAddr + owner->tsDataSize;

    if (GlobalPort *memory_port = owner->getValidGlobalPort(core_end, true)) {
      MemoryRequest *read_req =
          owner->readFromPort(memory_port, core_end, owner->tsDataSize);
      owner->activeTimeseriesWindowRequests[this].push_back(read_req);
      state = saveCache_WaitingCoreEnd;
    } else {
      if (debug())
        DPRINTF(WindowManager,
                "Did not find a global port to read %d bytes from 0x%016x\n",
                owner->tsDataSize, core_end);
    }
  } else if (state == saveCache_RequestEmptyEntry) {
    if (debug())
      DPRINTF(WindowManager, "State: saveCache_RequestEmptyEntry\n");

    assert(currentCoreHash != 0);

    Addr cache_entry_hash_addr = owner->spmAddr +
                                 currentCacheEntryIndex * cacheEntrySize +
                                 owner->tsDataSize;

    if (SPMPort *spm_port = owner->getValidSPMPort(cache_entry_hash_addr)) {
      MemoryRequest *read_req = owner->readFromPort(
          spm_port, cache_entry_hash_addr, owner->tsDataSize);
      owner->activeTimeseriesWindowRequests[this].push_back(read_req);

      state = saveCache_WaitingEmptyEntry;
    } else {
      if (debug())
        DPRINTF(WindowManager,
                "Did not find a spm port to read %d bytes from 0x%016x\n",
                owner->tsDataSize, cache_entry_hash_addr);
    }
  } else if (state == saveCache_SaveCacheEntry) {
    if (debug())
      DPRINTF(WindowManager, "State: saveCache_SaveCacheEntry\n");

    Addr cache_entry_meta_info_addr =
        owner->spmAddr + currentCacheEntryIndex * cacheEntrySize;
    Addr cache_entry_hash_addr = owner->spmAddr +
                                 currentCacheEntryIndex * cacheEntrySize +
                                 owner->tsDataSize;
    Addr cache_entry_stat_addr = owner->spmAddr +
                                 currentCacheEntryIndex * cacheEntrySize +
                                 coreStatOffset;

    if (SPMPort *spm_port =
            owner->getValidSPMPort(cache_entry_meta_info_addr, false)) {
      MemoryRequest *write_req =
          owner->writeToPort(spm_port, curTick(), cache_entry_meta_info_addr);
      owner->activeTimeseriesWindowRequests[this].push_back(write_req);

      write_req =
          owner->writeToPort(spm_port, currentCoreHash, cache_entry_hash_addr);
      owner->activeTimeseriesWindowRequests[this].push_back(write_req);

      write_req = owner->writeToPort(spm_port, saveCacheEntryStat,
                                     cache_entry_stat_addr);
      owner->activeTimeseriesWindowRequests[this].push_back(write_req);

      currentComputationCoreAddr = 0;
      computedResult = 0;
      saveCacheEntryStat = 0;
      state = initTraverse;
    } else {
      if (debug())
        DPRINTF(WindowManager,
                "Did not find a global port to write entry to 0x%016x\n",
                cache_entry_meta_info_addr);
    }
  } else if (state == saveCache_Replace) {
    if (debug())
      DPRINTF(WindowManager, "State: saveCache_Replace\n");

    Addr cache_entry_meta_info_addr =
        owner->spmAddr + minCacheEntryIndex * cacheEntrySize;
    Addr cache_entry_hash_addr = owner->spmAddr +
                                 minCacheEntryIndex * cacheEntrySize +
                                 owner->tsDataSize;
    Addr cache_entry_stat_addr =
        owner->spmAddr + minCacheEntryIndex * cacheEntrySize + coreStatOffset;

    if (SPMPort *spm_port =
            owner->getValidSPMPort(cache_entry_meta_info_addr, false)) {
      MemoryRequest *write_req =
          owner->writeToPort(spm_port, curTick(), cache_entry_meta_info_addr);
      owner->activeTimeseriesWindowRequests[this].push_back(write_req);

      write_req =
          owner->writeToPort(spm_port, currentCoreHash, cache_entry_hash_addr);
      owner->activeTimeseriesWindowRequests[this].push_back(write_req);

      write_req = owner->writeToPort(spm_port, saveCacheEntryStat,
                                     cache_entry_stat_addr);
      owner->activeTimeseriesWindowRequests[this].push_back(write_req);

      currentComputationCoreAddr = 0;
      minCacheEntryAccessTick = UINT64_MAX;
      minCacheEntryIndex = -1;
      saveCacheCoreAddr = 0;

      state = initTraverse;
    } else {
      if (debug())
        DPRINTF(WindowManager,
                "Did not find a global port to write entry to 0x%016x\n",
                cache_entry_meta_info_addr);
    }
  }

  else if (state == done) {
  }
}

void WindowManager::timeseriesTick() {
  // finishing conditions
  if (!activeTimeseriesWindows.empty() && allTimeseriesWindowsDone()) {
    // MemoryRequest *write_req =
    //     owner->writeToPort(openPEPort, (uint64_t)endToken, port_addr);
    // owner->activeTimeseriesWindowRequests[this].push_back(write_req);
    // write_req = owner->writeToPort(
    //     openPEPort, (uint64_t)endToken, port_addr);
    // owner->activeTimeseriesWindowRequests[this].push_back(write_req);
    // write_req = owner->writeToPort(
    //     openPEPort, (uint64_t)endToken, port_addr);
    // owner->activeTimeseriesWindowRequests[this].push_back(write_req);

    if (debug())
      DPRINTF(WindowManager, "ALL FINISHED!!!\n");

    *mmr &= 0xfc;
    *mmr |= 0x04;
    return;
  }

  // read from PEs (start, end)
  // TODO: respond port is not always the first port
  for (PEPort *port : peRequestStreamPorts) {
    if (port != peRequestStreamPorts[0] &&
        checkPort(port, peRequestLength, true)) {
      readFromPort(port, port->getStartAddr(), peRequestLength);
    }
  }

  if (debug())
    DPRINTF(WindowManager, "Number of ongoing timeseries windows %d\n",
            activeTimeseriesWindows.size());

  // advance timeseries windows by one
  // TODO: concurrent execution
  for (auto &w : activeTimeseriesWindows) {
    if (!w->isDone()) {
      w->firstScanTick();
      w->traverseTick();
      break;
    }
  }
}

void WindowManager::handleTimeseriesPEResponse(MemoryRequest *read_req,
                                               PEPort *pe_port) {
  // case 1: this is a result computation response from PE
  for (auto &pair : activeTimeseriesWindowRequests) {
    if (std::find(pair.second.begin(), pair.second.end(), read_req) !=
        pair.second.end()) {
      handleTimeseriesMemoryResponse(read_req->getPacket(), read_req);
      removeTimeseriesWindowRequest(read_req);
      return;
    }
  }

  // case 2: finding and finishing up the incomplete window
  uint64_t value = extractPERequestValue(read_req);
  for (auto &w : activeTimeseriesWindows) {
    if (w->samePEPort(pe_port) && w->waitingToStart()) {
      w->setTimeseriesPERequest(value);
      return;
    }
  }

  // case 3: no previous windows matching this port was found => new port
  // request
  TimeseriesWindow *w = new TimeseriesWindow(this, pe_port);
  w->setTimeseriesPERequest(value);
  // window should not already exist in active windows
  assert(std::find(activeTimeseriesWindows.begin(),
                   activeTimeseriesWindows.end(),
                   w) == activeTimeseriesWindows.end());
  activeTimeseriesWindows.push_back(w);
}

void WindowManager::TimeseriesWindow::handleMemoryResponseData(uint64_t data) {
  if (state == waitingCoreStart) {
    if (debug())
      DPRINTF(WindowManager, "State: waitingCoreStart; the start is 0x%016x\n",
              data);

    currentCoreStart = data;
    state = requestCoreEnd;
  } else if (state == waitingCoreEnd) {
    if (debug())
      DPRINTF(WindowManager, "State: waitingCoreEnd; the end is 0x%016x\n",
              data);

    currentCoreEnd = data;
    state = checkRange;
  } else if (state == waitingChildAddr) {
    if (debug())
      DPRINTF(WindowManager,
              "State: waitingChildAddr; the child address is 0x%016x\n", data);

    coreQueue.push(data);
    numReceivedChildren++;
    if (numReceivedChildren == numChildren) {
      state = requestCoreStart;
      numReceivedChildren = 0;
    } else {
      state = requestChildAddr;
    }
  }

  else if (state == waitingStat) {
    if (debug())
      DPRINTF(WindowManager, "State: waitingStat; the stat is 0x%016x\n", data);

    if (data == 0) {
      state = startTraverse;
    } else { // stat is present.
      traverseStack.pop();

      if (traverseStack.empty()) {
        if (debug())
          DPRINTF(WindowManager,
                  "done one computation core with present stat\n");
      }

      // root has stat
      if (numCheckedNodes == 1) {
        numCheckedNodes = 0;
        currentComputationCoreAddr = 0;
        state = initTraverse;
      }
      // non-root has stat
      if (numCheckedNodes > 1) {
        state = requestStat;
      }
    }
  } else if (state == waitingTraverseChildren) {
    if (debug())
      DPRINTF(WindowManager,
              "State: waitingTraverseChildren; the child address is 0x%016x\n",
              data);

    numReceivedChildren++;
    traverseStack.push(std::make_pair(data, false));
    if (numReceivedChildren == numChildren) {
      state = requestStat;
      numReceivedChildren = 0;
    } else {
      state = requestTraverseChildren;
    }
  } else if (state == computeStat_WaitingLeafValuesStartAddr) {
    if (debug())
      DPRINTF(WindowManager,
              "State: computeStat_WaitingLeafValuesStartAddr; the child "
              "address is 0x%016x\n",
              data);

    assert(openPEPort != nullptr);
    MemoryRequest *write_req =
        owner->writeToPort(openPEPort, data, openPEPort->getStartAddr());
    owner->activeTimeseriesWindowRequests[this].push_back(write_req);
    state = computeStat_RequestResult;
  } else if (state == computeStat_WaitingCoreChildAddr) {
    if (debug())
      DPRINTF(WindowManager,
              "State: computeStat_WaitingCoreChildAddr; the child address is "
              "0x%016x\n",
              data);

    computeStatChildAddr = data;
    numReceivedChildren++;
    state = computeStat_RequestCoreChildStat;
  } else if (state == computeStat_WaitingCoreChildStat) {
    if (debug())
      DPRINTF(WindowManager,
              "State: computeStat_WaitingCoreChildStat; the child stat is "
              "0x%016x\n",
              data);

    assert(openPEPort != nullptr);
    MemoryRequest *write_req =
        owner->writeToPort(openPEPort, data, openPEPort->getStartAddr());
    owner->activeTimeseriesWindowRequests[this].push_back(write_req);

    if (numReceivedChildren == numChildren) {
      numReceivedChildren = 0;
      state = computeStat_RequestResult;
    } else {
      state = computeStat_RequestCoreChildAddr;
    }
  } else if (state == computeStat_WaitingResult) {
    if (debug())
      DPRINTF(WindowManager,
              "State: computeStat_WaitingResult; the result from PE is "
              "0x%016x\n",
              data);

    computedResult = data;

    if (coresDone()) { // partial result computation
      endResults.push_back(computedResult);
      currentPartial = std::pair<Addr, uint64_t>();
      currentComputationCoreAddr = 0;
      state = initTraverse;
    } else { // normal node result computation
      state = computeStat_SaveResult;
    }
  } else if (state == computeStat_WaitingSaveResult) {
    if (debug())
      DPRINTF(WindowManager, "State: computeStat_WaitingSaveResult;\n");

    traverseStack.pop();
    if (traverseStack.empty()) { // done for this computation core
      if (debug())
        DPRINTF(WindowManager, "done one computation core with computation\n");

      numCheckedNodes = 0;
      endResults.push_back(computedResult);
      traverseStackHead = std::pair<Addr, bool>();

      // alternate cache/no cache
      if (useCache()) {
        state = saveCache;
      } else {
        currentComputationCoreAddr = 0;
        state = initTraverse;
      }
    } else {
      computedResult = 0;
      state = requestStat;
    }
  }

  else if (state == checkCache_WaitingCoreStart) {
    if (debug())
      DPRINTF(WindowManager,
              "State: checkCache_WaitingCoreStart; the start is 0x%016x\n",
              data);

    if (debug())
      DPRINTF(WindowManager, "hash before 0x%016x ", currentCoreHash);
    currentCoreHash ^= data;
    if (debug())
      DPRINTF(WindowManager, "hash after 0x%016x\n", currentCoreHash);
    state = checkCache_RequestCoreEnd;
  } else if (state == checkCache_WaitingCoreEnd) {
    if (debug())
      DPRINTF(WindowManager,
              "State: checkCache_WaitingCoreEnd; the end is 0x%016x\n", data);

    if (debug())
      DPRINTF(WindowManager, "hash before 0x%016x ", currentCoreHash);
    // currentCoreHash ^= data;
    currentCoreHash = checkCacheCoreAddr;
    if (debug())
      DPRINTF(WindowManager, "hash after 0x%016x\n", currentCoreHash);
    state = checkCache_RequestScanCacheEntry;
  } else if (state == checkCache_WaitingScanCacheEntry) {
    if (debug())
      DPRINTF(
          WindowManager,
          "State: checkCache_WaitingScanCacheEntry; the meta info is 0x%016x\n",
          data);

    if (currentCoreHash == data) {
      state = checkCache_RequestFoundEntryStat;
    } else {
      if (currentCacheEntryIndex == maxCacheIndex - 1) {
        state = checkCache_NotFound;
      } else {
        currentCacheEntryIndex++;
        state = checkCache_RequestScanCacheEntry;
      }
    }
  } else if (state == checkCache_WaitingFoundEntryStat) {
    if (debug())
      DPRINTF(WindowManager,
              "State: checkCache_WaitingFoundEntryStat; the result is "
              "0x%016x\n",
              data);

    endResults.push_back(data);
    state = checkCache_UpdateFoundEntryAccessCycle;
  }

  else if (state == saveCache_WaitingCoreStart) {
    if (debug())
      DPRINTF(WindowManager,
              "State: saveCache_WaitingCoreStart; the start is 0x%016x\n",
              data);

    if (debug())
      DPRINTF(WindowManager, "hash before 0x%016x ", currentCoreHash);
    currentCoreHash ^= data;
    if (debug())
      DPRINTF(WindowManager, "hash after 0x%016x\n", currentCoreHash);
    state = saveCache_RequestCoreEnd;
  } else if (state == saveCache_WaitingCoreEnd) {
    if (debug())
      DPRINTF(WindowManager,
              "State: saveCache_WaitingCoreEnd; the end is 0x%016x\n", data);

    if (debug())
      DPRINTF(WindowManager, "hash before 0x%016x ", currentCoreHash);
    // currentCoreHash ^= data;
    currentCoreHash = saveCacheCoreAddr;
    if (debug())
      DPRINTF(WindowManager, "hash after 0x%016x\n", currentCoreHash);
    currentCacheEntryIndex = 0;
    state = saveCache_RequestEmptyEntry;
  } else if (state == saveCache_WaitingEmptyEntry) {
    if (debug())
      DPRINTF(WindowManager,
              "State: saveCache_WaitingEmptyEntry; the hash is 0x%016x\n",
              data);

    if (data == 0) {
      state = saveCache_SaveCacheEntry;
    } else {
      if (currentCacheEntryIndex == maxCacheIndex - 1) {
        state = saveCache_Replace;
      } else {
        if (fmin(data, minCacheEntryAccessTick) <= minCacheEntryAccessTick) {
          minCacheEntryAccessTick = fmin(data, minCacheEntryAccessTick);
          minCacheEntryIndex = currentCacheEntryIndex;
        }
        currentCacheEntryIndex++;
        state = saveCache_RequestEmptyEntry;
      }
    }
  }
}

bool WindowManager::TimeseriesWindow::isLeafNode(Addr node_addr) {
  // TODO: better leaf check logic
  return leafCoresStartAddr <= node_addr && node_addr <= leafCoresEndAddr;
}

WindowManager::PEPort *
WindowManager::TimeseriesWindow::findPEPortForComputeState() {
  // TODO: write complete PE response port finding and sending
  return owner->peResponseStreamPorts[0];
}

void WindowManager::handleTimeseriesMemoryResponse(PacketPtr pkt,
                                                   MemoryRequest *req) {
  TimeseriesWindow *window = findCorrespondingTimeseriesWindow(pkt);
  uint64_t data = *((uint64_t *)req->getBuffer());
  window->handleMemoryResponseData(data);
}

void WindowManager::TimeseriesWindow::setTimeseriesPERequest(uint64_t value) {
  if (peRequest.startTimestamp == UINT64_MAX)
    peRequest.startTimestamp = value;
  else if (peRequest.endTimestamp == UINT64_MAX) {
    peRequest.endTimestamp = value;
    coreQueue.push(owner->tsCoresStartAddr);
    state = requestCoreStart;
  } else
    panic("Calling setTimeseriesPERequest when you should not!\n");
}

void WindowManager::handleSignalPEResponse(MemoryRequest *read_req,
                                           PEPort *pe_port) {
  // window creation
  SignalPERequest pe_req = constructSignalPERequest(read_req);
  Addr base_addr = pe_req.sourceAddr;
  std::vector<Offset> offsets;
  for (uint16_t idx = pe_req.startElement;
       idx < pe_req.startElement + pe_req.length; idx++)
    offsets.push_back(idx * signalDataSize);

  SignalWindow *window = new SignalWindow(this, base_addr, offsets,
                                          signalCurrentFreeSPMAddr, pe_port);
  activeSignalWindows.push(window);

  // Move the SPM poninter based on PE request
  signalCurrentFreeSPMAddr += pe_req.length * signalDataSize;
}
/* Timeseries-related functions */

/* ************************************************************************** */
/* ************************************************************************** */
/* ************************************************************************** */
/* ************************************************************************** */
/* ************************************************************************** */

/* Signal-related functions */
void WindowManager::signalTick() {
  // turning off condition
  if (activeSignalWindowRequests.empty() &&
      finishedPEs.size() == peRequestStreamPorts.size()) {
    *mmr &= 0xfc;
    *mmr |= 0x04;
    return;
  }

  // Check all the pe ports for incoming requests
  for (PEPort *port : peRequestStreamPorts) {
    if (checkPort(port, peRequestLength, true)) {
      readFromPort(port, port->getStartAddr(), peRequestLength);
    }
  }

  // remove windows without any requests
  for (auto it = activeSignalWindowRequests.begin();
       it != activeSignalWindowRequests.end(); it++)
    if (it->second.empty()) {
      // respond to the corresponding PE
      SignalWindow *window = it->first;
      sendSPMAddrToPE(window->getCorrespondingPEPort(),
                      window->getSPMBaseAddr());
      activeSignalWindowRequests.erase(it);
    }

  // check the single ongoing SignalWindow (head of the queue)
  if (activeSignalWindows.size()) {
    if (debug())
      DPRINTF(WindowManager, "Going over the head window...\n");
    if (!activeSignalWindows.front()->sendMemoryRequest()) {
      activeSignalWindows.pop();
    }
  }

  // check all spmRetryRequests and send them to the SPM
  while (!spmRetryRequests.empty()) {
    SignalWindow *window = spmRetryRequests.front().first;
    uint64_t data = spmRetryRequests.front().second;
    if (window->sendSPMRequest(data))
      spmRetryRequests.pop();
  }
}

void WindowManager::handleSignalMemoryResponse(PacketPtr pkt,
                                               MemoryRequest *read_req) {
  SignalWindow *window = findCorrespondingSignalWindow(pkt);
  uint64_t data = *((uint64_t *)read_req->getBuffer());

  if (!window->sendSPMRequest(data)) {
    spmRetryRequests.push(std::make_pair(window, data));
  }

  removeSignalWindowRequest(read_req);
}

void WindowManager::sendSPMAddrToPE(PEPort *req_pe_port, Addr spm_addr) {
  int port_idx =
      std::distance(peRequestStreamPorts.begin(),
                    std::find(peRequestStreamPorts.begin(),
                              peRequestStreamPorts.end(), req_pe_port));
  assert(port_idx < peResponseStreamPorts.size());

  PEPort *pe_resp_port = peResponseStreamPorts[port_idx];
  Addr destination_addr = pe_resp_port->getStartAddr();

  writeToPort(pe_resp_port, spm_addr, destination_addr);
}

WindowManager::SignalPERequest
WindowManager::constructSignalPERequest(MemoryRequest *read_req) {
  uint64_t result = extractPERequestValue(read_req);

  uint32_t source_addr = result >> 32;
  uint16_t start_element = (result >> 16) & 0x000000000000FFFF;
  uint16_t length = result & 0x000000000000FFFF;

  return (SignalPERequest){source_addr, start_element, length};
}

WindowManager::SignalWindow::SignalWindow(WindowManager *owner,
                                          Addr base_memory_addr,
                                          const std::vector<Offset> &offsets,
                                          Addr base_spm_addr, PEPort *pe_port)
    : windowName("SignalWindow"), owner(owner),
      baseMemoryAddr(base_memory_addr), spmBaseAddr(base_spm_addr),
      currentSPMOffset(0), correspondingPEPort(pe_port) {
  for (Offset o : offsets) {
    Addr req_addr = baseMemoryAddr + o;

    MemoryRequest *read_req =
        new MemoryRequest(req_addr, owner->signalDataSize);
    Request::Flags flags;
    RequestPtr req = std::make_shared<Request>(req_addr, owner->signalDataSize,
                                               flags, owner->masterId);
    PacketPtr pkt = new Packet(req, MemCmd::ReadReq);
    read_req->setPacket(pkt);
    pkt->allocate();
    memoryRequests.push(read_req);
  }
}

bool WindowManager::SignalWindow::sendMemoryRequest() {
  if (memoryRequests.empty()) {
    if (debug())
      DPRINTF(WindowManager, "No more memory read requests to send\n");
    return false;
  }

  MemoryRequest *read_req = memoryRequests.front();

  if (GlobalPort *memory_port = dynamic_cast<GlobalPort *>(
          owner->getValidGlobalPort(read_req->getAddress(), true))) {
    if (debug())
      DPRINTF(WindowManager,
              "Trying to request addr: 0x%016x, %d bytes through port: %s\n",
              read_req->getAddress(), read_req->getLength(),
              memory_port->name());

    read_req->setCarrierPort(memory_port);
    memory_port->sendPacket(read_req->getPacket());

    owner->activeReadRequests.push_back(read_req);
    owner->activeSignalWindowRequests[this].push_back(read_req);
    memoryRequests.pop();
  } else {
    if (debug())
      DPRINTF(WindowManager,
              "Did not find a global port to read %d bytes from 0x%016x\n",
              read_req->getLength(), read_req->getAddress());
  }

  return true;
}

bool WindowManager::SignalWindow::sendSPMRequest(uint64_t data) {
  // create the SPM write packet and send it!
  Addr addr = spmBaseAddr + currentSPMOffset;
  SPMPort *spm_port = owner->getValidSPMPort(addr, false);

  if (!spm_port)
    return false;

  MemoryRequest *write_req = owner->writeToPort(spm_port, data, addr);
  owner->activeSignalWindowRequests[this].push_back(write_req);

  currentSPMOffset += owner->signalDataSize;
  return true;
}
/* Signal-related functions */

void WindowManager::tick() {
  if (debug())
    DPRINTF(WindowManager, "Tick!\n");

  if (isTimeseriesMode()) {
    timeseriesTick();
  } else if (isSignalMode()) {
    signalTick();
  }

  scheduleEvent();
}

/* ************************************************************************** */
/* ************************************************************************** */
/* ************************************************************************** */
/* ************************************************************************** */
/* ************************************************************************** */

/* Utility functions */
MemoryRequest *WindowManager::readFromPort(RequestPort *port, Addr addr,
                                           size_t len) {
  MemoryRequest *read_req = new MemoryRequest(addr, len);
  Request::Flags flags;
  RequestPtr req = std::make_shared<Request>(addr, len, flags, masterId);
  PacketPtr pkt = new Packet(req, MemCmd::ReadReq);

  read_req->setCarrierPort(port);
  read_req->setPacket(pkt);
  pkt->allocate();

  if (debug())
    DPRINTF(WindowManager,
            "Trying to read addr: 0x%016x, %d bytes through port: %s\n", addr,
            len, port->name());

  if (PEPort *pe_port = dynamic_cast<PEPort *>(port)) {
    pe_port->sendPacket(pkt);
    activeReadRequests.push_back(read_req);
  } else if (GlobalPort *memory_port = dynamic_cast<GlobalPort *>(port)) {
    memory_port->sendPacket(pkt);
    activeReadRequests.push_back(read_req);
  } else if (SPMPort *spm_port = dynamic_cast<SPMPort *>(port)) {
    spm_port->sendPacket(pkt);
    activeReadRequests.push_back(read_req);
  } else {
    panic("sending port is not suitable for reading");
  }

  scheduleEvent();
  return read_req;
}

MemoryRequest *WindowManager::writeToPort(RequestPort *req_port,
                                          uint64_t input_data, Addr addr) {
  size_t data_size = isTimeseriesMode() ? tsDataSize : signalDataSize;

  MemoryRequest *write_req =
      new MemoryRequest(addr, (uint8_t *)&input_data, data_size);
  Request::Flags flags;
  RequestPtr req = std::make_shared<Request>(addr, data_size, flags, masterId);
  uint8_t *data = new uint8_t[data_size];
  std::memcpy(data, write_req->getBuffer(), data_size);
  req->setExtraData((uint64_t)data);
  write_req->setCarrierPort(req_port);

  if (debug())
    DPRINTF(
        WindowManager,
        "Trying to write to addr: 0x%016x, %d bytes, holding 0x%016x value, "
        "through port: %s\n",
        addr, data_size, *((uint64_t *)write_req->getBuffer()),
        req_port->name());

  PacketPtr pkt = new Packet(req, MemCmd::WriteReq);
  uint8_t *pkt_data = (uint8_t *)req->getExtraData();
  pkt->dataDynamic(pkt_data);
  write_req->setPacket(pkt);

  if (PEPort *port = dynamic_cast<PEPort *>(req_port)) {
    port->sendPacket(pkt);
    activeWriteRequests.push_back(write_req);
  } else if (SPMPort *port = dynamic_cast<SPMPort *>(req_port)) {
    port->sendPacket(pkt);
    activeWriteRequests.push_back(write_req);
  } else if (GlobalPort *port = dynamic_cast<GlobalPort *>(req_port)) {
    port->sendPacket(pkt);
    activeWriteRequests.push_back(write_req);
  } else {
    panic("sending port is not suitable for writing");
  }

  return write_req;
}

Tick WindowManager::read(PacketPtr pkt) {
  if (debug())
    DPRINTF(WindowManager,
            "The address range associated with this ACC was read!\n");

  Addr offset = pkt->req->getPaddr() - io_addr;
  uint64_t data;
  data = *(uint64_t *)(mmr + offset);

  switch (pkt->getSize()) {
  case 1:
    pkt->set<uint8_t>(data, endian);
    break;
  case 2:
    pkt->set<uint16_t>(data, endian);
    break;
  case 4:
    pkt->set<uint32_t>(data, endian);
    break;
  case 8:
    pkt->set<uint64_t>(data, endian);
    break;
  default:
    panic("Read size too big?\n");
    break;
  }

  pkt->makeAtomicResponse();
  return pioDelay;
}

Tick WindowManager::write(PacketPtr pkt) {
  if (debug()) {
    DPRINTF(WindowManager,
            "The address range associated with this ACC was written to!\n");
    DPRINTF(WindowManager, "Packet addr 0x%lx\n", pkt->req->getPaddr());
    DPRINTF(WindowManager, "IO addr 0x%lx\n", io_addr);
    DPRINTF(WindowManager, "Diff addr 0x%lx\n", pkt->req->getPaddr() - io_addr);
    DPRINTF(WindowManager, "Packet val (LE) %d\n", pkt->getLE<uint8_t>());
    DPRINTF(WindowManager, "Packet val (BE) %d\n", pkt->getBE<uint8_t>());
    DPRINTF(WindowManager, "Packet val %d\n", pkt->get<uint8_t>(endian));
  }

  pkt->writeData(mmr + (pkt->req->getPaddr() - io_addr));
  pkt->makeAtomicResponse();

  scheduleEvent();

  return pioDelay;
}

WindowManager::GlobalPort *WindowManager::getValidGlobalPort(Addr add,
                                                             bool read) {
  for (auto port : globalPorts) {
    AddrRangeList adl = port->getAddrRanges();
    for (auto address : adl)
      if (address.contains(add) && !(port->hasRetryPackets()))
        return port;
      else if (debug())
        DPRINTF(WindowManager, "I have got %d retry packets bruv\n",
                port->retryPackets.size());
  }
  return nullptr;
}

WindowManager::SPMPort *WindowManager::getValidSPMPort(Addr add, bool read) {
  for (auto port : spmPorts) {
    AddrRangeList adl = port->getAddrRanges();
    for (auto address : adl)
      if (!port->isActive() && address.contains(add) &&
          !(port->hasRetryPackets()))
        return port;
      else if (debug())
        DPRINTF(WindowManager, "I have got %d retry packets bruv\n",
                port->retryPackets.size());
  }
  return nullptr;
}

uint64_t WindowManager::extractPERequestValue(MemoryRequest *read_req) {
  size_t read_len = read_req->getLength();
  if (read_len == peRequestLength) { // 8 bytes
    assert(read_req->getTotalLength() == (Tick)read_len);

    uint64_t result = 0;
    for (int i = read_req->getTotalLength() - 1; i >= 0; i--)
      result = (result << 8) + read_req->getBuffer()[i];

    return result;
  }

  panic("Can't handle read lengths other than 8 for now!\n");
  return 0;
}

void WindowManager::scheduleEvent(Tick when) {
  Tick actual_when = (when == 0) ? curTick() + processingDelay : when;
  if (!tickEvent.scheduled())
    schedule(tickEvent, actual_when);
}
/* Utility functions */

/* Helper functions start */
std::string WindowManager::PEPort::getPENameFromPeerPort() {
  // TODO: find a way to get the PE name from the port
  if (!isConnected())
    panic("Port not connected");

  return std::to_string(getId());
}

bool WindowManager::checkPort(RequestPort *port, size_t len, bool is_read) {
  if (PEPort *pePort = dynamic_cast<PEPort *>(port))
    return pePort->streamValid(len, is_read);
  else
    panic("currently no support for validation check other than PE port\n");

  return false;
}

Port &WindowManager::getPort(const std::string &if_name, PortID idx) {
  if (if_name == "local") {
    if (idx >= localPorts.size())
      localPorts.resize((idx + 1));
    if (localPorts[idx] == nullptr) {
      const std::string portName = name() + csprintf(".local[%d]", idx);
      localPorts[idx] = new LocalPort(portName, this, idx);
    }
    return *localPorts[idx];
  } else if (if_name == "acp") {
    if (idx >= globalPorts.size())
      globalPorts.resize((idx + 1));
    if (globalPorts[idx] == nullptr) {
      const std::string portName = name() + csprintf(".acp[%d]", idx);
      globalPorts[idx] = new GlobalPort(portName, this, idx);
    }
    return *globalPorts[idx];
  } else if (if_name == "spm") {
    if (idx >= spmPorts.size())
      spmPorts.resize((idx + 1));
    if (spmPorts[idx] == nullptr) {
      const std::string portName = name() + csprintf(".spm[%d]", idx);
      spmPorts[idx] = new SPMPort(portName, this, idx);
    }
    return *spmPorts[idx];
  } else if (if_name == "pe_req_stream_ports") {
    if (idx >= peRequestStreamPorts.size())
      peRequestStreamPorts.resize((idx + 1));
    if (peRequestStreamPorts[idx] == nullptr) {
      const std::string portName =
          name() + csprintf(".pe_req_stream_ports[%d]", idx);
      peRequestStreamPorts[idx] = new PEPort(portName, this, idx);
    }
    return *peRequestStreamPorts[idx];
  } else if (if_name == "pe_resp_stream_ports") {
    if (idx >= peResponseStreamPorts.size())
      peResponseStreamPorts.resize((idx + 1));
    if (peResponseStreamPorts[idx] == nullptr) {
      const std::string portName =
          name() + csprintf(".pe_resp_stream_ports[%d]", idx);
      peResponseStreamPorts[idx] = new PEPort(portName, this, idx);
    }
    return *peResponseStreamPorts[idx];
  } else {
    return BasicPioDevice::getPort(if_name, idx);
  }
}

MemoryRequest *
WindowManager::findMemRequest(PacketPtr pkt,
                              const std::vector<MemoryRequest *> &target_vec) {
  auto it =
      find_if(begin(target_vec), end(target_vec),
              [&pkt](MemoryRequest *mr) { return mr->getPacket() == pkt; });

  if (it != end(target_vec))
    return *it;

  panic("Could not find memory request in request queues");
  return nullptr;
}

void WindowManager::removeRequest(MemoryRequest *mem_req,
                                  std::vector<MemoryRequest *> &target_vec) {
  if (debug())
    DPRINTF(WindowManager, "Clearing Request with addr 0x%016x\n",
            mem_req->getAddress());

  for (auto it = target_vec.begin(); it != target_vec.end(); ++it) {
    if ((*it) == mem_req) {
      it = target_vec.erase(it);
      return;
    }
  }

  // TODO: is it ok if the request is not present?
  if (debug())
    DPRINTF(WindowManager,
            "Did not find the memory request with 0x%016x to remove!\n",
            mem_req->getAddress());

  // TODO This is really weird. The code below should work but it doesn't.
  // auto it =
  //     find_if(begin(target_vec), end(target_vec),
  //             [&memReq](const MemoryRequest * mr) { return mr == memReq; });
  // if (it != end(target_vec))
  //   readMemReqs.erase(it);
  // else
  //   DPRINTF(WindowManager,
  //           "Could not find memory request in request queues (for
  //           removal)\n");
}

WindowManager::SignalWindow *
WindowManager::findCorrespondingSignalWindow(PacketPtr pkt) {
  for (const auto &elem : activeSignalWindowRequests)
    for (const auto &req : elem.second)
      if (req->getPacket() == pkt)
        return elem.first;

  panic("Could not find memory request in active window requests map\n");
  return nullptr;
}

void WindowManager::removeSignalWindowRequest(MemoryRequest *mem_req) {
  if (debug())
    DPRINTF(WindowManager, "Clearing Request with addr 0x%016x\n",
            mem_req->getAddress());

  for (const auto &elem : activeSignalWindowRequests) {
    for (auto it = elem.second.begin(); it != elem.second.end(); ++it) {
      if (*it == mem_req) {
        activeSignalWindowRequests[elem.first].erase(it);
        return;
      }
    }
  }

  // TODO: is it ok if the request is not present?
  if (debug())
    DPRINTF(WindowManager,
            "Did not find the memory request with 0x%016x to remove!\n",
            mem_req->getAddress());
}

WindowManager::TimeseriesWindow *
WindowManager::findCorrespondingTimeseriesWindow(PacketPtr pkt) {
  for (const auto &elem : activeTimeseriesWindowRequests)
    for (const auto &req : elem.second)
      if (req->getPacket() == pkt)
        return elem.first;

  panic("Could not find memory request in active window requests map\n");
  return nullptr;
}

void WindowManager::removeTimeseriesWindowRequest(MemoryRequest *mem_req) {
  if (debug())
    DPRINTF(WindowManager, "Clearing Request with addr 0x%016x\n",
            mem_req->getAddress());

  for (const auto &elem : activeTimeseriesWindowRequests) {
    for (auto it = elem.second.begin(); it != elem.second.end(); ++it) {
      if (*it == mem_req) {
        activeTimeseriesWindowRequests[elem.first].erase(it);
        return;
      }
    }
  }

  // TODO: is it ok if the request is not present?
  if (debug())
    DPRINTF(WindowManager,
            "Did not find the memory request with 0x%016x to remove!\n",
            mem_req->getAddress());
}
/* Helper functions end */

/* GenericRequestPort functions start */
bool WindowManager::GenericRequestPort::recvTimingResp(PacketPtr pkt) {
  owner->recvPacket(pkt);
  return true;
}

void WindowManager::GenericRequestPort::recvReqRetry() {
  if (debug())
    DPRINTF(WindowManager, "Got a retry...\n");

  while (retryPackets.size() && sendTimingReq(retryPackets.front())) {
    if (debug())
      DPRINTF(WindowManager, "Unblocked, sent blocked packet.\n");
    retryPackets.pop();

    owner->scheduleEvent();
  }
}

void WindowManager::GenericRequestPort::sendPacket(PacketPtr pkt) {
  if (hasRetryPackets() || !sendTimingReq(pkt)) {
    if (debug())
      DPRINTF(WindowManager,
              "sendTiming failed in sendPacket(pkt->req->getPaddr()=0x%x "
              "size=%d)\n",
              (unsigned int)pkt->req->getPaddr(), pkt->req->getSize());
    addRetryPacket(pkt);
  }
}
/* GenericRequestPort functions end */

/* SPMPort functions start */
bool WindowManager::SPMPort::recvTimingResp(PacketPtr pkt) {
  owner->recvPacket(pkt);
  return true;
}

void WindowManager::SPMPort::recvReqRetry() {
  if (debug())
    DPRINTF(WindowManager, "Got a retry...\n");

  while (retryPackets.size() && sendTimingReq(retryPackets.front())) {
    if (debug())
      DPRINTF(WindowManager, "Unblocked, sent blocked packet.\n");
    retryPackets.pop();

    owner->scheduleEvent();
  }
}

void WindowManager::SPMPort::sendPacket(PacketPtr pkt) {
  if (hasRetryPackets() || !sendTimingReq(pkt)) {
    if (debug())
      DPRINTF(WindowManager,
              "sendTiming failed in sendPacket(pkt->req->getPaddr()=0x%x "
              "size=%d)\n",
              (unsigned int)pkt->req->getPaddr(), pkt->req->getSize());
    addRetryPacket(pkt);
  }
}
/* SPMPort functions end */
