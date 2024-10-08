#include "hwacc/window_manager.hh"
#include "base/trace.hh"
#include "debug/WindowManager.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "sim/system.hh"
#include <algorithm>
#include <iterator>
#include <random>
#include <string>

WindowManager::WindowManager(const WindowManagerParams &p)
    : BasicPioDevice(p, p.pio_size), tickEvent(this), deviceName(p.devicename),
      debugEnabled(p.debug_enabled), clockPeriod(p.clock_period),
      masterId(p.system->getRequestorId(this, name())), io_size(p.pio_size),
      io_addr(p.pio_addr), endian(p.system->getGuestByteOrder()) {
  processingDelay = 1000 * clockPeriod;

  mmr = new uint8_t[io_size];
  std::fill(mmr, mmr + io_size, 0);

  numCreatedSignalWindows = 0;

  numCacheAccesses = 0;
  numCacheHits = 0;
  numCacheMisses = 0;
  numCacheReplacements = 0;
  numCacheInsertions = 0;
  avgSavedPortion = 0;
  maxCacheIndex = ((size_t)(spmSize / cacheEntrySize - 1));

  // dummy values for testing!!!
  // timeseriesCache.push_back(std::make_tuple(0, (TimeseriesRange){0, 63}, 1));
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
        handleSignalStreamReadResponse(read_req, pe_port);
      } else if (isTimeseriesMode()) {
        handleTimeseriesPEResponse(read_req, pe_port);
      }
    } else if (GlobalPort *global_port =
                   dynamic_cast<GlobalPort *>(carrier_port)) {
      global_port->waitingResponsePkt = nullptr;
      if (isSignalMode()) {
        // handleSignalMemoryResponse(pkt, read_req);
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
      // removeSignalWindowRequest(write_req);
    } else if (PEPort *port = dynamic_cast<PEPort *>(carrier_port)) {
    } else if (GlobalPort *port = dynamic_cast<GlobalPort *>(carrier_port)) {
      port->waitingResponsePkt = nullptr;
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

/* ************************************************************************** */
/* ************************************************************************** */
/* ************************************************************************** */
/* ************************************************************************** */
/* ************************************************************************** */

/* Timeseries-related functions */
WindowManager::TimeseriesWindow::TimeseriesWindow(WindowManager *owner,
                                                  PEPort *pe_port, size_t id)
    : state(none), windowName("TimeseriesWindow_" + std::to_string(id)),
      owner(owner), correspondingPEPort(pe_port), numReceivedChildren(0),
      currentCoreAddr(0), currentComputationCoreAddr(0), numCheckedNodes(0),
      connectedCalcPort(nullptr) {
  if (debug())
    DPRINTF(WindowManager, "maxCacheIndex %d\n", owner->maxCacheIndex);
}

void WindowManager::TimeseriesWindow::checkCacheFunction() {
  if (debug())
    DPRINTF(WindowManager, "Checking the cache for the initial query %d %d\n",
            initialQuery.start, initialQuery.end);

  owner->numCacheAccesses++;

  // step 1: add all entries inside the initial query
  std::vector<TimeseriesRange> nominated_ranges;
  if (useIdealCache()) {
    size_t num_ll_nodes = (leafCoresEndAddr - leafCoresStartAddr) /
                          ((numChildren + 3) * owner->tsDataSize);
    size_t ll_node_range = batchSize * numChildren;
    for (size_t i = 0; i < num_ll_nodes; i++) {
      uint64_t entry_start = i * ll_node_range;
      uint64_t entry_end = entry_start + ll_node_range - 1;

      if (entry_start >= initialQuery.start && entry_end <= initialQuery.end) {
        nominated_ranges.push_back((TimeseriesRange){entry_start, entry_end});
        endResults.push_back(1);
      }
    }
  } else {
    for (auto &entry : owner->timeseriesCache) {
      uint64_t entry_start = entry.range.start;
      uint64_t entry_end = entry.range.end;

      if (entry_start >= initialQuery.start && entry_end <= initialQuery.end) {
        nominated_ranges.push_back(entry.range);
      }
    }
  }

  // no nominated range -> missed
  if (nominated_ranges.empty()) {
    if (debug())
      DPRINTF(WindowManager, "found no hit ranges inside cache\n");

    subQueries.push_back(initialQuery);
    owner->numCacheMisses++;
    owner->missedRanges[initialQuery]++;
    return;
  }

  // step 2: select largest entries
  std::vector<TimeseriesRange> largest_ranges;
  for (auto &e1 : nominated_ranges) {
    bool is_large = true;
    for (auto &e2 : nominated_ranges) {
      if ((e2.start < e1.start && e2.end > e1.end) ||
          (e2.start == e1.start && e2.end > e1.end) ||
          (e2.start < e1.start && e2.end == e1.end)) {
        is_large = false;
        break;
      }
    }
    if (is_large) {
      largest_ranges.push_back(e1);
    }
  }

  // step 2.5: sort largest entries
  std::sort(largest_ranges.begin(), largest_ranges.end());

  // step 3: add hit results
  float saved_portion = 0;
  for (auto &e : largest_ranges) {
    for (auto &entry : owner->timeseriesCache) {
      TimeseriesRange entry_range = entry.range;
      if (e == entry_range) {
        if (debug())
          DPRINTF(WindowManager,
                  "found a covered range inside cache %d-%d diff %d\n",
                  entry_range.start, entry_range.end,
                  entry_range.end - entry_range.start);

        entry.numAccesses = entry.numAccesses + 1;
        endResults.push_back(entry.stat);

        saved_portion += (float)entry.range.end - (float)entry.range.start;
        owner->numCacheHits++;
        owner->hitRanges[entry_range]++;
        break;
      }
    }
  }
  saved_portion /= (float)initialQuery.end - (float)initialQuery.start;
  if (debug())
    DPRINTF(WindowManager, "cache saved %f percent of the initial query\n",
            saved_portion * 100);
  owner->avgSavedPortion += 100 * saved_portion;

  // step 4: make subQueries
  uint64_t start = initialQuery.start;
  for (auto &e : largest_ranges) {
    if (e.start > start) {
      if (debug())
        DPRINTF(WindowManager, "added sub query %d-%d\n", start, e.start - 1);
      subQueries.push_back((TimeseriesRange){start, e.start - 1});
    }
    start = e.end + 1;
  }
  if (start < initialQuery.end) {
    if (debug())
      DPRINTF(WindowManager, "added sub query %d-%d\n", start,
              initialQuery.end);
    subQueries.push_back((TimeseriesRange){start, initialQuery.end});
  }
}

void WindowManager::TimeseriesWindow::saveCacheFunction(Addr cc_address,
                                                        uint64_t cc_stat) {
  if (useIdealCache())
    return;

  TimeseriesRange cc_range = owner->ccAddressToRange[cc_address];
  if (debug())
    DPRINTF(WindowManager, "saving 0x%016x %d-%d stat 0x%016x to cache\n",
            cc_address, cc_range.start, cc_range.end, cc_stat);

  // case 1: cc address already in cache -> increase hit number
  for (auto &entry : owner->timeseriesCache) {
    if (entry.range == cc_range) {
      if (debug())
        DPRINTF(WindowManager,
                "request address already exists in cache; updating\n");

      entry.numAccesses = entry.numAccesses + 1;
      entry.stat = cc_stat;
      return;
    }
  }

  // limited special case: do not cache very small ranges
  uint64_t cc_diff = cc_range.end - cc_range.start;
  if (useLimitedCache() && cc_diff < limitedCacheMinDifference) {
    if (debug())
      DPRINTF(WindowManager, "cannot cache very small ranges (lower than %d)\n",
              limitedCacheMinDifference);
    return;
  }

  // case 2: cc not in cache; cache has space -> add to cache
  if (!owner->isCacheFull()) {
    if (debug())
      DPRINTF(WindowManager, "cache not full; found empty entry\n");

    owner->timeseriesCache.push_back((CacheEntry){1, cc_range, cc_stat});
    owner->numCacheInsertions++;
    return;
  }

  // TODO: improve implementation
  // case 3: cc not in cache; cache full -> replace
  std::vector<TimeseriesRange> low_access_entries;
  // case 3.1: find min number of accesses
  size_t min_num_accesses =
      (*std::min_element(owner->timeseriesCache.begin(),
                         owner->timeseriesCache.end(),
                         [](const CacheEntry &e1, const CacheEntry &e2) {
                           return e1.numAccesses < e2.numAccesses;
                         }))
          .numAccesses;
  assert(min_num_accesses > 0);
  if (debug())
    DPRINTF(WindowManager, "in saving to cache, min num accesses in %d\n",
            min_num_accesses);
  // case 3.2: fill the nominated list
  for (auto &entry : owner->timeseriesCache) {
    size_t entry_num_accesses = entry.numAccesses;
    if (entry_num_accesses == min_num_accesses)
      low_access_entries.push_back(entry.range);
  }
  // case 3.3: sort nominated list
  // current algo: based on range size
  // TODO: better algos
  std::sort(low_access_entries.begin(), low_access_entries.end(),
            [](const TimeseriesRange &lhs, const TimeseriesRange &rhs) {
              return (lhs.end - lhs.start) < (rhs.end - rhs.start);
            });
  if (debug())
    DPRINTF(WindowManager,
            "in saving to cache, smallest range %d-%d largest range %d-%d\n",
            low_access_entries.front().start, low_access_entries.front().end,
            low_access_entries.back().start, low_access_entries.back().end);
  // case 3.4: replace smallest range ONLY IF your range is larger
  uint64_t smallest_range_distance =
      low_access_entries.front().end - low_access_entries.front().start + 1;
  uint64_t cc_range_distance = cc_range.end - cc_range.start + 1;
  if (cc_range_distance < smallest_range_distance) {
    if (debug())
      DPRINTF(WindowManager, "cannot replace with a lower range\n");
    return;
  }

  for (auto &entry : owner->timeseriesCache) {
    if (entry.range == low_access_entries.front()) {
      if (debug())
        DPRINTF(WindowManager, "cache full; replaced smallest range\n");

      entry.numAccesses = 1;
      entry.range = cc_range;
      entry.stat = cc_stat;

      owner->numCacheReplacements++;
      owner->replacedRanges[entry.range]++;
      return;
    }
  }
}

void WindowManager::TimeseriesWindow::firstScanTick() {
  if (state == checkCache) {
    if (useCache()) {
      checkCacheFunction();
      if (subQueries.empty()) { // all the query is hit
        if (debug())
          DPRINTF(WindowManager, "ALL THE QUERY IS INSIDE CACHE\n");
        state = initTraverse;
        return;
      }
    } else {
      subQueries.push_back(initialQuery);
    }

    coreQueue.push(tsCoresStartAddr);
    state = requestCoreStart;
  } else if (state == requestCoreStart) {
    if (debug())
      DPRINTF(WindowManager, "State: requestCoreStart\n");

    if (coreQueue.empty()) {
      if (childrenPointers.empty()) { // done
        state = firstScanDone;
      } else { // have to cover children
        state = requestChildAddr;
      }
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
    for (auto &sub_q : subQueries) {
      uint64_t inp_start = sub_q.start;
      uint64_t inp_end = sub_q.end;
      double node_start, node_end;
      memcpy(&node_start, &currentCoreStart, sizeof(node_start));
      memcpy(&node_end, &currentCoreEnd, sizeof(node_end));

      if (debug())
        DPRINTF(WindowManager,
                "Checking range inp_start:%d inp_end:%d against node_start:%f "
                "node_end:%f\n",
                inp_start, inp_end, node_start, node_end);

      assert(node_start < node_end);

      bool is_leaf = isLeafNode(currentCoreAddr);
      bool case_skip = (node_start > inp_end) || (node_end < inp_start);
      bool case_add_to_list = (inp_start <= node_start && node_end <= inp_end);

      if (debug())
        DPRINTF(WindowManager, "is leaf %d skip %d add %d\n", is_leaf,
                case_skip, case_add_to_list);

      if (case_skip) { // skip
        state = requestCoreStart;
      } else if (case_add_to_list) { // add to list
        if (std::find(computationCores.begin(), computationCores.end(),
                      currentCoreAddr) == computationCores.end()) {
          if (debug())
            DPRINTF(WindowManager, "added to CCs: 0x%016x %f-%f diff %d\n",
                    currentCoreAddr, node_start, node_end,
                    currentCoreEnd - currentCoreStart);
          computationCores.push_back(currentCoreAddr);
          owner->ccAddressToRange[currentCoreAddr] =
              (TimeseriesRange){(uint64_t)node_start, (uint64_t)node_end};
        }
        state = requestCoreStart;
      } else if (!is_leaf) { // non-leaf & children
        if (debug())
          DPRINTF(WindowManager, "children of 0x%016x %f-%f are needed\n",
                  currentCoreAddr, node_start, node_end);

        // doing some high-level analytics to reduce the number of children
        double current_node_size = node_end - node_start + 1;
        double child_size = current_node_size / numChildren;
        for (size_t i = 0; i < numChildren; i++) {
          double child_start = node_start + i * child_size;
          double child_end = child_start + child_size - 1;
          bool skip_child = (child_end < inp_start) || (child_start > inp_end);
          if (skip_child) {
            if (debug())
              DPRINTF(WindowManager, "skipped child #%d %f-%f\n", i,
                      child_start, child_end);
            continue;
          }

          Addr child_pointer = currentCoreAddr + (i + 3) * owner->tsDataSize;
          if (std::find(childrenPointers.begin(), childrenPointers.end(),
                        child_pointer) == childrenPointers.end()) {
            childrenPointers.push_back(child_pointer);
          }
        }
        state = requestCoreStart;
      } else { // leaf & partial
        double partial_start = fmax(inp_start, node_start);
        double partial_diff = fabs(partial_start - fmin(inp_end, node_end));

        Addr partial_start_addr =
            currentCoreAddr +
            ((uint64_t)(partial_start - node_start)) * coreStatOffset;
        uint64_t partial_length = (uint64_t)partial_diff + 1;

        if (debug())
          DPRINTF(WindowManager,
                  "added to partials 0x%016x, %d while on %f-%f\n",
                  partial_start_addr, partial_length, node_start, node_end);

        partials.push(std::make_pair(partial_start_addr, partial_length));
        state = requestCoreStart;
      }
    }

    if (!coreQueue.empty())
      coreQueue.pop();
  } else if (state == requestChildAddr) {
    if (debug())
      DPRINTF(WindowManager, "State: requestChildAddr\n");

    Addr child_pointer = childrenPointers.front();

    if (debug())
      DPRINTF(WindowManager, "Requesting address for pointer 0x%016x\n",
              child_pointer);

    if (GlobalPort *memory_port =
            owner->getValidGlobalPort(child_pointer, true)) {
      MemoryRequest *read_req =
          owner->readFromPort(memory_port, child_pointer, owner->tsDataSize);
      owner->activeTimeseriesWindowRequests[this].push_back(read_req);
      state = waitingChildAddr;
    } else {
      if (debug())
        DPRINTF(WindowManager,
                "Did not find a global port to read %d bytes from 0x%016x\n",
                owner->tsDataSize, child_pointer);
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
      computationCores.pop_front();

      state = requestStat;
    }
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

    assert(connectedCalcPort == nullptr);
    connectedCalcPort = findFreeCalcUnitPort();
    if (connectedCalcPort == nullptr) {
      if (debug())
        DPRINTF(WindowManager,
                "Did not find a calc unit port for the compute state\n");
      return;
    }
    Addr port_addr = connectedCalcPort->getStartAddr();
    owner->setCalcUnitStatus(port_addr, true);
    if (debug())
      DPRINTF(WindowManager, "setting this calc 0x%016x BUSY\n", port_addr);

    if (coresDone() &&
        currentPartial != std::pair<Addr, uint64_t>()) { // partial calc
      if (debug())
        DPRINTF(WindowManager, "calculating partial start 0x%016x and len %d\n",
                currentPartial.first, currentPartial.second);

      MemoryRequest *write_req =
          owner->writeToPort(connectedCalcPort, (uint64_t)1, port_addr);
      owner->activeTimeseriesWindowRequests[this].push_back(write_req);

      write_req = owner->writeToPort(connectedCalcPort, currentPartial.second,
                                     port_addr);
      owner->activeTimeseriesWindowRequests[this].push_back(write_req);

      if (owner->fakeValues) {
        write_req =
            owner->writeToPort(connectedCalcPort, leafCoresEndAddr, port_addr);
        owner->activeTimeseriesWindowRequests[this].push_back(write_req);
      } else {
        write_req = owner->writeToPort(connectedCalcPort, currentPartial.first,
                                       port_addr);
        owner->activeTimeseriesWindowRequests[this].push_back(write_req);
      }

      state = computeStat_RequestResult;
    } else if (isLeafNode(traverseStackHead.first)) { // leaf node
      if (debug())
        DPRINTF(WindowManager, "calculating leaf node\n");

      MemoryRequest *write_req =
          owner->writeToPort(connectedCalcPort, (uint64_t)1, port_addr);
      owner->activeTimeseriesWindowRequests[this].push_back(write_req);

      write_req = owner->writeToPort(
          connectedCalcPort, (uint64_t)(batchSize * numChildren), port_addr);
      owner->activeTimeseriesWindowRequests[this].push_back(write_req);

      state = computeStat_RequestLeafValuesStartAddr;
    } else { // non-leaf node
      if (debug())
        DPRINTF(WindowManager, "calculating core node\n");

      MemoryRequest *write_req =
          owner->writeToPort(connectedCalcPort, (uint64_t)2, port_addr);
      owner->activeTimeseriesWindowRequests[this].push_back(write_req);

      write_req = owner->writeToPort(connectedCalcPort, (uint64_t)numChildren,
                                     port_addr);
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
    // finding the matching request port
    assert(connectedCalcPort != nullptr);
    int port_idx = std::distance(owner->responseStreamPorts.begin(),
                                 std::find(owner->responseStreamPorts.begin(),
                                           owner->responseStreamPorts.end(),
                                           connectedCalcPort));
    assert(port_idx < owner->requestStreamPorts.size());
    PEPort *calc_req_port = owner->requestStreamPorts[port_idx];
    assert(calc_req_port != nullptr);

    if (owner->checkPort(calc_req_port, owner->peRequestLength, true)) {
      if (debug())
        DPRINTF(WindowManager, "State: computeStat_RequestResult; found valid "
                               "port to read from\n");

      MemoryRequest *read_req = owner->readFromPort(
          calc_req_port, calc_req_port->getStartAddr(), owner->peRequestLength);
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

  else if (state == done) {
  }
}

void WindowManager::timeseriesTick() {
  if (calcStatus.empty()) {
    for (auto &addr : calcAddresses) {
      calcStatus[addr] = false;
    }
  }

  if (perPERequests.empty()) {
    // filling up requests!!!!!!!!!!
    std::fstream file_stream(
        "/local-scratch/localhome/mza148/Documents/Research/"
        "codebase/gem5-SALAM-app/ts/req.txt",
        std::ios_base::in);
    uint64_t a, b;
    while (file_stream >> a) {
      file_stream >> b;
      requests.push((TimeseriesRange){a, b});
    }
    DPRINTF(WindowManager, "requests size %d\n", requests.size());

    for (auto &req_pe_addr : reqPEAddresses) {
      do {
        perPERequests[req_pe_addr].push_back(requests.front());
        requests.pop();
      } while (perPERequests[req_pe_addr].size() < numPerPERequests);
    }
    DPRINTF(WindowManager, "done making per pe requests %d\n",
            perPERequests.size());
    DPRINTF(WindowManager, "requests size %d\n", requests.size());
  }

  if (activeTimeseriesWindows.empty()) {
    for (auto &pair : perPERequests) {
      PEPort *corrensponding_pe_port = nullptr;
      for (auto &req_pe_port : requestStreamPorts) {
        if (req_pe_port->getStartAddr() == pair.first) {
          corrensponding_pe_port = req_pe_port;
        }
      }
      assert(corrensponding_pe_port != nullptr);

      for (auto &pe_req : pair.second) {
        TimeseriesWindow *w = new TimeseriesWindow(
            this, corrensponding_pe_port, activeTimeseriesWindows.size());
        w->setTimeseriesPERequest(pe_req.start);
        w->setTimeseriesPERequest(pe_req.end);
        // window should not already exist in active windows
        assert(std::find(activeTimeseriesWindows.begin(),
                         activeTimeseriesWindows.end(),
                         w) == activeTimeseriesWindows.end());
        activeTimeseriesWindows.push_back(w);
      }
    }
  }

  // finishing conditions
  if (!activeTimeseriesWindows.empty() && allTimeseriesWindowsDone()) {
    DPRINTF(WindowManager, "ALL FINISHED!!!\n");
    DPRINTF(WindowManager, "Cache Stats:\n");
    DPRINTF(WindowManager, "Number of cache accesses: %lu\n", numCacheAccesses);
    DPRINTF(WindowManager, "Number of cache hits: %lu\n", numCacheHits);
    DPRINTF(WindowManager, "Number of cache misses: %lu\n", numCacheMisses);
    DPRINTF(WindowManager, "Number of cache insertions: %lu\n",
            numCacheInsertions);
    DPRINTF(WindowManager, "Number of cache replacements: %lu\n",
            numCacheReplacements);
    DPRINTF(WindowManager, "Avg saved portion: %f\n",
            avgSavedPortion / (reqPEAddresses.size() * numPerPERequests));

    DPRINTF(WindowManager, "cache dist\n");
    for (auto &entry : timeseriesCache) {
      DPRINTF(WindowManager, "number of accesses: %d\n", entry.numAccesses);
      DPRINTF(WindowManager, "range: %d-%d\n", entry.range.start,
              entry.range.end);
      DPRINTF(WindowManager, "stat: %d\n", entry.stat);
    }

    for (auto &e : missedRanges) {
      DPRINTF(WindowManager, "miss range %d-%d count %d\n", e.first.start,
              e.first.end, e.second);
    }
    for (auto &e : hitRanges) {
      DPRINTF(WindowManager, "hit range %d-%d count %d\n", e.first.start,
              e.first.end, e.second);
    }
    for (auto &e : replacedRanges) {
      DPRINTF(WindowManager, "replace range %d-%d count %d\n", e.first.start,
              e.first.end, e.second);
    }

    *mmr &= 0xfc;
    *mmr |= 0x04;
    return;
  }

  // read from PEs (start, end)
  // for (PEPort *port : requestStreamPorts) {
  //   if (std::find(reqPEAddresses.begin(), reqPEAddresses.end(),
  //                 port->getStartAddr()) != reqPEAddresses.end() &&
  //       checkPort(port, peRequestLength, true)) {
  //     readFromPort(port, port->getStartAddr(), peRequestLength);
  //   }
  // }

  // advance timeseries windows by one
  for (auto &pair : requestPEActiveTimeseriesWindows) {
    size_t num_window_forwards = 0;
    for (auto &w : pair.second) {
      if (!w->isDone()) {
        w->firstScanTick();
        w->traverseTick();
        num_window_forwards++;
        if (num_window_forwards == perRequestPEConcurrentTimeseriesWindows)
          break;
      }
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

  // // case 2: finding and finishing up the incomplete window
  // uint64_t value = extractPERequestValue(read_req);
  // for (auto &w : activeTimeseriesWindows) {
  //   if (w->samePEPort(pe_port) && w->waitingToStart()) {
  //     w->setTimeseriesPERequest(value);
  //     return;
  //   }
  // }

  // // case 3: no previous windows matching this port => new port request
  // TimeseriesWindow *w =
  //     new TimeseriesWindow(this, pe_port, activeTimeseriesWindows.size());
  // w->setTimeseriesPERequest(value);
  // // window should not already exist in active windows
  // assert(std::find(activeTimeseriesWindows.begin(),
  //                  activeTimeseriesWindows.end(),
  //                  w) == activeTimeseriesWindows.end());
  // activeTimeseriesWindows.push_back(w);
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
    childrenPointers.pop_front();

    if (childrenPointers.empty()) {
      state = requestCoreStart;
    } else {
      state = requestChildAddr;
    }
  }

  else if (state == waitingStat) {
    if (debug())
      DPRINTF(WindowManager, "State: waitingStat\n");

    if (data == 0) {
      state = startTraverse;
    } else { // stat is present.
      traverseStack.pop();

      if (traverseStack.empty()) {
        if (debug())
          DPRINTF(WindowManager,
                  "done one computation core with present stat\n");

        if (useCache())
          saveCacheFunction(currentComputationCoreAddr, data);

        endResults.push_back(data);
        state = initTraverse;
      }

      if (numCheckedNodes == 1) { // root has stat
        numCheckedNodes = 0;
        currentComputationCoreAddr = 0;
        state = initTraverse;
      } else if (numCheckedNodes > 1) { // non-root has stat
        if (debug())
          DPRINTF(WindowManager, "State: waitingStat; the stat is 0x%016x\n",
                  data);
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

    assert(connectedCalcPort != nullptr);

    if (owner->fakeValues) {
      MemoryRequest *write_req =
          owner->writeToPort(connectedCalcPort, leafCoresEndAddr,
                             connectedCalcPort->getStartAddr());
      owner->activeTimeseriesWindowRequests[this].push_back(write_req);
    } else {
      MemoryRequest *write_req = owner->writeToPort(
          connectedCalcPort, data, connectedCalcPort->getStartAddr());
      owner->activeTimeseriesWindowRequests[this].push_back(write_req);
    }
    // for (Addr offset = 0; offset < batchSize * numChildren; offset++) {
    //   Addr next_record_addr = data + (2 * offset + 1) * owner->tsDataSize;
    //   MemoryRequest *write_req =
    //       owner->writeToPort(connectedCalcPort, next_record_addr,
    //                          connectedCalcPort->getStartAddr());
    //   owner->activeTimeseriesWindowRequests[this].push_back(write_req);
    // }

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

    assert(connectedCalcPort != nullptr);
    MemoryRequest *write_req = owner->writeToPort(
        connectedCalcPort, data, connectedCalcPort->getStartAddr());
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
    assert(connectedCalcPort != nullptr);
    owner->setCalcUnitStatus(connectedCalcPort->getStartAddr(), false);
    if (debug())
      DPRINTF(WindowManager, "setting this calc 0x%016x FREE\n",
              connectedCalcPort->getStartAddr());
    connectedCalcPort = nullptr;

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

      // TODO: only adds CCs to cache
      if (useCache())
        saveCacheFunction(currentComputationCoreAddr, computedResult);

      currentComputationCoreAddr = 0;
      state = initTraverse;
    } else {
      computedResult = 0;
      state = requestStat;
    }
  }
}

bool WindowManager::TimeseriesWindow::isLeafNode(Addr node_addr) {
  // TODO: better leaf check logic
  if (debug())
    DPRINTF(WindowManager, "IS IT LEAF? S 0x%016x E 0x%016x Q 0x%016x ANS %d\n",
            leafCoresStartAddr, leafCoresEndAddr, node_addr,
            leafCoresStartAddr <= node_addr && node_addr < leafCoresEndAddr);
  return leafCoresStartAddr <= node_addr && node_addr < leafCoresEndAddr;
}

WindowManager::PEPort *WindowManager::TimeseriesWindow::findFreeCalcUnitPort() {
  Addr free_calc_unit_addr = owner->getFreeCalcUnitAddr();
  for (auto &port : owner->responseStreamPorts) {
    if (port->getStartAddr() == free_calc_unit_addr) {
      if (debug())
        DPRINTF(WindowManager, "found a free calc unit with addr 0x%016x\n",
                free_calc_unit_addr);
      return port;
    }
  }
  return nullptr;
}

void WindowManager::handleTimeseriesMemoryResponse(PacketPtr pkt,
                                                   MemoryRequest *req) {
  TimeseriesWindow *window = findCorrespondingTimeseriesWindow(pkt);
  uint64_t data = *((uint64_t *)req->getBuffer());
  window->handleMemoryResponseData(data);
}

void WindowManager::TimeseriesWindow::setTimeseriesPERequest(uint64_t value) {
  if (initialQuery.start == UINT64_MAX)
    initialQuery.start = value;
  else if (initialQuery.end == UINT64_MAX) {
    initialQuery.end = value;

    if (debug())
      DPRINTF(WindowManager, "new TS window created with range: %d %d\n",
              initialQuery.start, initialQuery.end);

    owner->requestPEActiveTimeseriesWindows[correspondingPEPort->getStartAddr()]
        .push_back(this);
    if (debug())
      DPRINTF(WindowManager, "added this window to req PE addr 0x%016x\n",
              correspondingPEPort->getStartAddr());

    state = checkCache;
  } else
    panic("Calling setTimeseriesPERequest when you should not!\n");
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
  if (areStreamsDone()) {
    *mmr &= 0xfc;
    *mmr |= 0x04;
    return;
  }

  signalMainLogic();

  removeDoneSignalWindows();

  // check all the pe ports for incoming requests
  if (inputStreams.empty()) {
    inputStreams = std::vector<PEPort *>(requestStreamPorts.begin(),
                                         requestStreamPorts.begin() + 2);
    for (auto is : inputStreams)
      inputStreamsDone[is] = false;
  }

  for (PEPort *port : inputStreams) {
    if (checkPort(port, peRequestLength, true)) {
      readFromPort(port, port->getStartAddr(), peRequestLength);
    }
  }
}

void WindowManager::removeDoneSignalWindows() {
  for (auto it = activeSignalWindows.begin();
       it != activeSignalWindows.end();) {
    if ((*it)->isDone()) {
      if (debug())
        DPRINTF(WindowManager, "erased one done window %s\n", (*it)->name());
      it = activeSignalWindows.erase(it);
    } else {
      it++;
    }
  }
}

void WindowManager::signalMainLogic() {
  bool ready = signalWindowsReadyToHash();

  if (ready) {
    if (debug())
      DPRINTF(WindowManager, "can finally move forward!\n");

    auto hash_value = computeHash();

    auto ste = accessSignalTable(hash_value);
    if (ste.valid())
      signalTableHitHandler(ste);
    else
      signalTableMissHandler(hash_value);

    for (auto signal_window : activeSignalWindows) {
      if (debug())
        DPRINTF(WindowManager, "I AM DONE\n");
      signal_window->makeDone();
    }
  }
}

WindowManager::HashType WindowManager::computeHash() {
  // algo num 1: random number between [1, 10]
  HashType hash_value = 1 + rand() % (100 - 1 + 1);

  if (debug())
    DPRINTF(WindowManager, "the hash value is %d\n", hash_value);

  return hash_value;
}

WindowManager::SignalTableEntry
WindowManager::accessSignalTable(HashType hash_value) {
  auto it = find_if(begin(signalTable), end(signalTable),
                    [hash_value](SignalTableEntry const &ste) {
                      return ste.hashValue == hash_value;
                    });

  if (it != end(signalTable))
    return *it;

  return (SignalTableEntry){0, 0};
}

void WindowManager::signalTableHitHandler(SignalTableEntry ste) {
  // TODO: send the skipped output

  if (debug())
    DPRINTF(WindowManager, "table hit; STE: 0x%016x 0x%016x\n", ste.hashValue,
            ste.outputValue);
}

void WindowManager::signalTableMissHandler(HashType hash_value) {
  // TODO: send the window via output stream
  // TODO: automate output stream address and port

  if (signalTable.size() < numSignalTableEntries) {
    signalTable.push_back((SignalTableEntry){hash_value, 2019595501});

    if (debug())
      DPRINTF(WindowManager, "table has space; adding 0x%016x 0x%016x\n",
              hash_value, 2019595501);
  }

  if (debug())
    DPRINTF(WindowManager, "sending all windows\n");

  for (auto w : activeSignalWindows) {
    if (w->isFull()) {
      w->sendEntriesForward();
    }
  }
}

void WindowManager::SignalWindow::sendEntriesForward() {
  PEPort *output_port =
      owner->findCorrespondingStreamOutputPort(getCorrespondingPEPort());

  if (debug())
    DPRINTF(WindowManager, "sending all entries to output stream xd\n");

  for (auto e : entries) {
    owner->writeToPort(output_port, e.timestamp, output_port->getStartAddr());
    owner->writeToPort(output_port, e.value, output_port->getStartAddr());
  }
}

WindowManager::PEPort *
WindowManager::findCorrespondingStreamOutputPort(PEPort *pe_port) {
  PEPort *output_port =
      pe_port == requestStreamPorts[0]
          ? responseStreamPorts[0]
          : (pe_port == requestStreamPorts[1] ? responseStreamPorts[1]
                                              : nullptr);
  assert(output_port != nullptr);

  return output_port;
}

void WindowManager::SignalWindow::handleNewSignalStreamInput(uint64_t data) {
  if (data == stopToken) {
    if (debug())
      DPRINTF(WindowManager, "stop token; marking this window full\n");

    full = true;
    return;
  }

  if (entries.empty() || entries.back().isComplete()) {
    if (debug())
      DPRINTF(WindowManager, "new record\n");

    entries.push_back((SignalRecord){data, UINT64_MAX});
  } else if (entries.back().isPartial()) {
    if (debug())
      DPRINTF(WindowManager, "completing\n");

    entries.back().value = data;
  }
}

void WindowManager::handleSignalStreamReadResponse(MemoryRequest *read_req,
                                                   PEPort *pe_port) {
  uint64_t data = *((uint64_t *)read_req->getBuffer());

  if (debug())
    DPRINTF(WindowManager, "received new stream input 0x%016x : 0x%016x\n",
            pe_port->getStartAddr(), data);

  // case 0: end token
  if (data == endToken) {
    if (debug())
      DPRINTF(WindowManager, "received end token\n");

    PEPort *output_port = findCorrespondingStreamOutputPort(pe_port);
    writeToPort(output_port, endToken, output_port->getStartAddr());

    inputStreamsDone[pe_port] = true;
    return;
  }

  // case 1: another value for existing window
  for (auto &signal_window : activeSignalWindows) {
    if (signal_window->getCorrespondingPEPort() == pe_port) {
      if (debug())
        DPRINTF(WindowManager, "window already exists; appending\n");

      signal_window->handleNewSignalStreamInput(data);
      return;
    }
  }

  // case 2: a new window
  if (debug())
    DPRINTF(WindowManager, "new window creation\n");

  SignalWindow *signal_window =
      new SignalWindow(this, pe_port, numCreatedSignalWindows);
  activeSignalWindows.push_back(signal_window);
  signal_window->handleNewSignalStreamInput(data);
  numCreatedSignalWindows++;
}

WindowManager::SignalWindow::SignalWindow(WindowManager *owner, PEPort *pe_port,
                                          size_t id)
    : windowName("SignalWindow_" + std::to_string(id)), owner(owner),
      correspondingPEPort(pe_port), done(false), full(false) {}
/* Signal-related functions */

void WindowManager::tick() {
  if (*mmr == 0x04)
    return;

  if (debug())
    DPRINTF(WindowManager, "Tick!\n");

  // printPortRanges();

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
    assert(!memory_port->waitingForResponse());
    memory_port->waitingResponsePkt = pkt;

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
    assert(!port->waitingForResponse());
    port->waitingResponsePkt = pkt;

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

template <typename Iter, typename RandomGenerator>
Iter select_randomly(Iter start, Iter end, RandomGenerator &g) {
  std::uniform_int_distribution<> dis(0, std::distance(start, end) - 1);
  std::advance(start, dis(g));
  return start;
}

template <typename Iter> Iter select_randomly(Iter start, Iter end) {
  static std::random_device rd;
  static std::mt19937 gen(rd());
  return select_randomly(start, end, gen);
}

WindowManager::GlobalPort *WindowManager::getValidGlobalPort(Addr add,
                                                             bool read) {
  while (1) {
    auto port = *select_randomly(globalPorts.begin(), globalPorts.end());
    AddrRangeList adl = port->getAddrRanges();
    for (auto address : adl)
      if (address.contains(add) && !(port->hasRetryPackets()) &&
          !port->waitingForResponse())
        return port;
  }

  // for (auto port : globalPorts) {
  //   AddrRangeList adl = port->getAddrRanges();
  //   for (auto address : adl)
  //     if (address.contains(add) && !(port->hasRetryPackets()) &&
  //         !port->waitingForResponse())
  //       return port;
  // }
  // return nullptr;
}

WindowManager::SPMPort *WindowManager::getValidSPMPort(Addr add, bool read) {
  for (auto port : spmPorts) {
    AddrRangeList adl = port->getAddrRanges();
    for (auto address : adl)
      if (!port->isActive() && address.contains(add) &&
          !(port->hasRetryPackets()))
        return port;
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
  if (PEPort *pe_port = dynamic_cast<PEPort *>(port))
    return pe_port->streamValid(len, is_read);
  else
    panic("currently no support for validation check other than PE port\n");

  return false;
}

void WindowManager::printPortRanges() {
  for (auto port : localPorts) {
    AddrRangeList adl = port->getAddrRanges();
    for (auto address : adl) {
      if (debug())
        DPRINTF(WindowManager, "localPorts: %s, Range: %lx-%lx\n", port->name(),
                address.start(), address.end());
    }
  }
  for (auto port : globalPorts) {
    AddrRangeList adl = port->getAddrRanges();
    for (auto address : adl) {
      if (debug())
        DPRINTF(WindowManager, "globalPorts: %s, Range: %lx-%lx\n",
                port->name(), address.start(), address.end());
    }
  }
  for (auto port : spmPorts) {
    AddrRangeList adl = port->getAddrRanges();
    for (auto address : adl) {
      if (debug())
        DPRINTF(WindowManager, "spmPorts: %s, Range: %lx-%lx\n", port->name(),
                address.start(), address.end());
    }
  }
  for (auto port : requestStreamPorts) {
    AddrRangeList adl = port->getAddrRanges();
    for (auto address : adl) {
      if (debug())
        DPRINTF(WindowManager, "requestStreamPorts: %s, Range: %lx-%lx\n",
                port->name(), address.start(), address.end());
    }
  }
  for (auto port : responseStreamPorts) {
    AddrRangeList adl = port->getAddrRanges();
    for (auto address : adl) {
      if (debug())
        DPRINTF(WindowManager, "responseStreamPorts: %s, Range: %lx-%lx\n",
                port->name(), address.start(), address.end());
    }
  }
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
  } else if (if_name == "req_stream_ports") {
    if (idx >= requestStreamPorts.size())
      requestStreamPorts.resize((idx + 1));
    if (requestStreamPorts[idx] == nullptr) {
      const std::string portName =
          name() + csprintf(".req_stream_ports[%d]", idx);
      requestStreamPorts[idx] = new PEPort(portName, this, idx);
    }
    return *requestStreamPorts[idx];
  } else if (if_name == "resp_stream_ports") {
    if (idx >= responseStreamPorts.size())
      responseStreamPorts.resize((idx + 1));
    if (responseStreamPorts[idx] == nullptr) {
      const std::string portName =
          name() + csprintf(".resp_stream_ports[%d]", idx);
      responseStreamPorts[idx] = new PEPort(portName, this, idx);
    }
    return *responseStreamPorts[idx];
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
