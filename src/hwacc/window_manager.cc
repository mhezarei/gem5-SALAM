#include "hwacc/window_manager.hh"
#include "base/trace.hh"
#include "debug/Window.hh"
#include "debug/WindowManager.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "sim/system.hh"

#include <string>

WindowManager::WindowManager(const WindowManagerParams &p)
    : BasicPioDevice(p, p.pio_size), tickEvent(this), deviceName(p.devicename),
      debugEnabled(p.debug_enabled), clockPeriod(p.clock_period),
      masterId(p.system->getRequestorId(this, name())), io_size(p.pio_size),
      io_addr(p.pio_addr), endian(p.system->getGuestByteOrder()) {
  processingDelay = 1000 * clockPeriod;

  peRequestLength = 8;
  dataSize = 4;
  tsDataSize = 8;
  endToken = 0xFFFFFFFFFFFFFFFF;

  spmSize = 2048;
  currentFreeSPMAddress = 0x10021080;
  coresStartAddr = 0x80C00000;

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
      if (*((uint64_t *)read_req->getBuffer()) == endToken) {
        finishedPEs.push_back(true);
        removeMemRequest(read_req, activeReadRequests);
        scheduleEvent();
        delete pkt;
        return;
      }

      // handlePESignalRequest(read_req, pe_port);
      handlePETimeseriesRequest(read_req, pe_port);
    } else if (GlobalPort *global_port =
                   dynamic_cast<GlobalPort *>(carrier_port)) {
      // handleSignalWindowMemoryResponse(pkt, read_req);
      handleTSWindowMemoryResponse(pkt, read_req);
    }

    removeMemRequest(read_req, activeReadRequests);
    delete read_req;
  } else if (pkt->isWrite()) {
    MemoryRequest *write_req = findMemRequest(pkt, activeWriteRequests);
    RequestPort *carrier_port = write_req->getCarrierPort();

    if (SPMPort *port = dynamic_cast<SPMPort *>(carrier_port)) {
      removeCorrespondingSignalWindowRequest(write_req);
      if (debug())
        DPRINTF(WindowManager, "Done with a write. addr: 0x%x, size: %d\n",
                pkt->req->getPaddr(), pkt->getSize());
    } else if (PEPort *port = dynamic_cast<PEPort *>(carrier_port)) {
      if (debug())
        DPRINTF(WindowManager, "Done with a write. addr: 0x%x, size: %d\n",
                pkt->req->getPaddr(), pkt->getSize());
    } else if (GlobalPort *port = dynamic_cast<GlobalPort *>(carrier_port)) {
      if (debug())
        DPRINTF(WindowManager, "Done with a write. addr: 0x%x, size: %d\n",
                pkt->req->getPaddr(), pkt->getSize());
      handleTSWindowMemoryResponse(pkt, write_req);
    } else {
      panic("WE ARE DROWNING!\n");
    }

    removeMemRequest(write_req, activeWriteRequests);
    delete write_req;
  } else {
    panic("Received packet that is neither read nor write\n");
  }

  scheduleEvent();
  delete pkt;
}

void WindowManager::handlePETimeseriesRequest(MemoryRequest *read_req,
                                              PEPort *pe_port) {
  for (auto &pair : activeTSWindowRequests) {
    if (std::find(pair.second.begin(), pair.second.end(), read_req) !=
        pair.second.end()) { // this was a result request from WM to PE
      handleTSWindowMemoryResponse(read_req->getPacket(), read_req);
      return;
    }
  }

  uint64_t value = extractPERequestValue(read_req);

  for (auto &window : ongoingTSWindows) {
    if (window->samePEPort(pe_port)) {
      window->setPERequest(value);
      return;
    }
  }

  // no previous windows matching this port was found => new port request
  TimeseriesWindow *w = new TimeseriesWindow(this, pe_port);
  w->setPERequest(value);
  ongoingTSWindows.push_back(w);
}

WindowManager::TimeseriesWindow::TimeseriesWindow(WindowManager *owner,
                                                  PEPort *pe_port)
    : windowName("TimeseriesWindow"), owner(owner),
      correspondingPEPort(pe_port) {
  peRequest = (PETimeseriesRequest){UINT64_MAX, UINT64_MAX};
  state = none;
  currentCoreAddr = 0;
  batchSize = 8;
  numChildren = 4;
  numReceivedChildren = 0;

  leavesStartAddr = 0x80c00118;
  leavesEndAddr = 0x80c00460;
  numCheckedNodes = 0;
}

bool WindowManager::TimeseriesWindow::isLeafNode(Addr node_addr) {
  // TODO: better leaf check logic
  return leavesStartAddr <= node_addr && node_addr <= leavesEndAddr;
}

void WindowManager::TimeseriesWindow::firstScanTick() {
  if (state == requestCoreStart) {
    if (debug())
      DPRINTF(Window, "State: requestCoreStart\n");

    if (coreQueue.empty()) {
      state = firstScanDone;
      return;
    }

    currentCoreAddr = coreQueue.front();
    // TODO: parameterize core element offsets
    Addr coreRangeStart = currentCoreAddr + 0;

    if (debug())
      DPRINTF(Window, "Core Start Addr: 0x%016x\n", currentCoreAddr);

    // request range start
    if (GlobalPort *memory_port = dynamic_cast<GlobalPort *>(
            owner->getValidGlobalPort(coreRangeStart, true))) {
      MemoryRequest *read_req =
          owner->readFromPort(memory_port, coreRangeStart, owner->tsDataSize);
      owner->activeTSWindowRequests[this].push_back(read_req);
      state = waitingCoreRangeStart;
    } else {
      if (debug())
        DPRINTF(Window,
                "Did not find a global port to read %d bytes from 0x%016x\n",
                owner->tsDataSize, coreRangeStart);
      currentCoreAddr = 0;
    }
  } else if (state == requestCoreEnd) {
    if (debug())
      DPRINTF(Window, "State: requestCoreEnd\n");

    Addr coreRangeEnd = currentCoreAddr + owner->tsDataSize;

    if (debug())
      DPRINTF(Window, "Core End Addr: 0x%016x\n", coreRangeEnd);

    // request range end
    if (GlobalPort *memory_port = dynamic_cast<GlobalPort *>(
            owner->getValidGlobalPort(coreRangeEnd, true))) {
      MemoryRequest *read_req =
          owner->readFromPort(memory_port, coreRangeEnd, owner->tsDataSize);
      owner->activeTSWindowRequests[this].push_back(read_req);
      state = waitingCoreRangeEnd;
    } else {
      if (debug())
        DPRINTF(Window,
                "Did not find a global port to read %d bytes from 0x%016x\n",
                owner->tsDataSize, coreRangeEnd);
    }
  } else if (state == checkRange) {
    if (debug())
      DPRINTF(Window, "State: checkRange\n");

    uint64_t inp_start = peRequest.startTimestamp;
    uint64_t inp_end = peRequest.endTimestamp;
    double node_start, node_end;
    memcpy(&node_start, &currentCoreStart, sizeof(node_start));
    memcpy(&node_end, &currentCoreEnd, sizeof(node_end));

    assert(node_start < node_end);

    if (debug())
      DPRINTF(Window,
              "Checking range inp_start:%d inp_end:%d against node_start:%f "
              "node_end:%f\n",
              inp_start, inp_end, node_start, node_end);

    bool is_leaf = isLeafNode(currentCoreAddr);
    bool case_skip = (node_start > inp_end || node_end < inp_start);
    bool case_finished = (node_start == inp_start && node_end == inp_end);
    bool case_add_to_list = (node_start >= inp_start && node_end <= inp_end);

    if (case_skip) {
      state = requestCoreStart;
    } else if (case_finished) {
      computationCores.push(currentCoreAddr);
      state = done;
    } else if (case_add_to_list) {
      computationCores.push(currentCoreAddr);
      state = requestCoreStart;
    } else { // this means either leaf/partial or non-leaf/children
      if (!is_leaf) {
        state = requestChildAddress;
      } else {
        // TODO: partial leaf scan please
        // state =
      }
    }

    if (!coreQueue.empty())
      coreQueue.pop();
  } else if (state == requestChildAddress) {
    if (debug())
      DPRINTF(Window, "State: requestChildAddress\n");

    assert(numReceivedChildren >= 0 && numReceivedChildren < numChildren);

    Addr childAddress =
        currentCoreAddr + (numReceivedChildren + 3) * owner->tsDataSize;

    if (debug())
      DPRINTF(Window,
              "Requesting address for Child number %d which is inside address "
              "0x%016x\n",
              numReceivedChildren, childAddress);

    // request child address
    if (GlobalPort *memory_port = dynamic_cast<GlobalPort *>(
            owner->getValidGlobalPort(childAddress, true))) {
      MemoryRequest *read_req =
          owner->readFromPort(memory_port, childAddress, owner->tsDataSize);
      owner->activeTSWindowRequests[this].push_back(read_req);
      state = waitingChildAddress;
    } else {
      if (debug())
        DPRINTF(Window,
                "Did not find a global port to read %d bytes from 0x%016x\n",
                owner->tsDataSize, childAddress);
    }
  } else if (state == firstScanDone) {
    state = initTraverse;
  }
}

void WindowManager::TimeseriesWindow::traverseTick() {
  if (state == initTraverse) {
    if (debug())
      DPRINTF(Window, "State: initTraverse\n");

    if (computationCores.empty()) {
      if (debug())
        DPRINTF(Window, "We are done DONE!\n");

      state = done;
      return;
    }

    DPRINTF(Window, "Going over this computation core: 0x%016x\n",
            computationCores.front());
    traverseStack.push(std::make_pair(computationCores.front(), false));
    computationCores.pop();
    state = requestStat;
  } else if (state == requestStat) {
    if (debug())
      DPRINTF(Window, "State: requestStat\n");

    if (traverseStack.top()
            .second) { // visited nodes do not need initial stat check
      if (debug())
        DPRINTF(Window, "no need for stat check\n");

      state = startTraverse;
      return;
    }

    Addr statAddr = traverseStack.top().first + 2 * owner->tsDataSize;

    if (debug())
      DPRINTF(Window, "I want stat addr: 0x%016x\n", statAddr);

    if (GlobalPort *memory_port = dynamic_cast<GlobalPort *>(
            owner->getValidGlobalPort(statAddr, true))) {
      MemoryRequest *read_req =
          owner->readFromPort(memory_port, statAddr, owner->tsDataSize);
      owner->activeTSWindowRequests[this].push_back(read_req);
      state = waitingStat;
      numCheckedNodes++;
    } else {
      if (debug())
        DPRINTF(Window,
                "Did not find a global port to read %d bytes from 0x%016x\n",
                owner->tsDataSize, statAddr);
    }
  } else if (state == startTraverse) {
    if (debug())
      DPRINTF(Window, "State: startTraverse\n");

    traverseStackHead = traverseStack.top();

    if (debug())
      DPRINTF(Window, "Head addr and visited: 0x%016x, %d\n",
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
      DPRINTF(Window, "State: requestTraverseChildren\n");

    assert(numReceivedChildren >= 0 && numReceivedChildren < numChildren);

    // adding to stack from end to beginning
    Addr childAddress =
        traverseStackHead.first +
        ((numChildren - numReceivedChildren - 1) + 3) * owner->tsDataSize;

    if (debug())
      DPRINTF(Window,
              "Requesting address for Child number %d which is inside address "
              "0x%016x\n",
              numReceivedChildren, childAddress);

    if (GlobalPort *memory_port = dynamic_cast<GlobalPort *>(
            owner->getValidGlobalPort(childAddress, true))) {
      MemoryRequest *read_req =
          owner->readFromPort(memory_port, childAddress, owner->tsDataSize);
      owner->activeTSWindowRequests[this].push_back(read_req);
      state = waitingTraverseChildren;
    } else {
      if (debug())
        DPRINTF(Window,
                "Did not find a global port to read %d bytes from 0x%016x\n",
                owner->tsDataSize, childAddress);
    }
  } else if (state == computeStat) {
    if (debug())
      DPRINTF(Window, "State: computeStat\n");

    openPEPort = findPEPortForComputeState();
    Addr port_addr = openPEPort->getStartAddress();

    if (isLeafNode(traverseStackHead.first)) { // leaf node
      MemoryRequest *write_req =
          owner->writeToPort(openPEPort, (uint64_t)1, port_addr);
      owner->activeTSWindowRequests[this].push_back(write_req);

      write_req = owner->writeToPort(
          openPEPort, (uint64_t)(batchSize * numChildren), port_addr);
      owner->activeTSWindowRequests[this].push_back(write_req);

      state = computeStat_RequestLeafChildrenStartAddress;
    } else { // non-leaf node
      MemoryRequest *write_req =
          owner->writeToPort(openPEPort, (uint64_t)2, port_addr);
      owner->activeTSWindowRequests[this].push_back(write_req);

      write_req =
          owner->writeToPort(openPEPort, (uint64_t)numChildren, port_addr);
      owner->activeTSWindowRequests[this].push_back(write_req);

      state = computeStat_RequestCoreChildren;
    }
  } else if (state == computeStat_RequestLeafChildrenStartAddress) {
    if (debug())
      DPRINTF(Window, "State: computeStat_RequestLeafChildrenStartAddress\n");

    // TODO: leaf's children might not be sequential
    // TODO: fix reversed order
    Addr childStartAddress = traverseStackHead.first + 3 * owner->tsDataSize;

    if (GlobalPort *memory_port = dynamic_cast<GlobalPort *>(
            owner->getValidGlobalPort(childStartAddress, true))) {
      MemoryRequest *read_req = owner->readFromPort(
          memory_port, childStartAddress, owner->tsDataSize);
      owner->activeTSWindowRequests[this].push_back(read_req);
      state = computeStat_WaitingLeafChildStartAddress;
    } else {
      if (debug())
        DPRINTF(Window,
                "Did not find a global port to read %d bytes from 0x%016x\n",
                owner->tsDataSize, childStartAddress);
    }
  } else if (state == computeStat_RequestCoreChildren) {
    if (debug())
      DPRINTF(Window, "State: computeStat_RequestCoreChildren\n");

    Addr childStartAddress =
        traverseStackHead.first + (numReceivedChildren + 3) * owner->tsDataSize;

    if (GlobalPort *memory_port = dynamic_cast<GlobalPort *>(
            owner->getValidGlobalPort(childStartAddress, true))) {
      MemoryRequest *read_req = owner->readFromPort(
          memory_port, childStartAddress, owner->tsDataSize);
      owner->activeTSWindowRequests[this].push_back(read_req);

      state = computeStat_WaitingCoreChildren;
    } else {
      if (debug())
        DPRINTF(Window,
                "Did not find a global port to read %d bytes from 0x%016x\n",
                owner->tsDataSize, childStartAddress);
    }
  } else if (state == computeStat_RequestCoreChildStat) {
    if (debug())
      DPRINTF(Window, "State: computeStat_RequestCoreChildStat\n");

    Addr childStatAddress = currentCoreChildAddress + 2 * owner->tsDataSize;

    if (GlobalPort *memory_port = dynamic_cast<GlobalPort *>(
            owner->getValidGlobalPort(childStatAddress, true))) {
      MemoryRequest *read_req =
          owner->readFromPort(memory_port, childStatAddress, owner->tsDataSize);
      owner->activeTSWindowRequests[this].push_back(read_req);
      state = computeStat_WaitingCoreChildStat;
    } else {
      if (debug())
        DPRINTF(Window,
                "Did not find a global port to read %d bytes from 0x%016x\n",
                owner->tsDataSize, childStatAddress);
    }
  } else if (state == computeStat_RequestResult) {
    if (debug())
      DPRINTF(Window, "State: computeStat_RequestResult\n");

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
          pe_req_port, pe_req_port->getStartAddress(), owner->peRequestLength);
      owner->activeTSWindowRequests[this].push_back(read_req);
      state = computeStat_WaitingResult;
    }
  } else if (state == computeStat_SaveResult) {
    if (debug())
      DPRINTF(Window, "State: computeStat_SaveResult\n");

    Addr currentCoreStatAddr = traverseStackHead.first + 2 * owner->tsDataSize;

    if (GlobalPort *memory_port = dynamic_cast<GlobalPort *>(
            owner->getValidGlobalPort(currentCoreStatAddr, true))) {
      MemoryRequest *write_req =
          owner->writeToPort(memory_port, computedResult, currentCoreStatAddr);
      owner->activeTSWindowRequests[this].push_back(write_req);
      state = computeStat_WaitingSaveResult;
    } else {
      if (debug())
        DPRINTF(Window,
                "Did not find a global port to read %d bytes from 0x%016x\n",
                owner->tsDataSize, currentCoreStatAddr);
    }
  } else if (state == done) {
    if (debug())
      DPRINTF(Window, "State: done with %d results\n", endResults.size());

    uint64_t result = 0;
    for (auto val : endResults)
      result += val;
    result /= endResults.size();

    DPRINTF(Window, "END RESULT IS 0x%016x\n", result);
  }
}

void WindowManager::TimeseriesWindow::handleResponseData(uint64_t data) {
  if (state == waitingCoreRangeStart) {
    if (debug())
      DPRINTF(Window, "State: waitingCoreRangeStart; the start is 0x%016x\n",
              data);

    currentCoreStart = data;
    state = requestCoreEnd;
  } else if (state == waitingCoreRangeEnd) {
    if (debug())
      DPRINTF(Window, "State: waitingCoreRangeEnd; the end is 0x%016x\n", data);

    currentCoreEnd = data;
    state = checkRange;
  } else if (state == waitingChildAddress) {
    if (debug())
      DPRINTF(Window,
              "State: waitingChildAddress; the child address is 0x%016x\n",
              data);

    coreQueue.push(data);
    numReceivedChildren++;
    if (numReceivedChildren == numChildren) {
      state = requestCoreStart;
      numReceivedChildren = 0;
    } else {
      state = requestChildAddress;
    }
  } else if (state == waitingStat) {
    if (debug())
      DPRINTF(Window, "State: waitingStat; the stat is 0x%016x\n", data);

    if (data == 0) {
      state = startTraverse;
    } else { // stat is present.
      traverseStack.pop();

      // root has stat
      if (numCheckedNodes == 1) {
        numCheckedNodes = 0;
        state = initTraverse;
      }
      // non-root has stat
      if (numCheckedNodes > 1) {
        state = requestStat;
      }
    }
  } else if (state == waitingTraverseChildren) {
    if (debug())
      DPRINTF(Window,
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
  } else if (state == computeStat_WaitingLeafChildStartAddress) {
    if (debug())
      DPRINTF(Window,
              "State: computeStat_WaitingLeafChildStartAddress; the child "
              "address is 0x%016x\n",
              data);

    assert(openPEPort != nullptr);
    MemoryRequest *write_req =
        owner->writeToPort(openPEPort, data, openPEPort->getStartAddress());
    owner->activeTSWindowRequests[this].push_back(write_req);
    state = computeStat_RequestResult;
  } else if (state == computeStat_WaitingCoreChildren) {
    if (debug())
      DPRINTF(Window,
              "State: computeStat_WaitingCoreChildren; the child address is "
              "0x%016x\n",
              data);

    currentCoreChildAddress = data;
    numReceivedChildren++;
    state = computeStat_RequestCoreChildStat;
  } else if (state == computeStat_WaitingCoreChildStat) {
    if (debug())
      DPRINTF(Window,
              "State: computeStat_WaitingCoreChildStat; the child stat is "
              "0x%016x\n",
              data);

    assert(openPEPort != nullptr);
    MemoryRequest *write_req =
        owner->writeToPort(openPEPort, data, openPEPort->getStartAddress());
    owner->activeTSWindowRequests[this].push_back(write_req);

    if (numReceivedChildren == numChildren) {
      numReceivedChildren = 0;
      state = computeStat_RequestResult;
    } else {
      state = computeStat_RequestCoreChildren;
    }
  } else if (state == computeStat_WaitingResult) {
    if (debug())
      DPRINTF(Window,
              "State: computeStat_WaitingResult; the result from PE is "
              "0x%016x\n",
              data);

    computedResult = data;
    state = computeStat_SaveResult;
  } else if (state == computeStat_WaitingSaveResult) {
    if (debug())
      DPRINTF(Window, "State: computeStat_WaitingSaveResult;\n");

    traverseStack.pop();
    if (traverseStack.empty()) { // done for this computation core
      numCheckedNodes = 0;
      state = initTraverse;
      endResults.push_back(computedResult);
    } else {
      computedResult = 0;
      state = requestStat;
    }
  }
}

void WindowManager::tsTick() {
  // turn off
  // if (ongoingTSWindows.empty()) {
  //   *mmr &= 0xfc;
  //   *mmr |= 0x04;
  //   return;
  // }

  // read from PEs (start, end)
  for (PEPort *port : peRequestStreamPorts) {
    if (port != peRequestStreamPorts[0] &&
        checkPort(port, peRequestLength, true)) {
      readFromPort(port, port->getStartAddress(), peRequestLength);
    }
  }

  // clear out done windows
  // for (auto it = ongoingTSWindows.begin(); it != ongoingTSWindows.end();
  // ++it) {
  //   if ((*it)->isDone()) {
  //     ongoingTSWindows.erase(it);
  //   }
  // }

  // advance timeseries windows by one
  for (auto &w : ongoingTSWindows) {
    w->firstScanTick();
    w->traverseTick();
  }
}

void WindowManager::tick() {
  if (debug())
    DPRINTF(WindowManager, "Tick!\n");

  // for (auto &p : peResponseStreamPorts)
  //   DPRINTF(WindowManager, "THROBBING RESP 0x%016x\n", p->getStartAddress());
  // for (auto &p : peRequestStreamPorts)
  //   DPRINTF(WindowManager, "THROBBING REQ 0x%016x\n", p->getStartAddress());

  // signalTick();
  tsTick();

  // Schedule the next tick if needed
  scheduleEvent();
}

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
      readFromPort(port, port->getStartAddress(), peRequestLength);
    }
  }

  // remove windows without any requests
  for (auto it = activeSignalWindowRequests.begin();
       it != activeSignalWindowRequests.end(); it++)
    if (it->second.empty()) {
      // respond to the corresponding PE
      Window *window = it->first;
      sendPEResponse(window->getCorrespondingPEPort(),
                     window->getSPMBaseAddr());

      activeSignalWindowRequests.erase(it);
    }

  // check the single ongoing Window (head of the queue)
  if (ongoingWindows.size()) {
    if (debug())
      DPRINTF(WindowManager, "Going over the head window...\n");
    if (!ongoingWindows.front()->sendMemoryRequest()) {
      ongoingWindows.pop();
    }
  }

  // check spmRetryPackets and send them to the SPM
  while (SPMPort *spmPort = findAvailableSPMPort()) {
    if (!spmRetryPackets.size())
      break;

    Window *window = spmRetryPackets.front().first;
    uint64_t data = spmRetryPackets.front().second;
    window->sendSPMRequest(data);
    spmRetryPackets.pop();
  }
}

/* Utility functions start */
WindowManager::PEPort *
WindowManager::TimeseriesWindow::findPEPortForComputeState() {
  // TODO: write complete PE response port finding and sending
  return owner->peResponseStreamPorts[0];
}

void WindowManager::handleTSWindowMemoryResponse(PacketPtr pkt,
                                                 MemoryRequest *req) {
  TimeseriesWindow *window = findCorrespondingTSWindow(pkt);
  uint64_t data = *((uint64_t *)req->getBuffer());
  window->handleResponseData(data);
  removeCorrespondingTSWindowRequest(req);
}

void WindowManager::TimeseriesWindow::setPERequest(uint64_t value) {
  if (peRequest.startTimestamp == UINT64_MAX)
    peRequest.startTimestamp = value;
  else if (peRequest.endTimestamp == UINT64_MAX) {
    peRequest.endTimestamp = value;
    coreQueue.push(owner->coresStartAddr);
    state = requestCoreStart;
  } else
    panic("Calling setPERequest when you should not!\n");
}

void WindowManager::handlePESignalRequest(MemoryRequest *read_req,
                                          PEPort *pe_port) {
  // Necessary steps for window creation
  PESignalRequest pe_req = constructPERequestFromReadRequest(read_req);
  Addr base_addr = pe_req.sourceAddr;
  std::vector<Offset> offsets;
  for (uint16_t idx = pe_req.startElement;
       idx < pe_req.startElement + pe_req.length; idx++)
    offsets.push_back(idx * dataSize);

  Window *window =
      new Window(this, base_addr, offsets, currentFreeSPMAddress, pe_port);
  ongoingWindows.push(window);

  // Move the SPM poninter based on PE request
  currentFreeSPMAddress += pe_req.length * dataSize;
}

void WindowManager::handleSignalWindowMemoryResponse(PacketPtr pkt,
                                                     MemoryRequest *read_req) {
  Window *window = findCorrespondingSignalWindow(pkt);
  uint64_t data = *((uint64_t *)read_req->getBuffer());

  if (!window->sendSPMRequest(data)) {
    spmRetryPackets.push(std::make_pair(window, data));
  }

  removeCorrespondingSignalWindowRequest(read_req);
}

MemoryRequest *WindowManager::readFromPort(RequestPort *port, Addr addr,
                                           size_t len) {
  MemoryRequest *read_req = new MemoryRequest(addr, len);
  Request::Flags flags;
  RequestPtr req = std::make_shared<Request>(addr, len, flags, masterId);
  PacketPtr pkt = new Packet(req, MemCmd::ReadReq);

  read_req->setCarrierPort(port);
  read_req->setPacket(pkt);
  pkt->allocate();

  if (PEPort *pe_port = dynamic_cast<PEPort *>(port)) {
    pe_port->sendPacket(pkt);
    activeReadRequests.push_back(read_req);
  } else if (GlobalPort *memory_port = dynamic_cast<GlobalPort *>(port)) {
    memory_port->sendPacket(pkt);
    activeReadRequests.push_back(read_req);
  } else {
    panic("sending port is not suitable for reading");
  }

  if (debug())
    DPRINTF(WindowManager,
            "Trying to read addr: 0x%016x, %d bytes through port: %s\n", addr,
            len, port->name());

  scheduleEvent();
  return read_req;
}

MemoryRequest *WindowManager::writeToPort(RequestPort *req_port, uint64_t input,
                                          Addr addr) {
  size_t data_size =
      tsDataSize; // TODO: fix variable data size for TS and signal

  MemoryRequest *write_req =
      new MemoryRequest(addr, (uint8_t *)&input, data_size);
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

void WindowManager::sendPEResponse(PEPort *req_pe_port, Addr spm_addr) {
  int port_idx =
      std::distance(peRequestStreamPorts.begin(),
                    std::find(peRequestStreamPorts.begin(),
                              peRequestStreamPorts.end(), req_pe_port));
  assert(port_idx < peResponseStreamPorts.size());

  PEPort *pe_resp_port = peResponseStreamPorts[port_idx];
  Addr destination_addr = pe_resp_port->getStartAddress();

  writeToPort(pe_resp_port, spm_addr, destination_addr);
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

WindowManager::SPMPort *WindowManager::findAvailableSPMPort() {
  for (auto port : spmPorts)
    if (!port->isActive())
      return port;
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

WindowManager::PESignalRequest
WindowManager::constructPERequestFromReadRequest(MemoryRequest *read_req) {
  uint64_t result = extractPERequestValue(read_req);

  uint32_t source_addr = result >> 32;
  uint16_t start_element = (result >> 16) & 0x000000000000FFFF;
  uint16_t length = result & 0x000000000000FFFF;

  return (PESignalRequest){source_addr, start_element, length};
}

void WindowManager::scheduleEvent(Tick when) {
  Tick actual_when = (when == 0) ? curTick() + processingDelay : when;
  if (!tickEvent.scheduled())
    schedule(tickEvent, actual_when);
}
/* Utility functions end */

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

    // TODO: This should just signal the engine that the packet completed
    // engine should schedule tick as necessary. Need a test case
    if (!owner->tickEvent.scheduled()) {
      owner->schedule(owner->tickEvent, curTick() + owner->processingDelay);
    }
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

    // TODO: This should just signal the engine that the packet completed
    // engine should schedule tick as necessary. Need a test case
    if (!owner->tickEvent.scheduled()) {
      owner->schedule(owner->tickEvent, curTick() + owner->processingDelay);
    }
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

void WindowManager::removeMemRequest(MemoryRequest *mem_req,
                                     std::vector<MemoryRequest *> &target_vec) {
  if (debug())
    DPRINTF(WindowManager, "Clearing Request with addr 0x%016x\n",
            mem_req->getAddress());

  for (auto it = target_vec.begin(); it != target_vec.end(); ++it) {
    if ((*it) == mem_req) {
      it = target_vec.erase(it);
      break;
    }
  }

  // TODO This is really wired. The code below should work but it doesn't.
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

WindowManager::Window *
WindowManager::findCorrespondingSignalWindow(PacketPtr pkt) {
  for (const auto &elem : activeSignalWindowRequests)
    for (const auto &req : elem.second)
      if (req->getPacket() == pkt)
        return elem.first;

  panic("Could not find memory request in active window requests map\n");
  return nullptr;
}

void WindowManager::removeCorrespondingSignalWindowRequest(
    MemoryRequest *mem_req) {
  if (debug())
    DPRINTF(WindowManager, "Clearing Request with addr 0x%016x\n",
            mem_req->getAddress());

  for (const auto &elem : activeSignalWindowRequests) {
    for (auto it = elem.second.begin(); it != elem.second.end(); ++it) {
      if (*it == mem_req) {
        activeSignalWindowRequests[elem.first].erase(it);
        break;
      }
    }
  }
}

WindowManager::TimeseriesWindow *
WindowManager::findCorrespondingTSWindow(PacketPtr pkt) {
  for (const auto &elem : activeTSWindowRequests)
    for (const auto &req : elem.second)
      if (req->getPacket() == pkt)
        return elem.first;

  panic("Could not find memory request in active window requests map\n");
  return nullptr;
}

void WindowManager::removeCorrespondingTSWindowRequest(MemoryRequest *mem_req) {
  if (debug())
    DPRINTF(WindowManager, "Clearing Request with addr 0x%016x\n",
            mem_req->getAddress());

  for (const auto &elem : activeTSWindowRequests) {
    for (auto it = elem.second.begin(); it != elem.second.end(); ++it) {
      if (*it == mem_req) {
        activeTSWindowRequests[elem.first].erase(it);
        break;
      }
    }
  }
}
/* Helper functions end */

/* Window functions start */
WindowManager::Window::Window(WindowManager *owner, Addr base_memory_addr,
                              const std::vector<Offset> &offsets,
                              Addr base_spm_addr, PEPort *pe_port)
    : windowName("Window"), owner(owner), baseMemoryAddress(base_memory_addr),
      spmBaseAddr(base_spm_addr), currentSPMOffset(0),
      correspondingPEPort(pe_port) {
  for (Offset o : offsets) {
    Addr req_addr = baseMemoryAddress + o;

    MemoryRequest *read_req = new MemoryRequest(req_addr, owner->dataSize);
    Request::Flags flags;
    RequestPtr req = std::make_shared<Request>(req_addr, owner->dataSize, flags,
                                               owner->masterId);
    PacketPtr pkt = new Packet(req, MemCmd::ReadReq);

    read_req->setPacket(pkt);
    pkt->allocate();

    memoryRequests.push(read_req);
  }
}

bool WindowManager::Window::sendMemoryRequest() {
  if (memoryRequests.empty()) {
    if (debug())
      DPRINTF(Window, "No more memory read requests to send\n");
    return false;
  }

  MemoryRequest *read_req = memoryRequests.front();

  if (GlobalPort *memory_port = dynamic_cast<GlobalPort *>(
          owner->getValidGlobalPort(read_req->getAddress(), true))) {
    if (debug())
      DPRINTF(Window,
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
      DPRINTF(Window,
              "Did not find a global port to read %d bytes from 0x%016x\n",
              read_req->getLength(), read_req->getAddress());
  }

  return true;
}

bool WindowManager::Window::sendSPMRequest(uint64_t data) {
  // create the SPM write packet and send it!
  Addr addr = spmBaseAddr + currentSPMOffset;
  SPMPort *spm_port = owner->findAvailableSPMPort();
  if (!spm_port)
    return false;

  MemoryRequest *write_req = owner->writeToPort(spm_port, data, addr);
  owner->activeSignalWindowRequests[this].push_back(write_req);

  currentSPMOffset += owner->dataSize;
  return true;
}
/* Window functions end */
