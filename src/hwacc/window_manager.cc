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
  requestLength = 4;
  sourceIDToAddr[1] = (Addr)(0x80c00000);
  sourceIDToAddr[2] = (Addr)(0x80c00190);

  spmSize = 512;
  currentFreeSPMAddress = 0x10021080;

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
      handlePERequest(read_req);
    } else if (GlobalPort *global_port =
                   dynamic_cast<GlobalPort *>(carrier_port)) {
      handleWindowMemoryResponse(pkt, read_req);
    }

    removeMemRequest(read_req, activeReadRequests);
  } else if (pkt->isWrite()) {
    MemoryRequest *write_req = findMemRequest(pkt, activeWriteRequests);
    if (SPMPort *port = dynamic_cast<SPMPort *>(write_req->getCarrierPort())) {
      if (debug())
        DPRINTF(WindowManager, "Done with a write. addr: 0x%x, size: %d\n",
                pkt->req->getPaddr(), pkt->getSize());
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

void WindowManager::tick() {
  if (debug())
    DPRINTF(WindowManager, "Tick!\n");

  // Check all the pe ports for incoming requests
  for (PEPort *port : peStreamPorts) {
    if (checkPort(port, requestLength, true)) {
      readFromPort(port, port->getStartAddress(), requestLength);
    }
  }

  // check the single ongoing Window (head of the queue)
  if (ongoingWindows.size()) {
    if (debug())
      DPRINTF(WindowManager, "Going over the head window...\n");
    Window *window = ongoingWindows.front();
    if (!window->sendMemoryRequest()) {
      ongoingWindows.pop();
    }
  }

  // check spmRetryPackets and send them to the SPM
  while (SPMPort *spmPort = findAvailableSPMPort()) {
    if (!spmRetryPackets.size())
      break;

    Window *window = spmRetryPackets.front().first;
    uint64_t data = spmRetryPackets.front().second;
    window->sendSPMRequest(spmPort, data);
    spmRetryPackets.pop();
  }

  // Schedule the next tick if needed
  scheduleEvent();
}

/* Utility functions start */
void WindowManager::handlePERequest(MemoryRequest *read_req) {
  // Necessary steps for window creation
  PERequest pe_req = constructPERequestFromReadRequest(read_req);
  Addr base_addr = sourceIDToAddr[pe_req.sourceID];
  std::vector<Offset> offsets;
  for (uint16_t idx = pe_req.startElement;
       idx < pe_req.startElement + pe_req.length; idx++)
    offsets.push_back(idx * requestLength);

  Window *window = new Window(this, base_addr, offsets, currentFreeSPMAddress);
  ongoingWindows.push(window);

  // Move the SPM poninter based on PE request
  currentFreeSPMAddress += pe_req.length * requestLength;
}

void WindowManager::handleWindowMemoryResponse(PacketPtr pkt,
                                               MemoryRequest *read_req) {
  Window *window = findCorrespondingWindow(pkt);
  uint64_t data = *((uint64_t *)read_req->getBuffer());
  SPMPort *spm_port = findAvailableSPMPort();

  if (!spm_port) {
    spmRetryPackets.push(std::make_pair(window, data));
  } else {
    window->sendSPMRequest(spm_port, data);
  }

  removeActiveWindowRequest(read_req);
}

void WindowManager::readFromPort(RequestPort *port, Addr addr, size_t len) {
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
  }

  if (debug())
    DPRINTF(WindowManager,
            "Trying to read addr: 0x%016x, %d bytes through port: %s\n", addr,
            len, port->name());

  scheduleEvent();
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
      if (address.contains(add))
        return port;
  }
  return nullptr;
}

WindowManager::SPMPort *WindowManager::findAvailableSPMPort() {
  for (auto port : spmPorts)
    if (!port->isActive())
      return port;
  return nullptr;
}

WindowManager::PERequest
WindowManager::constructPERequestFromReadRequest(MemoryRequest *read_req) {
  size_t read_len = read_req->getLength();
  if (read_len == 4) { // 4 bytes
    assert(read_req->getTotalLength() == (Tick)read_len);

    uint32_t result = 0;
    for (int i = read_req->getTotalLength() - 1; i >= 0; i--)
      result = (result << 8) + read_req->getBuffer()[i];

    return (PERequest){(uint8_t)(result >> 24), (uint16_t)((result << 8) >> 16),
                       (uint8_t)((result << 24) >> 24)};
  }

  panic("Can't handle read lengths other than 4 for now!\n");
  return (PERequest){0, 0, 0};
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

std::string WindowManager::PEPort::getPENameFromPeerPort() {
  // TODO: find a way to get the PE name from the port
  if (!isConnected())
    panic("Port not connected");

  return std::to_string(getId());
}

/* Helper functions start */
bool WindowManager::checkPort(RequestPort *port, size_t len, bool isRead) {
  if (PEPort *pePort = dynamic_cast<PEPort *>(port))
    return pePort->streamValid(len, isRead);

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
  } else if (if_name == "pe_stream_ports") {
    if (idx >= peStreamPorts.size())
      peStreamPorts.resize((idx + 1));
    if (peStreamPorts[idx] == nullptr) {
      const std::string portName =
          name() + csprintf(".pe_stream_ports[%d]", idx);
      peStreamPorts[idx] = new PEPort(portName, this, idx);
    }
    return *peStreamPorts[idx];
  } else {
    return BasicPioDevice::getPort(if_name, idx);
  }
}

MemoryRequest *
WindowManager::findMemRequest(PacketPtr pkt,
                              const std::vector<MemoryRequest *> &targetVec) {
  auto it =
      find_if(begin(targetVec), end(targetVec),
              [&pkt](MemoryRequest *mr) { return mr->getPacket() == pkt; });

  if (it != end(targetVec))
    return *it;

  panic("Could not find memory request in request queues");
  return nullptr;
}

WindowManager::Window *WindowManager::findCorrespondingWindow(PacketPtr pkt) {
  auto it = find_if(begin(activeWindowMemoryRequests),
                    end(activeWindowMemoryRequests),
                    [&pkt](const std::pair<MemoryRequest *, Window *> &elem) {
                      return elem.first->getPacket() == pkt;
                    });

  if (it != end(activeWindowMemoryRequests))
    return it->second;

  panic("Could not find memory request in vector of <window, req> pairs\n");
  return nullptr;
}

void WindowManager::removeMemRequest(MemoryRequest *memReq,
                                     std::vector<MemoryRequest *> &targetVec) {
  if (debug())
    DPRINTF(WindowManager, "Clearing Request with addr 0x%016x\n",
            memReq->getAddress());

  for (auto it = targetVec.begin(); it != targetVec.end(); ++it) {
    if ((*it) == memReq) {
      it = targetVec.erase(it);
      break;
    }
  }

  // TODO This is really wired. The code below should work but it doesn't.
  // auto it =
  //     find_if(begin(targetVec), end(targetVec),
  //             [&memReq](const MemoryRequest * mr) { return mr == memReq; });
  // if (it != end(targetVec))
  //   readMemReqs.erase(it);
  // else
  //   DPRINTF(WindowManager,
  //           "Could not find memory request in request queues (for
  //           removal)\n");
}

void WindowManager::removeActiveWindowRequest(MemoryRequest *mem_req) {
  if (debug())
    DPRINTF(WindowManager, "Clearing Request with addr 0x%016x\n",
            mem_req->getAddress());

  for (auto it = activeWindowMemoryRequests.begin();
       it != activeWindowMemoryRequests.end(); ++it) {
    if (it->first == mem_req) {
      it = activeWindowMemoryRequests.erase(it);
      break;
    }
  }
}
/* Helper functions end */

/* Window functions start */
WindowManager::Window::Window(WindowManager *owner, Addr base_memory_addr,
                              const std::vector<Offset> &offsets,
                              Addr base_spm_addr)
    : windowName("Window"), owner(owner), baseMemoryAddress(base_memory_addr),
      spmBaseAddr(base_spm_addr), currentSPMOffset(0) {
  for (Offset o : offsets) {
    Addr req_addr = baseMemoryAddress + o;

    MemoryRequest *read_req = new MemoryRequest(req_addr, owner->requestLength);
    Request::Flags flags;
    RequestPtr req = std::make_shared<Request>(req_addr, owner->requestLength,
                                               flags, owner->masterId);
    PacketPtr pkt = new Packet(req, MemCmd::ReadReq);

    read_req->setCarrierPort(owner->getValidGlobalPort(req_addr, true));
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

  MemoryRequest *req = memoryRequests.front();
  sentMemoryRequests.push_back(req);
  owner->activeWindowMemoryRequests.push_back(std::make_pair(req, this));
  owner->activeReadRequests.push_back(req);

  if (GlobalPort *memory_port =
          dynamic_cast<GlobalPort *>(req->getCarrierPort())) {
    if (debug())
      DPRINTF(Window,
              "Trying to request addr: 0x%016x, %d bytes through port: %s\n",
              req->getAddress(), req->getLength(), memory_port->name());
    memory_port->sendPacket(req->getPacket());
  } else {
    panic("WE ARE DROWNED!\n");
  }

  memoryRequests.pop();
  return true;
}

void WindowManager::Window::sendSPMRequest(WindowManager::SPMPort *spm_port,
                                           uint64_t data) {
  // create the SPM write packet and send it!
  Addr addr = spmBaseAddr + currentSPMOffset;
  MemoryRequest *write_req = new MemoryRequest(addr, owner->requestLength);
  Request::Flags flags;
  RequestPtr req = std::make_shared<Request>(addr, owner->requestLength, flags,
                                             owner->masterId);
  PacketPtr pkt = new Packet(req, MemCmd::WriteReq);

  write_req->setCarrierPort(spm_port);
  req->setExtraData(data);
  write_req->setPacket(pkt);
  pkt->allocate();
  spm_port->sendPacket(pkt);
  owner->activeWriteRequests.push_back(write_req);

  currentSPMOffset += owner->requestLength;

  if (debug())
    DPRINTF(Window,
            "Trying to write to addr: 0x%016x, %d bytes, holding %lld value, "
            "through port: %s\n",
            addr, owner->requestLength, data, spm_port->name());
}
/* Window functions end */
