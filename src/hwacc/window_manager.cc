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
  endToken = 0xFFFFFFFFFFFFFFFF;

  spmSize = 2048;
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
      if (*((uint64_t *)read_req->getBuffer()) == endToken) {
        finishedPEs.push_back(true);
        removeMemRequest(read_req, activeReadRequests);
        scheduleEvent();
        delete pkt;
        return;
      }
      handlePERequest(read_req, pe_port);
    } else if (GlobalPort *global_port =
                   dynamic_cast<GlobalPort *>(carrier_port)) {
      handleWindowMemoryResponse(pkt, read_req);
    }

    removeMemRequest(read_req, activeReadRequests);
    delete read_req;
  } else if (pkt->isWrite()) {
    MemoryRequest *write_req = findMemRequest(pkt, activeWriteRequests);
    RequestPort *carrier_port = write_req->getCarrierPort();
    if (SPMPort *port = dynamic_cast<SPMPort *>(carrier_port)) {
      removeCorrespondingWindowRequest(write_req);
      if (debug())
        DPRINTF(WindowManager, "Done with a write. addr: 0x%x, size: %d\n",
                pkt->req->getPaddr(), pkt->getSize());
    } else if (PEPort *port = dynamic_cast<PEPort *>(carrier_port)) {
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

  // turning off condition
  if (activeWindowRequests.empty() &&
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
  for (auto it = activeWindowRequests.begin(); it != activeWindowRequests.end();
       it++)
    if (it->second.empty()) {
      // respond to the corresponding PE
      Window *window = it->first;
      sendPEResponse(window->getCorrespondingPEPort(),
                     window->getSPMBaseAddr());

      activeWindowRequests.erase(it);
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

  // Schedule the next tick if needed
  scheduleEvent();
}

/* Utility functions start */
void WindowManager::handlePERequest(MemoryRequest *read_req, PEPort *pe_port) {
  // Necessary steps for window creation
  PERequest pe_req = constructPERequestFromReadRequest(read_req);
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

void WindowManager::handleWindowMemoryResponse(PacketPtr pkt,
                                               MemoryRequest *read_req) {
  Window *window = findCorrespondingWindow(pkt);
  uint64_t data = *((uint64_t *)read_req->getBuffer());

  if (!window->sendSPMRequest(data)) {
    spmRetryPackets.push(std::make_pair(window, data));
  }

  removeCorrespondingWindowRequest(read_req);
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

MemoryRequest *WindowManager::writeToPort(RequestPort *req_port, uint64_t input,
                                          Addr addr) {
  MemoryRequest *write_req =
      new MemoryRequest(addr, (uint8_t *)&input, dataSize);
  Request::Flags flags;
  RequestPtr req = std::make_shared<Request>(addr, dataSize, flags, masterId);
  uint8_t *data = new uint8_t[dataSize];
  std::memcpy(data, write_req->getBuffer(), dataSize);
  req->setExtraData((uint64_t)data);
  write_req->setCarrierPort(req_port);

  if (debug())
    DPRINTF(
        WindowManager,
        "Trying to write to addr: 0x%016x, %d bytes, holding 0x%016x value, "
        "through port: %s\n",
        addr, dataSize, *((uint64_t *)write_req->getBuffer()),
        req_port->name());

  PacketPtr pkt = new Packet(req, MemCmd::WriteReq);
  uint8_t *pkt_data = (uint8_t *)req->getExtraData();
  pkt->dataDynamic(pkt_data);
  write_req->setPacket(pkt);

  if (PEPort *port = dynamic_cast<PEPort *>(req_port)) {
    port->sendPacket(pkt);
  } else if (SPMPort *port = dynamic_cast<SPMPort *>(req_port)) {
    port->sendPacket(pkt);
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

  MemoryRequest *write_req =
      writeToPort(pe_resp_port, spm_addr, destination_addr);

  activeWriteRequests.push_back(write_req);
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

WindowManager::PERequest
WindowManager::constructPERequestFromReadRequest(MemoryRequest *read_req) {
  size_t read_len = read_req->getLength();
  if (read_len == 8) { // 8 bytes
    assert(read_req->getTotalLength() == (Tick)read_len);

    uint64_t result = 0;
    for (int i = read_req->getTotalLength() - 1; i >= 0; i--)
      result = (result << 8) + read_req->getBuffer()[i];

    uint32_t source_addr = result >> 32;
    uint16_t start_element = (result >> 16) & 0x000000000000FFFF;
    uint16_t length = result & 0x000000000000FFFF;

    return (PERequest){source_addr, start_element, length};
  }

  panic("Can't handle read lengths other than 8 for now!\n");
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

WindowManager::Window *WindowManager::findCorrespondingWindow(PacketPtr pkt) {
  for (const auto &elem : activeWindowRequests)
    for (const auto &req : elem.second)
      if (req->getPacket() == pkt)
        return elem.first;

  panic("Could not find memory request in active window requests map\n");
  return nullptr;
}

void WindowManager::removeCorrespondingWindowRequest(MemoryRequest *mem_req) {
  if (debug())
    DPRINTF(WindowManager, "Clearing Request with addr 0x%016x\n",
            mem_req->getAddress());

  for (const auto &elem : activeWindowRequests) {
    for (auto it = elem.second.begin(); it != elem.second.end(); ++it) {
      if (*it == mem_req) {
        activeWindowRequests[elem.first].erase(it);
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
    owner->activeWindowRequests[this].push_back(read_req);
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

  owner->activeWriteRequests.push_back(write_req);
  owner->activeWindowRequests[this].push_back(write_req);

  currentSPMOffset += owner->dataSize;
  return true;
}
/* Window functions end */
