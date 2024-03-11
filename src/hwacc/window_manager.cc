#include "hwacc/window_manager.hh"
#include "base/trace.hh"
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
  nextTick = nextCycle();
  peRequestLength = 4;

  mmr = new uint8_t[io_size];
  std::fill(mmr, mmr + io_size, 0);
}

WindowManager::PERequest
WindowManager::constructPERequestFromReadRequest(MemoryRequest *readReq) {
  size_t readLen = readReq->getLength();
  if (readLen == peRequestLength) { // 4 bytes
    assert(readReq->getTotalLength() == (Tick)readLen);

    uint32_t result = 0;
    for (int i = readReq->getTotalLength() - 1; i >= 0; i--)
      result = (result << 8) + readReq->getBuffer()[i];

    return (PERequest){(uint8_t)(result >> 24), (uint16_t)((result << 8) >> 16),
                       (uint8_t)((result << 24) >> 24)};
  }

  return (PERequest){0, 0, 0};
}

void WindowManager::recvPacket(PacketPtr pkt) {
  if (pkt->isRead()) {
    if (debug())
      DPRINTF(WindowManager, "Received read packet\n");

    MemoryRequest *readReq = findMemRequest(pkt, activeReadRequests);
    RequestPort *carrierPort = readReq->getCarrierPort();

    if (PEPort *pePort = dynamic_cast<PEPort *>(carrierPort)) {
      PERequest peReq = constructPERequestFromReadRequest(readReq);
      peRequestMapper[pePort->getPENameFromPeerPort()].push_back(peReq);
    }

    removeMemRequest(readReq, activeReadRequests);
  } else if (pkt->isWrite()) {
    if (debug())
      DPRINTF(WindowManager, "Received write packet\n");
  } else {
    panic("Received packet that is neither read nor write\n");
  }

  if (!tickEvent.scheduled()) {
    schedule(tickEvent, nextTick);
  }

  delete pkt;
}

void WindowManager::readFromPort(RequestPort *port, Addr addr, size_t len) {
  MemoryRequest *readReq = new MemoryRequest(addr, len);
  Request::Flags flags;
  RequestPtr req = std::make_shared<Request>(addr, len, flags, masterId);
  PacketPtr pkt = new Packet(req, MemCmd::ReadReq);

  readReq->setCarrierPort(port);
  readReq->setPacket(pkt);
  pkt->allocate();

  if (PEPort *pePort = dynamic_cast<PEPort *>(port)) {
    pePort->sendPacket(pkt);
    activeReadRequests.push_back(readReq);
  }

  if (debug())
    DPRINTF(WindowManager,
            "Trying to read addr: 0x%016x, %d bytes through port: %s\n", addr,
            len, port->name());
}

void WindowManager::tick() {
  if (debug())
    DPRINTF(WindowManager, "Tick!\n");

  /*
    TODO: Complete the tick function which is called every cycle

    Step 1: Check all the pe ports for incoming requests
      Here, after checking for incoming requests, each request (with its added
    info) is added to a mapper which maps the requesting PE (using its name) to
    its requests.
  */

  // Check all the pe ports for incoming requests
  for (auto &port : peStreamPorts) {
    // 1. read the request data from that port (if any)
    // 2. add the requests to the mapper when the results are received

    if (checkPort(port, peRequestLength, true)) {
      readFromPort(port, port->getStartAddress(), peRequestLength);
    }
  }
  // Schedule the next tick
  schedule(tickEvent, nextTick);
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

  if (!tickEvent.scheduled()) {
    schedule(tickEvent, nextTick);
  }

  return pioDelay;
}

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
      owner->schedule(owner->tickEvent, owner->nextTick);
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
      owner->schedule(owner->tickEvent, owner->nextTick);
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
  //             [&memReq](const MemoryRequestPtr mr) { return mr == memReq; });
  // if (it != end(targetVec))
  //   readMemReqs.erase(it);
  // else
  //   DPRINTF(WindowManager,
  //           "Could not find memory request in request queues (for
  //           removal)\n");
}
/* Helper functions end */
