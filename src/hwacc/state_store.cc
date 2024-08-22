#include "hwacc/state_store.hh"
#include "base/trace.hh"
#include "debug/StateStore.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "sim/system.hh"
#include <algorithm>
#include <iterator>
#include <random>
#include <string>

StateStore::StateStore(const StateStoreParams &p)
    : BasicPioDevice(p, p.pio_size), tickEvent(this), deviceName(p.devicename),
      debugEnabled(p.debug_enabled), clockPeriod(p.clock_period),
      masterId(p.system->getRequestorId(this, name())), io_size(p.pio_size),
      io_addr(p.pio_addr), endian(p.system->getGuestByteOrder()) {
  processingDelay = 1000 * clockPeriod;

  mmr = new uint8_t[io_size];
  std::fill(mmr, mmr + io_size, 0);
}

void StateStore::recvPacket(PacketPtr pkt) {
  if (pkt->isRead()) {
    MemoryRequest *read_req = findMemoryRequest(pkt, activeReadRequests);
    RequestPort *carrier_port = read_req->getCarrierPort();
    pkt->writeData(read_req->getBuffer() +
                   (pkt->req->getPaddr() - read_req->getBeginAddr()));

    if (debug())
      DPRINTF(StateStore, "Done with a read. addr: 0x%x, size: %d, data: %s\n",
              pkt->req->getPaddr(), pkt->getSize(), read_req->printBuffer());

    if (PEPort *pe_port = dynamic_cast<PEPort *>(carrier_port)) {
      handlePERequestRead(read_req, pe_port);
    } else {
      panic("Unknown port receiving the read response!\n");
    }

    removeMemoryRequest(read_req, activeReadRequests);
    delete read_req;
  } else if (pkt->isWrite()) {
    MemoryRequest *write_req = findMemoryRequest(pkt, activeWriteRequests);
    RequestPort *carrier_port = write_req->getCarrierPort();

    if (debug())
      DPRINTF(StateStore, "Done with a write. addr: 0x%x, size: %d\n",
              pkt->req->getPaddr(), pkt->getSize());

    if (SPMPort *port = dynamic_cast<SPMPort *>(carrier_port)) {
    } else if (PEPort *port = dynamic_cast<PEPort *>(carrier_port)) {
    } else if (GlobalPort *port = dynamic_cast<GlobalPort *>(carrier_port)) {
    } else {
      panic("Unknown port receiving the write response!\n");
    }

    removeMemoryRequest(write_req, activeWriteRequests);
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

void StateStore::handlePERequestRead(MemoryRequest *read_req, PEPort *pe_port) {
  uint64_t data = *((uint64_t *)read_req->getBuffer());

  if (debug())
    DPRINTF(StateStore, "received new stream input 0x%016x : 0x%016x\n",
            pe_port->getStartAddr(), data);

  if (data == streamTokens.at("A")) {
    ongoingRecordName = "A";
  } else if (data == streamTokens.at("B")) {
    ongoingRecordName = "B";
  } else {
    if (ongoingRecordTimestamp == 0) {
      ongoingRecordTimestamp = data;
      mostRecentStreamTimestamp[ongoingRecordName] = data;
    } else if (ongoingRecordKey == 0)
      ongoingRecordKey = data;
    else
      ongoingRecordValues.push_back(data);
  }

  if (ongoingRecordValues.size() == numRecordValueFields.at(ongoingRecordName)) {
    clearOldEntries();
    matchRecords();
    sendMatchingRecords();
  }
}

void StateStore::clearOldEntries() {
  bool isSameTimestampDivision =
      ((size_t)mostRecentStreamTimestamp["A"] - 1) / windowSize ==
      ((size_t)mostRecentStreamTimestamp["B"] - 1) / windowSize;

  if (!isSameTimestampDivision)
    return;

  size_t new_timestamp_division =
      ((size_t)mostRecentStreamTimestamp["B"] - 1) / windowSize;

  for (auto it = storage.begin(); it != storage.end(); it++) {
    size_t e_ts_div = ((size_t)((*it).timestamp) - 1) / windowSize;
    if (e_ts_div < new_timestamp_division) {
      it = storage.erase(it);
    }
  }
}

void StateStore::sendMatchingRecords() {
  PEPort *port = getPEPortFromAddress(joinResponsePort, responseStreamPorts);
  Addr port_start_addr = port->getStartAddr();
  for (Entry e : matchingEntries) {
    writeToPort(port, e.timestamp, port_start_addr);
    writeToPort(port, e.key, port_start_addr);
    for (uint64_t v : e.values) {
      writeToPort(port, v, port_start_addr);
    }
  }
}

void StateStore::matchRecords() {
  // TODO: copying is not a good way of doing this. so it iterating vector. fix.
  std::copy_if(storage.begin(), storage.end(),
               std::back_inserter(matchingEntries), [this](Entry e) {
                 return e.streamName != ongoingRecordName &&
                        e.key == ongoingRecordKey;
               });
}

StateStore::PEPort *
StateStore::getPEPortFromAddress(Addr addr,
                                 const std::vector<PEPort *> &port_vector) {
  for (PEPort *port : port_vector)
    if (port->getStartAddr() == addr)
      return port;

  panic("cannot find a port with the given address\n");
}

void StateStore::joinLogic() {
  // step 1: read the incoming port in order to see whether there are inputs
  PEPort *request_port =
      getPEPortFromAddress(joinRequestPort, requestStreamPorts);
  if (checkPort(request_port, peRequestLength, true)) {
    readFromPort(request_port, request_port->getStartAddr(), peRequestLength);
  }
}

void StateStore::tick() {
  if (*mmr == 0x04)
    return;

  if (debug())
    DPRINTF(StateStore, "Tick!\n");

  // printPortRanges();

  joinLogic();

  scheduleEvent();
}

/* ************************************************************************** */
/* ************************************************************************** */
/* ************************************************************************** */
/* ************************************************************************** */
/* ************************************************************************** */

/* Utility functions */
MemoryRequest *StateStore::readFromPort(RequestPort *port, Addr addr,
                                        size_t len) {
  MemoryRequest *read_req = new MemoryRequest(addr, len);
  Request::Flags flags;
  RequestPtr req = std::make_shared<Request>(addr, len, flags, masterId);
  PacketPtr pkt = new Packet(req, MemCmd::ReadReq);

  read_req->setCarrierPort(port);
  read_req->setPacket(pkt);
  pkt->allocate();

  if (debug())
    DPRINTF(StateStore,
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

MemoryRequest *StateStore::writeToPort(RequestPort *req_port,
                                       uint64_t input_data, Addr addr,
                                       size_t data_size) {
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
        StateStore,
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

Tick StateStore::read(PacketPtr pkt) {
  if (debug())
    DPRINTF(StateStore,
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

Tick StateStore::write(PacketPtr pkt) {
  if (debug()) {
    DPRINTF(StateStore,
            "The address range associated with this ACC was written to!\n");
    DPRINTF(StateStore, "Packet addr 0x%lx\n", pkt->req->getPaddr());
    DPRINTF(StateStore, "IO addr 0x%lx\n", io_addr);
    DPRINTF(StateStore, "Diff addr 0x%lx\n", pkt->req->getPaddr() - io_addr);
    DPRINTF(StateStore, "Packet val (LE) %d\n", pkt->getLE<uint8_t>());
    DPRINTF(StateStore, "Packet val (BE) %d\n", pkt->getBE<uint8_t>());
    DPRINTF(StateStore, "Packet val %d\n", pkt->get<uint8_t>(endian));
  }

  pkt->writeData(mmr + (pkt->req->getPaddr() - io_addr));
  pkt->makeAtomicResponse();

  scheduleEvent();

  return pioDelay;
}

StateStore::GlobalPort *StateStore::getValidGlobalPort(Addr add, bool read) {
  for (auto port : globalPorts) {
    AddrRangeList adl = port->getAddrRanges();
    for (auto address : adl)
      if (address.contains(add) && !(port->hasRetryPackets()) &&
          !port->waitingForResponse())
        return port;
  }
  return nullptr;
}

StateStore::SPMPort *StateStore::getValidSPMPort(Addr add, bool read) {
  for (auto port : spmPorts) {
    AddrRangeList adl = port->getAddrRanges();
    for (auto address : adl)
      if (!port->isActive() && address.contains(add) &&
          !(port->hasRetryPackets()))
        return port;
  }
  return nullptr;
}

void StateStore::scheduleEvent(Tick when) {
  Tick actual_when = (when == 0) ? curTick() + processingDelay : when;
  if (!tickEvent.scheduled())
    schedule(tickEvent, actual_when);
}
/* Utility functions */

/* Helper functions start */
std::string StateStore::PEPort::getPENameFromPeerPort() {
  // TODO: find a way to get the PE name from the port
  if (!isConnected())
    panic("Port not connected");

  return std::to_string(getId());
}

bool StateStore::checkPort(RequestPort *port, size_t len, bool is_read) {
  if (PEPort *pe_port = dynamic_cast<PEPort *>(port))
    return pe_port->streamValid(len, is_read);
  else
    panic("currently no support for validation check other than PE port\n");

  return false;
}

void StateStore::printPortRanges() {
  for (auto port : localPorts) {
    AddrRangeList adl = port->getAddrRanges();
    for (auto address : adl) {
      if (debug())
        DPRINTF(StateStore, "localPorts: %s, Range: %lx-%lx\n", port->name(),
                address.start(), address.end());
    }
  }
  for (auto port : globalPorts) {
    AddrRangeList adl = port->getAddrRanges();
    for (auto address : adl) {
      if (debug())
        DPRINTF(StateStore, "globalPorts: %s, Range: %lx-%lx\n", port->name(),
                address.start(), address.end());
    }
  }
  for (auto port : spmPorts) {
    AddrRangeList adl = port->getAddrRanges();
    for (auto address : adl) {
      if (debug())
        DPRINTF(StateStore, "spmPorts: %s, Range: %lx-%lx\n", port->name(),
                address.start(), address.end());
    }
  }
  for (auto port : requestStreamPorts) {
    AddrRangeList adl = port->getAddrRanges();
    for (auto address : adl) {
      if (debug())
        DPRINTF(StateStore, "requestStreamPorts: %s, Range: %lx-%lx\n",
                port->name(), address.start(), address.end());
    }
  }
  for (auto port : responseStreamPorts) {
    AddrRangeList adl = port->getAddrRanges();
    for (auto address : adl) {
      if (debug())
        DPRINTF(StateStore, "responseStreamPorts: %s, Range: %lx-%lx\n",
                port->name(), address.start(), address.end());
    }
  }
}

Port &StateStore::getPort(const std::string &if_name, PortID idx) {
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
StateStore::findMemoryRequest(PacketPtr pkt,
                              const std::vector<MemoryRequest *> &target_vec) {
  auto it =
      find_if(begin(target_vec), end(target_vec),
              [&pkt](MemoryRequest *mr) { return mr->getPacket() == pkt; });

  if (it != end(target_vec))
    return *it;

  panic("Could not find memory request in request queues");
  return nullptr;
}

void StateStore::removeMemoryRequest(MemoryRequest *mem_req,
                                     std::vector<MemoryRequest *> &target_vec) {
  if (debug())
    DPRINTF(StateStore, "Clearing Request with addr 0x%016x\n",
            mem_req->getAddress());

  for (auto it = target_vec.begin(); it != target_vec.end(); ++it) {
    if ((*it) == mem_req) {
      it = target_vec.erase(it);
      return;
    }
  }

  // TODO: is it ok if the request is not present?
  if (debug())
    DPRINTF(StateStore,
            "Did not find the memory request with 0x%016x to remove!\n",
            mem_req->getAddress());

  // TODO This is really weird. The code below should work but it doesn't.
  // auto it =
  //     find_if(begin(target_vec), end(target_vec),
  //             [&memReq](const MemoryRequest * mr) { return mr == memReq; });
  // if (it != end(target_vec))
  //   readMemReqs.erase(it);
  // else
  //   DPRINTF(StateStore,
  //           "Could not find memory request in request queues (for
  //           removal)\n");
}
/* Helper functions end */

/* GenericRequestPort functions start */
bool StateStore::GenericRequestPort::recvTimingResp(PacketPtr pkt) {
  owner->recvPacket(pkt);
  return true;
}

void StateStore::GenericRequestPort::recvReqRetry() {
  if (debug())
    DPRINTF(StateStore, "Got a retry...\n");

  while (retryPackets.size() && sendTimingReq(retryPackets.front())) {
    if (debug())
      DPRINTF(StateStore, "Unblocked, sent blocked packet.\n");
    retryPackets.pop();

    owner->scheduleEvent();
  }
}

void StateStore::GenericRequestPort::sendPacket(PacketPtr pkt) {
  if (hasRetryPackets() || !sendTimingReq(pkt)) {
    if (debug())
      DPRINTF(StateStore,
              "sendTiming failed in sendPacket(pkt->req->getPaddr()=0x%x "
              "size=%d)\n",
              (unsigned int)pkt->req->getPaddr(), pkt->req->getSize());
    addRetryPacket(pkt);
  }
}
/* GenericRequestPort functions end */

/* SPMPort functions start */
bool StateStore::SPMPort::recvTimingResp(PacketPtr pkt) {
  owner->recvPacket(pkt);
  return true;
}

void StateStore::SPMPort::recvReqRetry() {
  if (debug())
    DPRINTF(StateStore, "Got a retry...\n");

  while (retryPackets.size() && sendTimingReq(retryPackets.front())) {
    if (debug())
      DPRINTF(StateStore, "Unblocked, sent blocked packet.\n");
    retryPackets.pop();

    owner->scheduleEvent();
  }
}

void StateStore::SPMPort::sendPacket(PacketPtr pkt) {
  if (hasRetryPackets() || !sendTimingReq(pkt)) {
    if (debug())
      DPRINTF(StateStore,
              "sendTiming failed in sendPacket(pkt->req->getPaddr()=0x%x "
              "size=%d)\n",
              (unsigned int)pkt->req->getPaddr(), pkt->req->getSize());
    addRetryPacket(pkt);
  }
}
/* SPMPort functions end */
