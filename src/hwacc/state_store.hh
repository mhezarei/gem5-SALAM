#ifndef __HWACC_STATE_STORE_HH__
#define __HWACC_STATE_STORE_HH__

#include "dev/io_device.hh"
#include "hwacc/LLVMRead/src/mem_request.hh"
#include "hwacc/scratchpad_memory.hh"
#include "hwacc/stream_port.hh"
#include "params/StateStore.hh"

#include <queue>
#include <string>
#include <vector>

class StateStore : public BasicPioDevice {
public:
  bool debug() const { return debugEnabled; }

private:
  class SPMPort : public ScratchpadRequestPort {
    friend class StateStore;

  private:
    StateStore *owner;
    std::queue<PacketPtr> retryPackets;
    MemoryRequest *activeReadRequest, *activeWriteRequest;

  public:
    SPMPort(const std::string &name, StateStore *owner,
            PortID id = InvalidPortID)
        : ScratchpadRequestPort(name, owner, id), owner(owner) {}

  protected:
    void addRetryPacket(PacketPtr pkt) { retryPackets.push(pkt); }
    bool hasRetryPackets() { return !retryPackets.empty(); }
    void setActiveReadRequest(MemoryRequest *req) { activeReadRequest = req; }
    void setActiveWriteRequest(MemoryRequest *req) { activeWriteRequest = req; }
    bool isReading() { return activeReadRequest == nullptr; }
    bool isWriting() { return activeWriteRequest == nullptr; }
    bool isActive() { return !isReading() && !isWriting(); }

    virtual bool recvTimingResp(PacketPtr pkt);
    virtual void recvReqRetry();
    virtual void recvRangeChange() {};
    void sendPacket(PacketPtr pkt);
    bool debug() { return owner->debug(); }

    Addr getStartAddr() { return (*getAddrRanges().begin()).start(); }
  };

  class GenericRequestPort : public StreamRequestPort {
    friend class StateStore;
    friend class SignalWindow;

  protected:
    StateStore *owner;
    PacketPtr waitingResponsePkt;

  public:
    GenericRequestPort(const std::string &name, StateStore *owner,
                       PortID id = InvalidPortID)
        : StreamRequestPort(name, owner, id), owner(owner) {}

  protected:
    std::queue<PacketPtr> retryPackets;

    void addRetryPacket(PacketPtr pkt) { retryPackets.push(pkt); }
    bool hasRetryPackets() { return !retryPackets.empty(); }
    Addr getStartAddr() { return (*getAddrRanges().begin()).start(); }
    bool waitingForResponse() { return waitingResponsePkt != nullptr; }

    virtual bool recvTimingResp(PacketPtr pkt);
    virtual void recvReqRetry();
    virtual void recvRangeChange() {};
    void sendPacket(PacketPtr pkt);
    bool debug() { return owner->debug(); }
  };

  class PEPort : public GenericRequestPort {
    friend class StateStore;

  public:
    PEPort(const std::string &name, StateStore *owner,
           PortID id = InvalidPortID)
        : GenericRequestPort(name, owner, id) {}

  protected:
    std::string getPENameFromPeerPort();
  };

  class LocalPort : public GenericRequestPort {
    friend class StateStore;

  public:
    LocalPort(const std::string &name, StateStore *owner,
              PortID id = InvalidPortID)
        : GenericRequestPort(name, owner, id) {}
  };

  class GlobalPort : public GenericRequestPort {
    friend class StateStore;

  public:
    GlobalPort(const std::string &name, StateStore *owner,
               PortID id = InvalidPortID)
        : GenericRequestPort(name, owner, id) {}
  };

  class TickEvent : public Event {
  private:
    StateStore *owner;

  public:
    TickEvent(StateStore *_owner) : Event(CPU_Tick_Pri), owner(_owner) {}
    void process() { owner->tick(); }
    virtual const char *description() const { return "StateStore tick"; }
    bool debug() { return owner->debug(); }
  };

  TickEvent tickEvent;
  virtual void tick();

  // Class parameters
  std::string deviceName;
  bool debugEnabled;
  int processingDelay, clockPeriod;
  RequestorID masterId;
  Addr io_size;
  Addr io_addr;
  uint8_t *mmr;
  ByteOrder endian;

  // Ports
  std::vector<SPMPort *> spmPorts;
  std::vector<LocalPort *> localPorts;
  std::vector<GlobalPort *> globalPorts;
  std::vector<PEPort *> requestStreamPorts;
  std::vector<PEPort *> responseStreamPorts;

  // data structures
  std::vector<MemoryRequest *> activeReadRequests;
  std::vector<MemoryRequest *> activeWriteRequests;

  /* ********************************************************************* */

  // constants
  inline static const std::map<std::string, size_t> numRecordValueFields = {
      {"A", 2}, {"B", 1}}; // including timestamp
  inline static const std::map<std::string, uint64_t> streamTokens = {
      {"A", 0x40F8E8E000000000}, {"B", 0x4118B91000000000}};
  inline static const Addr joinRequestPort = 0x100204c0;
  inline static const Addr joinResponsePort = 0x10020100;
  inline static const size_t peRequestLength = 8;
  inline static const size_t windowSize = 5;

  // definitions
  struct Entry {
    std::string streamName;
    uint64_t timestamp;
    uint64_t key;
    std::vector<uint64_t> values;
  };

  //
  uint64_t ongoingRecordTimestamp;
  uint64_t ongoingRecordKey;
  std::vector<uint64_t> ongoingRecordValues;
  std::string ongoingRecordName = "";
  std::vector<Entry> storage;
  std::vector<Entry> matchingEntries;
  std::map<std::string, uint64_t> mostRecentStreamTimestamp = {{"A", 0},
                                                               {"B", 0}};

  void handlePERequestRead(MemoryRequest *read_req, PEPort *pe_port);
  void clearOldEntries();
  void matchRecords();
  void sendMatchingRecords();
  PEPort *getPEPortFromAddress(Addr addr,
                               const std::vector<PEPort *> &port_vector);
  void joinLogic();

  /* *********************************************************************
   */

  // Utility functions
  MemoryRequest *writeToPort(RequestPort *port, uint64_t data, Addr addr,
                             size_t data_size = 8);
  MemoryRequest *readFromPort(RequestPort *port, Addr addr, size_t len);
  void recvPacket(PacketPtr pkt);
  bool checkPort(RequestPort *port, size_t len, bool is_read);
  MemoryRequest *
  findMemoryRequest(PacketPtr pkt,
                    const std::vector<MemoryRequest *> &target_vec);
  void removeMemoryRequest(MemoryRequest *mem_req,
                           std::vector<MemoryRequest *> &target_vec);
  GlobalPort *getValidGlobalPort(Addr add, bool read);
  SPMPort *getValidSPMPort(Addr add, bool read = true);
  void printPortRanges();
  void scheduleEvent(Tick when = 0);

public:
  StateStore(const StateStoreParams &p);
  virtual Tick read(PacketPtr pkt) override;
  virtual Tick write(PacketPtr pkt) override;
  Port &getPort(const std::string &if_name,
                PortID idx = InvalidPortID) override;
  std::string getName() const { return name(); }
};

#endif // __HWACC_STATE_STORE_HH__