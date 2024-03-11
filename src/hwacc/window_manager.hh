#ifndef __HWACC_WINDOW_MANAGER_HH__
#define __HWACC_WINDOW_MANAGER_HH__

#include "dev/io_device.hh"
#include "hwacc/LLVMRead/src/mem_request.hh"
#include "hwacc/scratchpad_memory.hh"
#include "hwacc/stream_port.hh"
#include "params/WindowManager.hh"

#include <queue>
#include <vector>

class WindowManager : public BasicPioDevice {
public:
  bool debug() const { return debugEnabled; }

private:
  struct PERequest {
    uint8_t sourceID;
    uint16_t startElement;
    uint8_t Length;
  };

  class SPMPort : public ScratchpadRequestPort {
    friend class WindowManager;

  private:
    WindowManager *owner;

  public:
    SPMPort(const std::string &name, WindowManager *owner,
            PortID id = InvalidPortID)
        : ScratchpadRequestPort(name, owner, id), owner(owner) {}

  private:
    std::queue<PacketPtr> retryPackets;

  protected:
    void addRetryPacket(PacketPtr pkt) { retryPackets.push(pkt); }
    bool hasRetryPackets() { return !retryPackets.empty(); }

    virtual bool recvTimingResp(PacketPtr pkt);
    virtual void recvReqRetry();
    virtual void recvRangeChange(){};
    void sendPacket(PacketPtr pkt);
    bool debug() { return owner->debug(); }
  };

  class GenericRequestPort : public StreamRequestPort {
    friend class WindowManager;

  protected:
    WindowManager *owner;

  public:
    GenericRequestPort(const std::string &name, WindowManager *owner,
                       PortID id = InvalidPortID)
        : StreamRequestPort(name, owner, id), owner(owner) {}

  protected:
    std::queue<PacketPtr> retryPackets;

    void addRetryPacket(PacketPtr pkt) { retryPackets.push(pkt); }
    bool hasRetryPackets() { return !retryPackets.empty(); }

    virtual bool recvTimingResp(PacketPtr pkt);
    virtual void recvReqRetry();
    virtual void recvRangeChange(){};
    void sendPacket(PacketPtr pkt);
    bool debug() { return owner->debug(); }
  };

  class PEPort : private GenericRequestPort {
    friend class WindowManager;

  public:
    PEPort(const std::string &name, WindowManager *owner,
           PortID id = InvalidPortID)
        : GenericRequestPort(name, owner, id) {}

  protected:
    std::string getPENameFromPeerPort();
    Addr getStartAddress() { return (*getAddrRanges().begin()).start(); }
  };

  class LocalPort : private GenericRequestPort {
    friend class WindowManager;

  public:
    LocalPort(const std::string &name, WindowManager *owner,
              PortID id = InvalidPortID)
        : GenericRequestPort(name, owner, id) {}
  };

  class TickEvent : public Event {
  private:
    WindowManager *owner;

  public:
    TickEvent(WindowManager *_owner) : Event(CPU_Tick_Pri), owner(_owner) {}
    void process() { owner->tick(); }
    virtual const char *description() const { return "WindowManager tick"; }
    bool debug() { return owner->debug(); }
  };

  TickEvent tickEvent;
  virtual void tick();

  // Class parameters
  std::string deviceName;
  bool debugEnabled;
  int processingDelay, clockPeriod;
  Tick nextTick;
  RequestorID masterId;
  Addr io_size;
  Addr io_addr;
  uint8_t *mmr;
  ByteOrder endian;

  // Class variables
  size_t peRequestLength;

  // Ports
  std::vector<SPMPort *> spmPorts;
  std::vector<LocalPort *> localPorts;
  std::vector<PEPort *> peStreamPorts;

  // Necessary data structures
  std::map<std::string, std::vector<PERequest>> peRequestMapper;
  std::vector<MemoryRequest *> activeReadRequests;

  void recvPacket(PacketPtr pkt);
  bool checkPort(RequestPort *port, size_t len, bool isRead);
  void readFromPort(RequestPort *port, Addr addr, size_t len);
  MemoryRequest *findMemRequest(PacketPtr pkt,
                                const std::vector<MemoryRequest *> &targetVec);
  void removeMemRequest(MemoryRequest *memReq,
                        std::vector<MemoryRequest *> &targetVec);
  PERequest constructPERequestFromReadRequest(MemoryRequest *readReq);

public:
  WindowManager(const WindowManagerParams &p);
  virtual Tick read(PacketPtr pkt);
  virtual Tick write(PacketPtr pkt);
  Port &getPort(const std::string &if_name,
                PortID idx = InvalidPortID) override;
};

#endif // __HWACC_WINDOW_MANAGER_HH__