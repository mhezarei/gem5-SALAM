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
  // TODO: change sourceID to real address
  struct PERequest {
    uint8_t sourceID;
    uint16_t startElement;
    uint8_t length;
  };
  typedef uint64_t Offset;

  class SPMPort : public ScratchpadRequestPort {
    friend class WindowManager;

  private:
    WindowManager *owner;
    std::queue<PacketPtr> retryPackets;
    MemoryRequest *activeReadRequest, *activeWriteRequest;

  public:
    SPMPort(const std::string &name, WindowManager *owner,
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
    virtual void recvRangeChange(){};
    void sendPacket(PacketPtr pkt);
    bool debug() { return owner->debug(); }
  };

  class GenericRequestPort : public StreamRequestPort {
    friend class WindowManager;
    friend class Window;

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

  class PEPort : public GenericRequestPort {
    friend class WindowManager;

  public:
    PEPort(const std::string &name, WindowManager *owner,
           PortID id = InvalidPortID)
        : GenericRequestPort(name, owner, id) {}

  protected:
    std::string getPENameFromPeerPort();
    Addr getStartAddress() { return (*getAddrRanges().begin()).start(); }
  };

  class LocalPort : public GenericRequestPort {
    friend class WindowManager;

  public:
    LocalPort(const std::string &name, WindowManager *owner,
              PortID id = InvalidPortID)
        : GenericRequestPort(name, owner, id) {}
  };

  class GlobalPort : public GenericRequestPort {
    friend class WindowManager;

  public:
    GlobalPort(const std::string &name, WindowManager *owner,
               PortID id = InvalidPortID)
        : GenericRequestPort(name, owner, id) {}
  };

  class Window {
  private:
    std::string windowName;
    WindowManager *owner;
    Addr baseMemoryAddress;
    std::queue<MemoryRequest *> memoryRequests;
    std::vector<MemoryRequest *> sentMemoryRequests;

    Addr spmBaseAddr;
    Offset currentSPMOffset;

  public:
    Window(WindowManager *owner, Addr base_memory_addr,
           const std::vector<Offset> &offsets, Addr base_spm_addr);
    bool debug() { return owner->debug(); }
    bool sendMemoryRequest();
    void sendSPMRequest(SPMPort *spm_port, uint64_t data);
    std::string name() const { return windowName; }
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
  RequestorID masterId;
  Addr io_size;
  Addr io_addr;
  uint8_t *mmr;
  ByteOrder endian;

  // Class variables
  size_t requestLength;

  // SPM related info
  Addr currentFreeSPMAddress;
  size_t spmSize;

  // Ports
  std::vector<SPMPort *> spmPorts;
  std::vector<LocalPort *> localPorts;
  std::vector<GlobalPort *> globalPorts;
  std::vector<PEPort *> peStreamPorts;

  // Necessary data structures
  std::vector<MemoryRequest *> activeReadRequests;
  std::vector<MemoryRequest *> activeWriteRequests;
  std::queue<Window *> ongoingWindows;
  // TODO: this will be replaced with actual address communication
  std::map<uint8_t, Addr> sourceIDToAddr;
  std::queue<std::pair<Window *, uint64_t>> spmRetryPackets;
  std::vector<std::pair<MemoryRequest *, Window *>> activeWindowMemoryRequests;

  void handlePERequest(MemoryRequest *read_req);
  void handleWindowMemoryResponse(PacketPtr pkt, MemoryRequest *read_req);
  void scheduleEvent(Tick when = 0);
  void recvPacket(PacketPtr pkt);
  bool checkPort(RequestPort *port, size_t len, bool isRead);
  void readFromPort(RequestPort *port, Addr addr, size_t len);
  MemoryRequest *findMemRequest(PacketPtr pkt,
                                const std::vector<MemoryRequest *> &targetVec);
  void removeMemRequest(MemoryRequest *mem_req,
                        std::vector<MemoryRequest *> &targetVec);
  PERequest constructPERequestFromReadRequest(MemoryRequest *read_req);
  GlobalPort *getValidGlobalPort(Addr add, bool read);
  SPMPort *findAvailableSPMPort();
  Window *findCorrespondingWindow(PacketPtr pkt);
  void removeActiveWindowRequest(MemoryRequest *mem_req);

public:
  WindowManager(const WindowManagerParams &p);
  virtual Tick read(PacketPtr pkt) override;
  virtual Tick write(PacketPtr pkt) override;
  Port &getPort(const std::string &if_name,
                PortID idx = InvalidPortID) override;
  std::string getName() const { return name(); }
};

#endif // __HWACC_WINDOW_MANAGER_HH__