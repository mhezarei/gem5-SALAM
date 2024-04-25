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
  struct PESignalRequest {
    uint32_t sourceAddr;
    uint16_t startElement;
    uint16_t length;
  };
  struct PETimeseriesRequest {
    uint64_t startTimestamp;
    uint64_t endTimestamp;
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

  class TimeseriesWindow {
  private:
    enum State {
      none,
      requestCoreStart,
      waitingCoreRangeStart,
      requestCoreEnd,
      waitingCoreRangeEnd,
      checkRange,
      requestChildAddress,
      waitingChildAddress,
      firstScanDone,

      initTraverse,
      requestStat,
      waitingStat,
      startTraverse,
      requestTraverseChildren,
      waitingTraverseChildren,

      computeStat,
      computeStat_RequestLeafChildrenStartAddress,
      computeStat_WaitingLeafChildStartAddress,
      computeStat_RequestCoreChildren,
      computeStat_WaitingCoreChildren,
      computeStat_RequestCoreChildStat,
      computeStat_WaitingCoreChildStat,
      computeStat_RequestResult,
      computeStat_WaitingResult,
      computeStat_SaveResult,
      computeStat_WaitingSaveResult,

      done
    } state;

    std::string windowName;
    WindowManager *owner;
    PETimeseriesRequest peRequest;
    PEPort *correspondingPEPort;
    std::queue<Addr> computationCores;
    std::queue<std::pair<Addr, uint64_t>> partials;
    std::queue<Addr> coreQueue;
    int batchSize;
    int numChildren;
    int numReceivedChildren;
    Addr currentCoreAddr;
    uint64_t currentCoreStart, currentCoreEnd;

    // traversal info
    std::pair<Addr, uint64_t>> currentPartial;
    std::stack<std::pair<Addr, bool>> traverseStack;
    std::vector<uint64_t> endResults;
    Addr leavesStartAddr, leavesEndAddr;
    std::pair<Addr, bool> traverseStackHead;
    PEPort *openPEPort;
    Addr currentCoreChildAddress;
    uint64_t computedResult;
    uint64_t numCheckedNodes;

  public:
    TimeseriesWindow(WindowManager *owner, PEPort *pe_port);
    bool debug() { return owner->debug(); }
    void setPERequest(uint64_t value); // sets either start or end
    std::string name() const { return windowName; }
    bool samePEPort(PEPort *pe_port) { return correspondingPEPort == pe_port; }
    PETimeseriesRequest getPERequest() { return peRequest; }
    void firstScanTick();
    void traverseTick();
    void handleResponseData(uint64_t data);
    bool isDone() { return state == done; }
    PEPort *findPEPortForComputeState();
    bool isLeafNode(Addr node_addr);
    bool coresDone() {return traverseStack.empty() && computationCores.empty() && travereStackHead == nullptr;}
  };

  class Window {
  private:
    std::string windowName;
    WindowManager *owner;
    Addr baseMemoryAddress;
    std::queue<MemoryRequest *> memoryRequests;

    Addr spmBaseAddr;
    Offset currentSPMOffset;
    PEPort *correspondingPEPort;

  public:
    Window(WindowManager *owner, Addr base_memory_addr,
           const std::vector<Offset> &offsets, Addr base_spm_addr,
           PEPort *pe_port);
    bool debug() { return owner->debug(); }
    bool sendMemoryRequest();
    bool sendSPMRequest(uint64_t data);
    std::string name() const { return windowName; }
    PEPort *getCorrespondingPEPort() { return correspondingPEPort; }
    Addr getSPMBaseAddr() { return spmBaseAddr; }
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
  size_t peRequestLength, dataSize, tsDataSize;
  std::vector<bool> finishedPEs;
  uint64_t endToken;
  Addr coresStartAddr;

  // SPM related info
  Addr currentFreeSPMAddress;
  size_t spmSize;

  // Ports
  std::vector<SPMPort *> spmPorts;
  std::vector<LocalPort *> localPorts;
  std::vector<GlobalPort *> globalPorts;
  std::vector<PEPort *> peRequestStreamPorts;
  std::vector<PEPort *> peResponseStreamPorts;

  // Necessary data structures
  std::vector<MemoryRequest *> activeReadRequests;
  std::vector<MemoryRequest *> activeWriteRequests;
  std::queue<Window *> ongoingWindows;
  std::queue<std::pair<Window *, uint64_t>> spmRetryPackets;
  std::map<Window *, std::vector<MemoryRequest *>> activeSignalWindowRequests;
  std::map<TimeseriesWindow *, std::vector<MemoryRequest *>>
      activeTSWindowRequests;

  // timeseries related (for now kept separate)
  std::vector<TimeseriesWindow *> ongoingTSWindows;

  // Utility functions
  MemoryRequest *writeToPort(RequestPort *port, uint64_t data, Addr addr);
  MemoryRequest *readFromPort(RequestPort *port, Addr addr, size_t len);
  void handlePESignalRequest(MemoryRequest *read_req, PEPort *pe_port);
  void handleSignalWindowMemoryResponse(PacketPtr pkt, MemoryRequest *read_req);
  void handleTSWindowMemoryResponse(PacketPtr pkt, MemoryRequest *req);
  void sendPEResponse(PEPort *pe_port, Addr spm_addr);
  void scheduleEvent(Tick when = 0);
  void recvPacket(PacketPtr pkt);
  bool checkPort(RequestPort *port, size_t len, bool is_read);
  MemoryRequest *findMemRequest(PacketPtr pkt,
                                const std::vector<MemoryRequest *> &target_vec);
  void removeMemRequest(MemoryRequest *mem_req,
                        std::vector<MemoryRequest *> &target_vec);
  PESignalRequest constructPERequestFromReadRequest(MemoryRequest *read_req);
  GlobalPort *getValidGlobalPort(Addr add, bool read);
  SPMPort *findAvailableSPMPort();
  Window *findCorrespondingSignalWindow(PacketPtr pkt);
  void removeCorrespondingSignalWindowRequest(MemoryRequest *mem_req);
  TimeseriesWindow *findCorrespondingTSWindow(PacketPtr pkt);
  void removeCorrespondingTSWindowRequest(MemoryRequest *mem_req);
  void signalTick();
  void tsTick();
  void handlePETimeseriesRequest(MemoryRequest *read_req, PEPort *pe_port);
  uint64_t extractPERequestValue(MemoryRequest *read_req);

public:
  WindowManager(const WindowManagerParams &p);
  virtual Tick read(PacketPtr pkt) override;
  virtual Tick write(PacketPtr pkt) override;
  Port &getPort(const std::string &if_name,
                PortID idx = InvalidPortID) override;
  std::string getName() const { return name(); }
};

#endif // __HWACC_WINDOW_MANAGER_HH__