#ifndef __HWACC_WINDOW_MANAGER_HH__
#define __HWACC_WINDOW_MANAGER_HH__

#include "dev/io_device.hh"
#include "hwacc/LLVMRead/src/mem_request.hh"
#include "hwacc/scratchpad_memory.hh"
#include "hwacc/stream_port.hh"
#include "params/WindowManager.hh"

#include <queue>
#include <string>
#include <vector>

class WindowManager : public BasicPioDevice {
public:
  bool debug() const { return debugEnabled; }

private:
  inline static const size_t peRequestLength = 8;
  inline static const size_t tsDataSize = 8;
  inline static const size_t signalDataSize = 4;
  inline static const uint64_t endToken = 0xFFFFFFFFFFFFFFFF;
  inline static const Addr spmAddr = 0x10021080;
  inline static const size_t spmSize = 1024;
  inline static const Addr tsCoresStartAddr = 0x80C00000;
  inline static const std::string operatingMode = "timeseries";

private:
  struct SignalPERequest {
    uint32_t sourceAddr;
    uint16_t startElement;
    uint16_t length;
  };
  struct TimeseriesPERequest {
    uint64_t startTimestamp = UINT64_MAX;
    uint64_t endTimestamp = UINT64_MAX;
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
    virtual void recvRangeChange() {};
    void sendPacket(PacketPtr pkt);
    bool debug() { return owner->debug(); }

    Addr getStartAddr() { return (*getAddrRanges().begin()).start(); }
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
    virtual void recvRangeChange() {};
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
    Addr getStartAddr() { return (*getAddrRanges().begin()).start(); }
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

  class TickEvent : public Event {
  private:
    WindowManager *owner;

  public:
    TickEvent(WindowManager *_owner) : Event(CPU_Tick_Pri), owner(_owner) {}
    void process() { owner->tick(); }
    virtual const char *description() const { return "WindowManager tick"; }
    bool debug() { return owner->debug(); }
  };

  class TimeseriesWindow {
  private:
    inline static const bool shouldUseCache = false;
    inline static const size_t batchSize = 64;
    inline static const size_t numChildren = 16;
    inline static const size_t cacheEntrySize = 24;
    /*
      8KB data: 0x80c00118 - 0x80C00460
      64MB data: 0x80c0a218 - 0x80ca2180
      1GB data: 0x80ca2218 - 0x81622180
    */
    inline static const Addr leafCoresStartAddr = 0x80c0a218;
    inline static const Addr leafCoresEndAddr = 0x80ca2180;

    inline static const size_t coreRangeStartOffset = 0;
    inline static const size_t coreRangeEndOffset = tsDataSize;
    inline static const size_t coreStatOffset = 2 * tsDataSize;

    enum State {
      none,

      requestCoreStart,
      waitingCoreStart,
      requestCoreEnd,
      waitingCoreEnd,
      checkRange,
      requestChildAddr,
      waitingChildAddr,
      firstScanDone,

      initTraverse,
      requestStat,
      waitingStat,
      startTraverse,
      requestTraverseChildren,
      waitingTraverseChildren,

      checkCache,
      checkCache_RequestCoreStart,
      checkCache_WaitingCoreStart,
      checkCache_RequestCoreEnd,
      checkCache_WaitingCoreEnd,
      checkCache_RequestScanCacheEntry,
      checkCache_WaitingScanCacheEntry,
      checkCache_RequestFoundEntryStat,
      checkCache_WaitingFoundEntryStat,
      checkCache_UpdateFoundEntryAccessCycle,
      checkCache_NotFound,

      saveCache,
      saveCache_RequestCoreStart,
      saveCache_WaitingCoreStart,
      saveCache_RequestCoreEnd,
      saveCache_WaitingCoreEnd,
      saveCache_RequestEmptyEntry,
      saveCache_WaitingEmptyEntry,
      saveCache_RequestCoreStat,
      saveCache_WaitingCoreStat,
      saveCache_SaveCacheEntry,
      saveCache_Replace,

      computeStat,
      computeStat_RequestLeafChildrenStartAddr,
      computeStat_WaitingLeafChildStartAddr,
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
    TimeseriesPERequest peRequest;
    PEPort *correspondingPEPort;
    std::queue<Addr> computationCores;
    std::queue<std::pair<Addr, uint64_t>> partials;
    std::queue<Addr> coreQueue;
    int numReceivedChildren;
    Addr currentCoreAddr;
    uint64_t currentCoreStart, currentCoreEnd;
    Addr currentComputationCoreAddr;

    // traversal info
    std::pair<Addr, uint64_t> currentPartial;
    std::stack<std::pair<Addr, bool>> traverseStack;
    std::vector<uint64_t> endResults;
    std::pair<Addr, bool> traverseStackHead;
    PEPort *openPEPort;
    Addr baseMemoryAddr;
    uint64_t computedResult;
    uint64_t numCheckedNodes;

    // cache-related info
    size_t maxCacheIndex;
    Addr currentCoreHash;
    size_t currentCacheEntryIndex;
    Addr saveCacheCoreAddr;
    Addr saveCacheEntryStat;
    Addr checkCacheCoreAddr;
    Tick minAccessTick;
    size_t minAccessTickIndex;

  public:
    TimeseriesWindow(WindowManager *owner, PEPort *pe_port);
    bool debug() { return owner->debug(); }
    void setTimeseriesPERequest(uint64_t value); // sets either start or end
    std::string name() const { return windowName; }
    bool samePEPort(PEPort *pe_port) { return correspondingPEPort == pe_port; }
    bool waitingToStart() { return state == none; }
    TimeseriesPERequest getPERequest() { return peRequest; }
    void firstScanTick();
    void traverseTick();
    void handleMemoryResponseData(uint64_t data);
    bool isDone() { return state == done; }
    PEPort *findPEPortForComputeState();
    bool isLeafNode(Addr node_addr);
    bool coresDone() {
      return traverseStack.empty() && computationCores.empty();
    }
    bool useCache() { return shouldUseCache; }
  };

  class Window {
  private:
    std::string windowName;
    WindowManager *owner;
    Addr baseMemoryAddr;
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

  TickEvent tickEvent;
  virtual void tick();
  void signalTick();
  void timeseriesTick();

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
  std::vector<bool> finishedPEs;

  // SPM related info
  Addr signalCurrentFreeSPMAddr;

  // Ports
  std::vector<SPMPort *> spmPorts;
  std::vector<LocalPort *> localPorts;
  std::vector<GlobalPort *> globalPorts;
  std::vector<PEPort *> peRequestStreamPorts;
  std::vector<PEPort *> peResponseStreamPorts;

  // Necessary data structures
  std::queue<std::pair<Window *, uint64_t>> spmRetryPackets;
  std::vector<MemoryRequest *> activeReadRequests;
  std::vector<MemoryRequest *> activeWriteRequests;
  // TODO: change this to vector with the necessary changes
  std::queue<Window *> activeSignalWindows;
  std::vector<TimeseriesWindow *> activeTimeseriesWindows;
  std::map<Window *, std::vector<MemoryRequest *>> activeSignalWindowRequests;
  std::map<TimeseriesWindow *, std::vector<MemoryRequest *>>
      activeTimeseriesWindowRequests;

  // Utility functions
  MemoryRequest *writeToPort(RequestPort *port, uint64_t data, Addr addr);
  MemoryRequest *readFromPort(RequestPort *port, Addr addr, size_t len);
  void scheduleEvent(Tick when = 0);
  void recvPacket(PacketPtr pkt);
  bool checkPort(RequestPort *port, size_t len, bool is_read);
  MemoryRequest *findMemRequest(PacketPtr pkt,
                                const std::vector<MemoryRequest *> &target_vec);
  void removeRequest(MemoryRequest *mem_req,
                     std::vector<MemoryRequest *> &target_vec);
  GlobalPort *getValidGlobalPort(Addr add, bool read);
  SPMPort *getValidSPMPort(Addr add, bool read = true);
  uint64_t extractPERequestValue(MemoryRequest *read_req);

  // Signal utility functions
  void handleSignalPEResponse(MemoryRequest *read_req, PEPort *pe_port);
  void handleSignalMemoryResponse(PacketPtr pkt, MemoryRequest *read_req);
  void sendSPMAddrToPE(PEPort *pe_port, Addr spm_addr);
  SignalPERequest constructSignalPERequest(MemoryRequest *read_req);
  Window *findCorrespondingSignalWindow(PacketPtr pkt);
  void removeSignalWindowRequest(MemoryRequest *mem_req);
  bool isSignalMode() { return operatingMode == "signal"; }

  // Timeseries utility functions
  void handleTimeseriesMemoryResponse(PacketPtr pkt, MemoryRequest *req);
  TimeseriesWindow *findCorrespondingTimeseriesWindow(PacketPtr pkt);
  void removeTimeseriesWindowRequest(MemoryRequest *mem_req);
  void handleTimeseriesPEResponse(MemoryRequest *read_req, PEPort *pe_port);
  bool isTimeseriesMode() { return operatingMode == "timeseries"; }
  bool allTimeseriesWindowsDone() {
    return std::all_of(activeTimeseriesWindows.begin(),
                       activeTimeseriesWindows.end(),
                       [](auto &w) { return !w->isDone(); });
  }

public:
  WindowManager(const WindowManagerParams &p);
  virtual Tick read(PacketPtr pkt) override;
  virtual Tick write(PacketPtr pkt) override;
  Port &getPort(const std::string &if_name,
                PortID idx = InvalidPortID) override;
  std::string getName() const { return name(); }
};

#endif // __HWACC_WINDOW_MANAGER_HH__