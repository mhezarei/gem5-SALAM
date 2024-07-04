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
  inline static const uint64_t endToken = UINT64_MAX;
  inline static const Addr spmAddr = 0x10021080;
  inline static const size_t spmSize = 64 * 1024;
  inline static const std::string operatingMode = "timeseries";
  inline static const bool fakeValues = false;

  inline static const size_t numPerPERequests = 1024;
  inline static const size_t perRequestPEConcurrentTimeseriesWindows = 8;
  inline static const std::vector<Addr> calcAddresses = {
      0x100201c0, 0x10020240, 0x100202c0, 0x10020340, 0x100203c0, 0x10020440,
      0x100204c0, 0x10020540, 0x100205c0, 0x10020640, 0x100206c0, 0x10020740,
      0x100207c0, 0x10020840, 0x100208c0, 0x10020940, 0x100209c0, 0x10020a40,
      0x10020ac0, 0x10020b40, 0x10020bc0, 0x10020c40, 0x10020cc0, 0x10020d40,
      0x10020dc0, 0x10020e40, 0x10020ec0, 0x10020f40, 0x10020fc0, 0x10021040,
      0x100210c0, 0x10021140, 0x100211c0, 0x10021240, 0x100212c0, 0x10021340,
      0x100213c0, 0x10021440, 0x100214c0, 0x10021540, 0x100215c0, 0x10021640,
      0x100216c0, 0x10021740, 0x100217c0, 0x10021840, 0x100218c0, 0x10021940,
      0x100219c0, 0x10021a40, 0x10021ac0, 0x10021b40, 0x10021bc0, 0x10021c40,
      0x10021cc0, 0x10021d40, 0x10021dc0, 0x10021e40, 0x10021ec0, 0x10021f40,
      0x10021fc0, 0x10022040, 0x100220c0, 0x10022140, 0x100221c0, 0x10022240,
      0x100222c0, 0x10022340, 0x100223c0, 0x10022440, 0x100224c0, 0x10022540,
      0x100225c0, 0x10022640, 0x100226c0, 0x10022740, 0x100227c0, 0x10022840,
      0x100228c0, 0x10022940, 0x100229c0, 0x10022a40, 0x10022ac0, 0x10022b40,
      0x10022bc0, 0x10022c40, 0x10022cc0, 0x10022d40, 0x10022dc0, 0x10022e40,
      0x10022ec0, 0x10022f40, 0x10022fc0, 0x10023040, 0x100230c0, 0x10023140,
      0x100231c0, 0x10023240, 0x100232c0, 0x10023340, 0x100233c0, 0x10023440,
      0x100234c0, 0x10023540, 0x100235c0, 0x10023640, 0x100236c0, 0x10023740,
      0x100237c0, 0x10023840, 0x100238c0, 0x10023940, 0x100239c0, 0x10023a40,
      0x10023ac0, 0x10023b40, 0x10023bc0, 0x10023c40, 0x10023cc0, 0x10023d40,
      0x10023dc0, 0x10023e40, 0x10023ec0, 0x10023f40, 0x10023fc0, 0x10024040,
      0x100240c0, 0x10024140,
  };
  inline static const std::vector<Addr> reqPEAddresses = {
      0x1002eb40, 0x1002ec80, 0x1002edc0, 0x1002ef00, 0x1002f040, 0x1002f180,
      0x1002f2c0, 0x1002f400, 0x1002f540, 0x1002f680, 0x1002f7c0, 0x1002f900,
      0x1002fa40, 0x1002fb80, 0x1002fcc0, 0x1002fe00,
  };

private:
  struct SignalPERequest {
    uint32_t sourceAddr;
    uint16_t startElement;
    uint16_t length;
  };
  struct TimeseriesRange {
    uint64_t start = UINT64_MAX;
    uint64_t end = UINT64_MAX;

    bool operator<(const TimeseriesRange &other) const {
      return std::make_pair(start, end) <
             std::make_pair(other.start, other.end);
    }

    bool operator==(const TimeseriesRange &other) const {
      return start == other.start && end == other.end;
    }
  };
  struct CacheEntry {
    size_t numAccesses;
    TimeseriesRange range;
    uint64_t stat;
  };
  // TODO: clean this up please
  inline static const size_t cacheEntrySize = sizeof(CacheEntry);
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
    friend class SignalWindow;

  protected:
    WindowManager *owner;
    PacketPtr waitingResponsePkt;

  public:
    GenericRequestPort(const std::string &name, WindowManager *owner,
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
    friend class WindowManager;

  public:
    PEPort(const std::string &name, WindowManager *owner,
           PortID id = InvalidPortID)
        : GenericRequestPort(name, owner, id) {}

  protected:
    std::string getPENameFromPeerPort();
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
    inline static const std::string cacheType = "limited";
    inline static const uint64_t limitedCacheMinDifference = 4095;
    inline static const size_t batchSize = 4;
    inline static const size_t numChildren = 4;
    inline static const Addr tsCoresStartAddr = 0x80c00000;
    /*
      F4B4: 0x856aaa98 - 0x936aaa98
      F8B8: 0x82524918 - 0x8d524918
      F16B16: 0x81622218 - 0x8ae22218
      F32B32: 0x81508418 - 0x92d08418
      F64B64: 0x80e20818 - 0x89420818
      F64B4: 0x80e20818 - 0x89420818
      small: 0x80c00118 - 0x80c00498
    */
    inline static const Addr leafCoresStartAddr = 0x856aaa98;
    inline static const Addr leafCoresEndAddr = 0x936aaa98;

    inline static const size_t coreRangeStartOffset = 0;
    inline static const size_t coreRangeEndOffset = tsDataSize;
    inline static const size_t coreStatOffset = 2 * tsDataSize;

    enum State {
      none,

      checkCache,

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

      computeStat,
      computeStat_RequestLeafValuesStartAddr,
      computeStat_WaitingLeafValuesStartAddr,
      computeStat_RequestCoreChildAddr,
      computeStat_WaitingCoreChildAddr,
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

    TimeseriesRange initialQuery;
    std::vector<TimeseriesRange> subQueries;

    PEPort *correspondingPEPort;
    std::deque<Addr> computationCores;
    std::queue<std::pair<Addr, uint64_t>> partials;
    std::queue<Addr> coreQueue;
    int numReceivedChildren;
    Addr currentCoreAddr;
    uint64_t currentCoreStart, currentCoreEnd;
    Addr currentComputationCoreAddr;

    std::deque<Addr> childrenPointers;

    // traversal info
    std::pair<Addr, uint64_t> currentPartial;
    std::stack<std::pair<Addr, bool>> traverseStack;
    std::vector<uint64_t> endResults;
    std::pair<Addr, bool> traverseStackHead;
    Addr computeStatChildAddr;
    uint64_t computedResult;
    uint64_t numCheckedNodes;

    // Tkh, koosi sher
    PEPort *connectedCalcPort;

  public:
    TimeseriesWindow(WindowManager *owner, PEPort *pe_port, size_t id);
    bool debug() { return owner->debug(); }
    void setTimeseriesPERequest(uint64_t value); // sets either start or end
    std::string name() const { return windowName; }
    bool samePEPort(PEPort *pe_port) { return correspondingPEPort == pe_port; }
    bool waitingToStart() { return state == none; }
    TimeseriesRange getPERequest() { return initialQuery; }
    void firstScanTick();
    void traverseTick();
    void handleMemoryResponseData(uint64_t data);
    bool isDone() { return state == done; }
    PEPort *findFreeCalcUnitPort();
    bool isLeafNode(Addr node_addr);
    bool coresDone() {
      return traverseStack.empty() && computationCores.empty();
    }
    bool useCache() { return cacheType != ""; }
    bool useLimitedCache() { return cacheType == "limited"; }
    bool useIdealCache() { return cacheType == "ideal"; }
    void checkCacheFunction();
    void saveCacheFunction(Addr cc_address, uint64_t cc_stat);
  };

  class SignalWindow {
  private:
    inline static const bool shouldUseCache = false;
    inline static const int numCores = 6;
    inline static const std::map<std::string, Addr> coreStreamAddr = {
        {"0_3", 0}, {"1_2", 0}, {"2_4", 0}, {"3_5", 0}, {"4_5", 0}};
    inline static const std::map<std::string, Addr> wmCoreStreamAddr = {
        {"0", 0}, {"1", 0}, {"2", 0}, {"3", 0}, {"4_5", 0}, {"5", 0}};

  private:
    std::string windowName;
    WindowManager *owner;
    Addr baseMemoryAddr;
    std::queue<MemoryRequest *> memoryRequests;

    Addr spmBaseAddr;
    Offset currentSPMOffset;
    PEPort *correspondingPEPort;

  public:
    SignalWindow(WindowManager *owner, Addr base_memory_addr,
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

  // Ports
  std::vector<SPMPort *> spmPorts;
  std::vector<LocalPort *> localPorts;
  std::vector<GlobalPort *> globalPorts;
  std::vector<PEPort *> peRequestStreamPorts;
  std::vector<PEPort *> peResponseStreamPorts;

  // Tkh, koosi sher
  std::map<Addr, bool> calcStatus;
  std::queue<TimeseriesRange> requests;
  std::map<Addr, std::vector<TimeseriesRange>> perPERequests;
  std::map<Addr, TimeseriesRange> ccAddressToRange;

  // Timeseries cache
  std::vector<CacheEntry> timeseriesCache;
  size_t numCacheAccesses, numCacheHits, numCacheMisses, numCacheReplacements,
      numCacheInsertions;
  float avgSavedPortion;
  std::map<TimeseriesRange, size_t> missedRanges;
  std::map<TimeseriesRange, size_t> hitRanges;
  std::map<TimeseriesRange, size_t> replacedRanges;
  std::vector<Addr> fixedCacheAddresses;
  size_t maxCacheIndex;

  // data structures
  std::vector<MemoryRequest *> activeReadRequests;
  std::vector<MemoryRequest *> activeWriteRequests;

  // signal data structures
  std::queue<std::pair<SignalWindow *, uint64_t>> spmRetryRequests;
  std::vector<bool> finishedPEs;
  Addr signalCurrentFreeSPMAddr;
  // TODO: change this to vector with the necessary changes
  std::queue<SignalWindow *> activeSignalWindows;
  std::map<SignalWindow *, std::vector<MemoryRequest *>>
      activeSignalWindowRequests;

  // timeseries data structures
  std::vector<TimeseriesWindow *> activeTimeseriesWindows;
  std::map<TimeseriesWindow *, std::vector<MemoryRequest *>>
      activeTimeseriesWindowRequests;
  std::map<Addr, std::vector<TimeseriesWindow *>>
      requestPEActiveTimeseriesWindows;

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
  SignalWindow *findCorrespondingSignalWindow(PacketPtr pkt);
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
                       [](auto &w) { return w->isDone(); });
  }
  void printPortRanges();
  Addr getFreeCalcUnitAddr() {
    for (const auto &pair : calcStatus) {
      if (pair.second == false) {
        return pair.first;
      }
    }
    return 0;
  }
  void setCalcUnitStatus(Addr calc_unit_addr, bool busy) {
    for (auto &pair : calcStatus) {
      if (pair.first == calc_unit_addr) {
        calcStatus[pair.first] = busy;
      }
    }
  }
  bool isCacheFull() { return timeseriesCache.size() == maxCacheIndex + 1; }

public:
  WindowManager(const WindowManagerParams &p);
  virtual Tick read(PacketPtr pkt) override;
  virtual Tick write(PacketPtr pkt) override;
  Port &getPort(const std::string &if_name,
                PortID idx = InvalidPortID) override;
  std::string getName() const { return name(); }
};

#endif // __HWACC_WINDOW_MANAGER_HH__