from m5.params import *
from m5.proxy import *
from m5.objects.Device import BasicPioDevice


class StateStore(BasicPioDevice):
    type = "StateStore"
    cxx_header = "hwacc/state_store.hh"

    system = Param.System(Parent.any, "Parent system")
    debug_enabled = Param.Bool(
        False, "Whether or not this device will display debug messages"
    )

    devicename = Param.String("state_store", "Device name")
    clock_period = Param.Int(10, "Clock period in ns")
    pio_size = Param.Addr(
        0x8,
        "Size of MMRs. Should be large enough to support flags, config, and global var addresses",
    )

    spm = VectorRequestPort("Request ports connected to private scratchpad memory")
    req_stream_ports = VectorRequestPort(
        "Request ports connected to processing element queues"
    )
    resp_stream_ports = VectorRequestPort(
        "Response ports connected to processing element queues"
    )
    local = VectorRequestPort("Request ports connected to local cluster xbar")
    acp = VectorRequestPort(
        "Request ports connected to coherency xbar (for memory accesses)"
    )
