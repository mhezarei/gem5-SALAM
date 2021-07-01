from m5.params import *
from m5.proxy import *
from m5.SimObject import SimObject
from CommInterface import CommInterface
from CycleCounts import CycleCounts
#from HWInterface import HWInterface

class ComputeUnit(SimObject):
    type = 'ComputeUnit'
    cxx_header = "hwacc/compute_unit.hh"

    comm_int = Param.CommInterface(Parent.any, "Communication interface to connect to")
    cycles = Param.CycleCounts(Parent.any, "Load instruction runtime cycles")

    #comm_int = Param.CommInterface(Parent.any, "Communication interface to connect to")
    #hw_int = Param.HWInterface(Parent.any, "Hardware interface to connect to")