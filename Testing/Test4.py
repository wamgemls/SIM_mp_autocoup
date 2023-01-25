import matplotlib.pyplot as plt
import numpy as np
import matplotlib

from enum import Enum,auto

class PlannerMode(Enum):
    STANDSTILL = auto()
    COUPLING_PHASE_TILL_PREKINGPIN = auto()
    COUPLING_PHASE_TILL_KINGPIN = auto()


planner_mode = PlannerMode.COUPLING_PHASE_TILL_KINGPIN


if planner_mode is PlannerMode.COUPLING_PHASE_TILL_KINGPIN:
    print("success")


