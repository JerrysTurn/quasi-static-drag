import numpy as np
from utils.drag_planner import *

if __name__ == "__main__":
    planner = StableTopContactPushServer()
    waypoint = planner.plan()
    print(waypoint)