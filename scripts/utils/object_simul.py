import copy
import numpy as np

class SimulObject(object):
    def __init__(self, init_pos, init_rot):
        self._q = np.array([init_pos[0], init_pos[1], np.deg2rad(init_rot)])
        self._v = np.array([0, 0, 0])
    
    @property
    def q(self)->np.array:
        return self._q
    
    @property
    def v(self)->np.array:
        return self._v
    
    @v.setter
    def v(self, v):
        self._v = v
        self._v[2] = np.deg2rad(v[2])

    def apply_v(self, v, sim_step):
        self._v = v
        self._q += v * sim_step

class ObjectDragger(SimulObject):
    def __init__(self, init_pos, init_rot, radius, force):
        super().__init__(init_pos, init_rot)
        self._radius = radius
        self._N = force
    
    @property
    def r(self)->float:
        return self._radius
    
    @property
    def N(self)->float:
        return self._N

    @N.setter
    def N(self, N):
        self._N = N

class ObjectPullee(SimulObject):
    def __init__(self, init_pos, init_rot, width, height):
        super().__init__(init_pos, init_rot)
        self._width = width
        self._height = height
    
    @property
    def width(self)->float:
        return self._width
    
    @property
    def height(self)->float:
        return self._height
    
class ObjectObstacle(object):
    def __init__(self, obstacles_info):
        pass
