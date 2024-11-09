# Modeling for quasi-static analysis of planar sliding
# Assume Rectangular object with major axis(10.0cm) and minor axis(10.0cm) lie on (0,0) on general coordinate

import numpy as np
import yaml

from numpy.linalg import inv
from scipy.linalg import eigh
from scipy.optimize import brentq
from utils.utils import squareInfo2EqRadius, get_rotation, get_jacobian

class DragServer():
    def __init__(self):
        # initialize constant
        with open('../config/config.yaml', 'r') as f:
            config = yaml.load(f, Loader=yaml.FullLoader)

        self.Ow = config['env']['weight'] * config['env']['gravity']
        self.eq_radius_o = squareInfo2EqRadius(config['pullee']['WIDTH'], config['pullee']['HEIGHT'])

        self.mu1    = config['env']['mu1']
        self.mu2    = config['env']['mu2']
        self.c_o    = config['env']['c_o']
        self.c_p    = config['env']['c_p']
        self.delta  = config['env']['delta']

    def update(self, dragger, pullee):
        self.Hw             = dragger.N
        self.eq_radius_h    = dragger.r

        self.q_o        = pullee.q
        self.q_h        = dragger.q
        self.q_h_dot    = dragger.v
        self.q_rel      = get_rotation(-self.q_o[2]) @ (self.q_h - self.q_o)
        
        self.update_limit_surface_A()
        self.update_limit_surface_B()
        self.G = get_rotation(self.q_rel[2]).T @ get_jacobian(self.q_rel[0], self.q_rel[1])
        self.A_dot = self.G @ self.A @ self.G.T

        # generalized eigenvalue decomposition
        eigen_values, eigen_vectors = eigh(self.B, self.A_dot)
        self.lmda = np.diag(eigen_values)
        self.phi = eigen_vectors
        self.C = self.lmda - np.eye(3)
        
    def update_limit_surface_A(self):
        # define a 3x3 positive definite matrix A
        element = np.array([self.mu1*(self.Ow + self.Hw), self.mu1*(self.Ow + self.Hw), self.eq_radius_o*self.c_o*self.mu1*(self.Ow + self.Hw)])
        A_cop = np.diag(element)
        A_cop = inv(A_cop)**2

        # consider shift of pressure becasuse of patch
        # parameter can be changed to fit the experimental data
        s = 1 - np.power(self.c_p*self.Hw/self.Ow + 1, -self.delta)
        self.A = get_jacobian(-s*self.q_rel[0], -s*self.q_rel[1]) @ A_cop @ get_jacobian(-s*self.q_rel[0], -s*self.q_rel[1]).T
    
    def update_limit_surface_B(self):
        # define a 3x3 positive definite matrix B
        element = np.array([self.mu2 * self.Hw, self.mu2 * self.Hw, self.eq_radius_h * self.c_o * self.mu2 * self.Hw])
        B = np.diag(element)
        self.B = inv(B)**2

    def object_velocity_calculation(self, q_h_dot):
        v_h = get_rotation(self.q_h[2]).T @ q_h_dot
        v_bar_h = self.phi.T @ v_h

        # ========== MODE SELECTION ALGORITHM ==========
        if np.linalg.norm(v_h) == 0:
            # sticking mode
            if np.all((self.lmda - 1) > 0) is True:
                mode = 0
            # slipping mode
            elif np.all((self.lmda - 1) < 0) is True:
                mode = 1
            # pivoting mode
            else:
                mode = 2
        else:
            # sticking mode
            if v_bar_h.T @ self.C @ v_bar_h < 0:
                mode = 0
            # slipping mode
            elif v_bar_h.T @ self.C @ inv(self.lmda)**2 @ v_bar_h >= 0:
                mode = 1
            # pivoting mode
            else:
                mode = 2

        # ========== VELOCITY CALCULATION ==========
        
        # print("mode: ", mode)
        class Equation:
            def __init__(self, v_bar_h, lmda, C):
                self.v_bar_h = v_bar_h
                self.lmda = lmda
                self.C = C

            def equation(self, alpha):
                return (self.C[0,0] * (self.v_bar_h[0] / (alpha * self.lmda[0,0] + 1))**2 + \
                        self.C[1,1] * (self.v_bar_h[1] / (alpha * self.lmda[1,1] + 1))**2 + \
                        self.C[2,2] * (self.v_bar_h[2] / (alpha * self.lmda[2,2] + 1))**2)
            
        # sticking mode
        if mode == 0:
            v_o = inv(self.G) @ v_h
            v_rel = np.array([0.0, 0.0, 0.0]).T

        # slipping mode
        elif mode == 1:
            v_o = np.array([0.0, 0.0, 0.0]).T
            v_rel = v_h

        # pivoting mode
        else:
            eq = Equation(v_bar_h, self.lmda, self.C)

            try:
                alpha = brentq(eq.equation, 0, 100)
                v_o = inv(self.G) @ inv(np.eye(3) + alpha * self.B @ inv(self.A_dot)) @ v_h
            except ValueError as e:
                print("previous velocity will be used") 

        q_o_dot = get_rotation(self.q_o[2]) @ v_o

        return q_o_dot
    
    def sticky_velocity_candidate(self, unit_v):
        # ========== STICKY MODE ==========
        velocity_candidate = np.array([0.0, 0.0, 0.0]).T
        angles = np.arange(0, 2*np.pi, np.pi/10)
        for theta in angles:
            q_h_dot = np.array([unit_v * np.cos(theta), unit_v * np.sin(theta), 0.0]).T
            velocity = self.object_velocity_calculation(q_h_dot)
            velocity = get_rotation(np.radians(90)).T @ velocity
            slip_error = np.linalg.norm(velocity[:2] - q_h_dot[:2])
            if slip_error < 0.02:
                velocity_candidate = np.vstack((velocity_candidate, velocity))

        return velocity_candidate
if __name__ == '__main__':
    drag_server = DragServer()
    drag_server.update()
    
        