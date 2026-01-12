import numpy as np
import config

class CartPendulum:
    def __init__(self):
        self.M = config.M
        self.m = config.m
        self.l = config.l
        self.g = config.g
        self.J = config.J_com
        
        
    def get_derivatives(self, state, u):
        
        x = state[0]
        phi = state[1]
        x_dot = state[2]
        phi_dot = state[3]
        
        sin_phi = np.sin(phi)
        cos_phi = np.cos(phi)
        
        J_tot = self.J + self.m * self.l**2
        
        # Mass matrix determinant-like term
        denom = (self.M + self.m) - (self.m**2 * self.l**2 * cos_phi**2) / J_tot
        
        # Numerator for x_ddot
        num_x = u + self.m * self.l * (phi_dot**2) * sin_phi - (self.m**2 * self.l**2 * self.g * sin_phi * cos_phi) / J_tot
        
        x_ddot = num_x / denom
        
        # Back substitute for phi_ddot
        phi_ddot = (self.m * self.g * self.l * sin_phi - self.m * self.l * x_ddot * cos_phi) / J_tot
        
        return np.array([x_dot, phi_dot, x_ddot, phi_ddot])

    def get_energy(self, state):
        
        phi = state[1]
        phi_dot = state[3]
        J_H = self.J + self.m * self.l**2
        
        Erp = 0.5 * J_H * (phi_dot**2) + self.m * self.g * self.l * np.cos(phi)
        return Erp

    def get_upright_energy(self):
        
        return self.m * self.g * self.l
