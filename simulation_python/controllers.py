import numpy as np
import config

class Controller:
    def __init__(self, sys_dyn, ksu, kcw, kvw, kem, mu):
        self.sys = sys_dyn
        self.ksu = ksu
        self.kcw = kcw
        self.kvw = kvw
        self.kem = kem
        self.mu = mu
        
        self.Lqr_K = config.K_PAPER # Default to paper's K, can serve as fallback
        
        self.Eup = self.sys.get_upright_energy() 

    def swing_up(self, state):

        x, phi, x_dot, phi_dot = state
        
        term1 = -self.ksu * np.sign(phi_dot * np.cos(phi))
        
        arg_log = 1.0 - abs(x) / config.L_track
        if arg_log <= 1e-6:
            arg_log = 1e-6
        term2 = self.kcw * np.sign(x) * np.log(arg_log)
        
        x_ddot_cmd = term1 + term2
        
        return self.convert_accel_to_force(state, x_ddot_cmd)

    def cruise(self, state):
        
        x, phi, x_dot, phi_dot = state
        Erp = self.sys.get_energy(state)
        
        #Cart Well
        arg_log_x = 1.0 - abs(x) / config.L_track
        if arg_log_x <= 1e-6: arg_log_x = 1e-6
        u_cw = self.kcw * np.sign(x) * np.log(arg_log_x)
        
        #Velocity Well
        arg_log_v = 1.0 - abs(x_dot) / config.X_DOT_MAX
        if arg_log_v <= 1e-6: arg_log_v = 1e-6
        u_vw = self.kvw * np.sign(x_dot) * np.log(arg_log_v)
        
        #for the sign 
        diff_E = Erp - self.Eup
        
        term_exp = np.exp(np.abs(Erp - self.mu * self.Eup)) - 1.0
        
        #u_energy-maint
        u_em = self.kem * term_exp * np.sign(diff_E) * np.sign(phi_dot * np.cos(phi))
        
        x_ddot_cmd = u_cw + u_vw + u_em
        
        return self.convert_accel_to_force(state, x_ddot_cmd)

    def stabilize(self, state):
        
        # Eq Section 4.3: u = -KX.
        u_force = -np.dot(self.Lqr_K, state)
        return u_force

    def convert_accel_to_force(self, state, x_ddot_des):
        x, phi, x_dot, phi_dot = state
        J_tot = self.sys.J + self.sys.m * self.sys.l**2
        
        Mass_Term = (self.sys.M + self.sys.m) - (self.sys.m**2 * self.sys.l**2 * np.cos(phi)**2) / J_tot
        
        C_G_Terms = (self.sys.m**2 * self.sys.l**2 * self.sys.g * np.sin(phi) * np.cos(phi)) / J_tot \
                    - self.sys.m * self.sys.l * np.sin(phi) * (phi_dot**2)
        
        F = x_ddot_des * Mass_Term + C_G_Terms
        return F

    def get_control(self, t, state, mode_override=None):
        
        pass # To be implemented in simulate 
