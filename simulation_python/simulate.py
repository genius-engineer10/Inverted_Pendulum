import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
import os
import dynamics
import config
import controllers

def run_simulation(case_name='fig3'):
    # Setup
    params = config.PARAM_SETS[case_name]
    sys = dynamics.CartPendulum()
    ctrl = controllers.Controller(sys, 
                                  params['ksu'], params['kcw'], params['kvw'], params['kem'], params['mu'])

    dt = 0.005 
    t_end = params['time_max']
    t_span = (0, t_end)
    
    # Initial State
    state = np.array(config.INITIAL_STATE)
    
    # Data logging
    t_log = []
    x_log = []
    phi_log = []
    u_log = []
    mode_log = []
    phi_dot_log=[]
    # 0: Swing Up, 1: Cruise, 2: Stabilize
    
    current_mode = 0 
    
    def integration_step(t, y):
        nonlocal current_mode
        
        x, phi, x_dot, phi_dot = y
        
        Erp = sys.get_energy(y)
        Eup = sys.get_upright_energy()

        #if our pendulum angle exceeds 2pi then helps to maintain between -pi to pi        
        phi_norm = (phi + np.pi) % (2 * np.pi) - np.pi
        
        if current_mode != 2:
            if abs(phi_norm) < 0.15 and abs(phi_dot) < 1.0: 
                 current_mode = 2
            elif current_mode == 0:
                 # Check transition to cruise
                 # If Energy is close to Eup
                 if Erp > 1.02* Eup:
                     current_mode = 1
        
        # --- Control Computation ---
        u = 0.0
        if current_mode == 0: # Swing Up
            u = ctrl.swing_up(y)
        elif current_mode == 1: # Cruise
            u = ctrl.cruise(y)
        elif current_mode == 2: # Stabilize
            y_lin = y.copy()
            y_lin[1] = phi_norm
            u = ctrl.stabilize(y_lin)

        return u
    t = 0.0
    y = state
    
    while t < t_end:
        u = integration_step(t, y)
        
        # Log
        t_log.append(t)
        x_log.append(y[0])
        phi_log.append(y[1])
        phi_dot_log.append(y[3])
        u_log.append(u)
        mode_log.append(current_mode)
        
        k1 = sys.get_derivatives(y, u)
        
        y1 = y + 0.5 * dt * k1
        k2 = sys.get_derivatives(y1, u)
        
        y2 = y + 0.5 * dt * k2
        k3 = sys.get_derivatives(y2, u)
        
        y3 = y + dt * k3
        k4 = sys.get_derivatives(y3, u)
        
        y = y + (dt/6.0) * (k1 + 2*k2 + 2*k3 + k4)
        t += dt
        
    # Plotting
    plt.figure(figsize=(12, 10))
    
    plt.subplot(4, 1, 1)
    plt.plot(t_log, x_log, label='Cart Position (x)')
    plt.axhline(0.5, color='r', linestyle='--', label='Track Limit')
    plt.axhline(-0.5, color='r', linestyle='--')
    plt.ylabel('x (m)')
    plt.legend()
    plt.title(f'Simulation Results - {case_name}')
    plt.grid(True)
    
    plt.subplot(4, 1, 2)
    plt.plot(t_log, np.deg2rad(np.degrees(phi_log)), label='Pendulum Angle (rad)')
    plt.ylabel('Angle (rad)')
    plt.legend()
    plt.grid(True)

    plt.subplot(4, 1, 4)
    plt.plot(t_log, np.deg2rad(np.degrees(phi_dot_log)), label='Angular Velocity (rad/sec)')
    plt.ylabel('Angular Velocity (rad/sec)')
    plt.legend()
    plt.grid(True)
    
    plt.subplot(4, 1, 3)
    plt.plot(t_log, u_log, label='Control Force (N)')
    plt.ylabel('Force (N)')
    plt.xlabel('Time (s)')
    plt.legend()
    plt.grid(True)
    
    plt.savefig(f'simulation_{case_name}.png')
    print(f"Simulation {case_name} completed. Saved plot.")

if __name__ == "__main__":
    run_simulation('fig3')
    run_simulation('fig4')
    run_simulation('fig5')
