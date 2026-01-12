import numpy as np

# System Parameters 
M = 2.4       
m = 0.23      
two_l = 0.36  
l = two_l / 2 
g = 9.81      

J_com = (1.0 / 12.0) * m * (two_l**2) 

J_H = J_com + m * (l**2)

# Cart track limits
L_track = 0.5 

PARAM_SETS = {
    'fig3': {'ksu': 1.64, 'kcw': 2.25, 'kvw': 5.0, 'kem': 6.0, 'mu': 1.05, 'swings': 3, 'time_max': 20},
    'fig4': {'ksu': 1.30, 'kcw': 2.25, 'kvw': 5.0, 'kem': 6.0, 'mu': 1.05, 'swings': 4, 'time_max': 12},
    'fig5': {'ksu': 0.87, 'kcw': 2.25, 'kvw': 5.0, 'kem': 6.0, 'mu': 1.05, 'swings': 6, 'time_max': 15},
}

# LQR Tuning Matrices (Section 4.3)
Q = np.diag([100, 100, 50, 1])
R = np.array([[5]])

# Initial Condition
INITIAL_STATE = [0.0, np.pi, 0.0, 0.0] 

X_DOT_MAX = 5.0 #i tuned this 5 based on hit and trial here

K_PAPER = np.array([-4.4721, -75.9573, -7.1427, -11.5948])
