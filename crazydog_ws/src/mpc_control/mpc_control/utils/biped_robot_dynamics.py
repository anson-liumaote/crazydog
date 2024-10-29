import numpy as np
import control
import urdf_loader
from LQR_pin import InvertedPendulumLQR



WHEEL_RADIUS = 0.07     # m
WHEEL_MASS = 0.28  # kg
URDF_PATH = "/home/crazydog/crazydog/crazydog_ws/src/mpc_control/mpc_control/robot_models/big bipedal robot v1/urdf/big bipedal robot v1.urdf"
dt = 1/100
pos = np.array([0., 0., 0., 0., 0., 0., 1.,
                0., -1.18, 2.0, 1., 0.,
                0., -1.18, 2.0, 1., 0.])
lqr_controller = InvertedPendulumLQR(pos=pos, 
                                    urdf=URDF_PATH, 
                                    wheel_r=WHEEL_RADIUS, 
                                    M=WHEEL_MASS, 
                                    delta_t=dt, 
                                    show_animation=False)
l = lqr_controller.l_bar

A, B = lqr_controller.get_model_matrix()
C = np.array([[1, 0, 0, 0],
              [0, 0, 1, 0]])

D = np.array([[0],
              [0]])

sys = control.StateSpace(A, B, C, D)
sys_discrete = control.c2d(sys, dt, method='zoh')
A_zoh = np.array(sys_discrete.A)
B_zoh = np.array(sys_discrete.B)