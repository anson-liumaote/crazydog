import math
import time
import matplotlib.pyplot as plt
import numpy as np
from numpy.linalg import inv, eig
import pinocchio as pin
import urdf_loader
import osqp
import numpy as np
import scipy as sp
from scipy import sparse

from crazydog_ws.src.mpc_control.mpc_control.utils.biped_robot_dynamics import l, dt, sys_discrete, A_zoh, B_zoh


class InvertedPendulumMPC:
    # def __init__(self, hip, knee, l_bar=3.0, M=0.48, m=2*(0.06801+0.07172)+0.45376, g=9.8, Q=None, R=None, delta_t=1/50, sim_time=15.0, show_animation=True):
    def __init__(self, urdf=None, pos = None, wheel_r=None, M=None, m=None, g=9.81, Q=None, R=None, delta_t=None, sim_time=15.0, show_animation=True):    
    # transform isaac sim angle to com.py angle
        robot = urdf_loader.loadRobotModel(urdf_path=urdf)
        robot.pos = pos
        self.com, self.l_bar = robot.calculateCom(plot=False)
        print('lenth:', self.l_bar)
        self.M = M  # mass of the cart [kg]self.R = R if R is not None else np.diag([0.1])  # input cost matrix
        self.m = robot.calculateMass()  # mass of the pendulum [kg]
        print('cart mass:', self.m)
        self.g = g  # gravity [m/s^2]
        self.nx = 4  # number of states
        self.nu = 1  # number of inputs
        self.wheel_r = wheel_r
        self.Q = Q #if Q is not None else np.diag([0, 1.5, 150.0, 100.0])  # state cost matrix , best in IsaacSim
        self.R = R #if R is not None else np.diag([1e-6])  # input cost matrix
        self.N = 10

        self.delta_t = delta_t  # time tick [s]
        self.sim_time = sim_time  # simulation time [s]

        self.show_animation = show_animation

        self.Ad, self.Bd = self.get_model_matrix()
        self.setup_mpc(self.Ad, self.Bd)
        # self.K, _, _ = self.dlqr(self.A, self.B, self.Q, self.R)
        print("Q:", self.Q)
        print("R:", self.R)


    def setup_mpc(self, Ad, Bd):
        nx, nu = self.nx, self.nu

        # Constraints
        u0 = 0
        umin = np.array([-1.5]) - u0
        umax = np.array([1.5]) - u0
        xmin = np.array([-np.inf,-np.inf,-np.inf,-np.inf])
        xmax = np.array([np.inf, np.inf, np.inf, np.inf])

        # Objective function
        Q = sparse.diags(self.Q)
        QN = Q
        R = sparse.diags(self.R)

        # Initial and reference states
        x0 = np.array([0., 0., 0., 0.])
        xr = np.array([0., 0., 0.07, 0.])

        # Prediction horizon
        N = self.N

        # Cast MPC problem to a QP: x = (x(0),x(1),...,x(N),u(0),...,u(N-1))
        # - quadratic objective
        P = sparse.block_diag([sparse.kron(sparse.eye(N), Q), QN,
                            sparse.kron(sparse.eye(N), R)], format='csc')
        # - linear objective
        q = np.hstack([np.kron(np.ones(N), -Q@xr), -QN@xr, np.zeros(N*nu)])
        # - linear dynamics
        Ax = sparse.kron(sparse.eye(N+1),-sparse.eye(nx)) + sparse.kron(sparse.eye(N+1, k=-1), Ad)
        Bu = sparse.kron(sparse.vstack([sparse.csc_matrix((1, N)), sparse.eye(N)]), Bd)
        Aeq = sparse.hstack([Ax, Bu])
        leq = np.hstack([-x0, np.zeros(N*nx)])
        ueq = leq
        # - input and state constraints
        Aineq = sparse.eye((N+1)*nx + N*nu)
        lineq = np.hstack([np.kron(np.ones(N+1), xmin), np.kron(np.ones(N), umin)])
        uineq = np.hstack([np.kron(np.ones(N+1), xmax), np.kron(np.ones(N), umax)])
        # - OSQP constraints
        A = sparse.vstack([Aeq, Aineq], format='csc')
        self.l = np.hstack([leq, lineq])
        self.u = np.hstack([ueq, uineq])

        # Create an OSQP object
        self.prob = osqp.OSQP()

        # Setup workspace
        self.prob.setup(P, q, A, self.l, self.u)


    def get_model_matrix(self):
        Jz = (1/3) * self.m * self.l_bar**2
        I = (1/2) * self.M * self.wheel_r**2
        Q_eq = Jz * self.m + (Jz + self.m * self.l_bar * self.l_bar) * \
            (2 * self.M + (2 * I) / (self.wheel_r**2))
        A_23 = -(self.m**2)*(self.l_bar**2)*self.g / Q_eq
        A_43 = self.m*self.l_bar*self.g * \
            (self.m+2*self.M+(2*I/(self.wheel_r**2)))/Q_eq
        A = np.array([
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, A_23, 0.0],
            [0.0, 0.0, 0.0, 1.0],
            [0.0, 0.0, A_43, 0.0]
        ])

        A = np.eye(self.nx) + self.delta_t * A

        B_21 = (Jz+self.m*self.l_bar**2+self.m *
                self.l_bar*self.wheel_r)/Q_eq/self.wheel_r
        B_41 = -((self.m*self.l_bar/self.wheel_r)+self.m +
                 2*self.M+(2*I/(self.wheel_r**2)))/Q_eq
        B = np.array([
            [0.0],
            [2*B_21],
            [0.0],
            [2*B_41]
        ])
        B = self.delta_t * B

        Ad = sparse.csc_matrix(A_zoh.tolist())
        Bd = sparse.csc_matrix(B_zoh.tolist())

        return Ad, Bd

    def update(self, x):
        
        x0 = x.flatten()

        # Update initial state
        self.l[:self.nx] = -x0
        self.u[:self.nx] = -x0
        self.prob.update(l=self.l, u=self.u)
    
    def solve(self):
        # Solve
        res = self.prob.solve()

        # Check solver status
        if res.info.status != 'solved':
            raise ValueError('OSQP did not solve the problem!')

        # Apply first control input to the plant
        ctrl = res.x[-self.N*self.nu:-(self.N-1)*self.nu]

        u = np.array([[ctrl[0]]])

        return u

if __name__ == '__main__':
    WHEEL_RADIUS = 0.08     # m
    WHEEL_MASS = 0.695  # kg
    URDF_PATH = "/home/crazydog/crazydog/crazydog_ws/src/mpc_control/mpc_control/robot_models/big bipedal robot v1/urdf/big bipedal robot v1.urdf"
    Q = [1e-9, 1e-5, 0.01, 1e-6]       # 1e-9, 1e-9, 0.01, 1e-6
    R = [1e-6]
    q = np.array([0., 0., 0., 0., 0., 0., 1.,
                        0., -1.18, 2.0, 1., 0.,
                        0., -1.18, 2.0, 1., 0.])
    mpc_controller = InvertedPendulumMPC(pos=q, 
                                        urdf=URDF_PATH, 
                                        wheel_r=WHEEL_RADIUS, 
                                        M=WHEEL_MASS, Q=Q, R=R, 
                                        delta_t=1/500, 
                                        show_animation=False)