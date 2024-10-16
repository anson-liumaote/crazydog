import math
import time
import matplotlib.pyplot as plt
import numpy as np
from numpy.linalg import inv, eig
import pinocchio as pin
import urdf_loader


class InvertedPendulumLQR:
    # def __init__(self, hip, knee, l_bar=3.0, M=0.48, m=2*(0.06801+0.07172)+0.45376, g=9.8, Q=None, R=None, delta_t=1/50, sim_time=15.0, show_animation=True):
    def __init__(self, 
                 l_bar=None, 
                 urdf=None, 
                 pos = None, 
                 wheel_r=None, 
                 M=None, 
                 m=None, 
                 g=9.81, 
                 Q=None, 
                 R=None, 
                 delta_t=None, 
                 sim_time=15.0, 
                 show_animation=False, 
                 dynamic_K=False, 
                 max_l=None, 
                 min_l=None,
                 slice_w=None,
                 advance_lqr_K=None):    
    # transform isaac sim angle to com.py angle
        if l_bar is None:
            print('old loading type')
            robot = urdf_loader.loadRobotModel(urdf_path=urdf)
            robot.pos = pos
            self.com, self.l_bar = robot.calculateCom(plot=False)
            # self.l_bar = 0.20348261632961423
            self.m = robot.calculateMass()  # mass of the pendulum [kg]
        else:
            self.m = m
            self.l_bar = l_bar
        print('lenth:', self.l_bar)
        print('cart mass:', self.m)
        self.M = M  # mass of the cart [kg]self.R = R if R is not None else np.diag([0.1])  # input cost matrix
        self.g = g  # gravity [m/s^2]
        self.nx = 4  # number of states
        self.nu = 1  # number of inputs
        self.wheel_r = wheel_r
        self.Q = Q #if Q is not None else np.diag([0, 1.5, 150.0, 100.0])  # state cost matrix , best in IsaacSim
        self.R = R #if R is not None else np.diag([1e-6])  # input cost matrix
        self.adv_k = advance_lqr_K
        self.delta_t = delta_t  # time tick [s]
        
        if dynamic_K == False:
            self.A, self.B = self.get_model_matrix()
            self.K, _, _ = self.dlqr(self.A, self.B, self.Q, self.R)
            print("Q:", self.Q)
            print("R:", self.R)
            print("K:", self.K)
        else:
            self.K_list = []
            self.max_l = max_l
            self.min_l = min_l
            self.slice_w = slice_w
            self.solve_K()
            self.K = None
            print(self.K_list)

    def solve_K(self):
        for l in np.arange(self.min_l, self.max_l, self.slice_w):
            self.l_bar = l
            A, B = self.get_model_matrix()
            K, _, _ = self.dlqr(A, B, self.Q, self.R)
            self.K_list.append(K)

    def change_K(self, l):
        self.l_bar = l
        index = int((self.l_bar-self.min_l)/self.slice_w)
        self.K = self.K_list[index]
        print('K change to:', self.K)


    def solve_DARE(self, A, B, Q, R, maxiter=150, eps=0.01):
        """
        Solve a discrete time Algebraic Riccati equation (DARE)
        """
        P = Q

        for i in range(maxiter):
            Pn = A.T @ P @ A - A.T @ P @ B @ \
                inv(R + B.T @ P @ B) @ B.T @ P @ A + Q
            if (abs(Pn - P)).max() < eps:
                break
            P = Pn

        return Pn

    def dlqr(self, A, B, Q, R):
        """
        Solve the discrete time lqr controller.
        x[k+1] = A x[k] + B u[k]
        cost = sum x[k].T*Q*x[k] + u[k].T*R*u[k]
        """

        # first, try to solve the ricatti equation
        P = self.solve_DARE(A, B, Q, R)

        # compute the LQR gain
        K = inv(B.T @ P @ B + R) @ (B.T @ P @ A)

        eigVals, eigVecs = eig(A - B @ K)
        return K, P, eigVals

    def lqr_control(self, x, x_desire):
        start = time.time()
        u = -self.K @ (x - x_desire)
        # elapsed_time = time.time() - start
        # print(f"calc time:{elapsed_time:.6f} [sec]")
        return u
    
    def advance_lqr_control(self, x, x_desire):
    
        start = time.time()
        u = -self.adv_k @ (x - x_desire)

        return u

    def get_model_matrix(self):
        # A = np.array([
        #     [0.0, 1.0, 0.0, 0.0],
        #     [0.0, 0.0, self.m * self.g / self.M, 0.0],
        #     [0.0, 0.0, 0.0, 1.0],
        #     [0.0, 0.0, self.g * (self.M + self.m) / (self.l_bar * self.M), 0.0]
        # ])
        # print('A=',A)
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

        # B = np.array([
        #     [0.0],
        #     [1.0 / self.M],
        #     [0.0],
        #     [1.0 / (self.l_bar * self.M)]
        # ])
        # print('B=',B)
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

        return A, B
