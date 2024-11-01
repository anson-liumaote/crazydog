import math
import time
import matplotlib.pyplot as plt
import numpy as np
from numpy.linalg import inv, eig
import pinocchio as pin
import urdf_loader

class InvertedPendulumLQR:
    # def __init__(self, hip, knee, l_bar=3.0, M=0.48, m=2*(0.06801+0.07172)+0.45376, g=9.8, Q=None, R=None, delta_t=1/50, sim_time=15.0, show_animation=True):
    def __init__(self, l_bar=None, urdf=None, pos = None, wheel_r=None, M=None, m=None, g=9.81, Q=None, R=None, delta_t=None, sim_time=15.0, show_animation=False):    
    # transform isaac sim angle to com.py angle
        if l_bar is None:
            print('old loading type')
            robot = urdf_loader.loadRobotModel(urdf_path=urdf)
            robot.pos = pos
            self.com, self.l_bar = robot.calculateCom(plot=False)
            # self.l_bar = 0.20348261632961423
            self.M = M  # mass of the cart [kg]self.R = R if R is not None else np.diag([0.1])  # input cost matrix
            self.m = robot.calculateMass()  # mass of the pendulum [kg]
        else:
            self.m = m
            self.l_bar = l_bar
        print('lenth:', self.l_bar)
        print('cart mass:', self.m)
        self.g = g  # gravity [m/s^2]
        self.nx = 4  # number of states
        self.nu = 1  # number of inputs
        self.wheel_r = wheel_r
        self.Q = Q #if Q is not None else np.diag([0, 1.5, 150.0, 100.0])  # state cost matrix , best in IsaacSim
        self.R = R #if R is not None else np.diag([1e-6])  # input cost matrix

        # self.Q = Q if Q is not None else np.diag([0.1, 0.001, 30.0, 0.0])  # state cost matrix
        # self.R = R if R is not None else np.diag([0.001])  # input cost matrix

        self.delta_t = delta_t  # time tick [s]
        self.sim_time = sim_time  # simulation time [s]

        self.show_animation = show_animation

        self.A, self.B = self.get_model_matrix()
        self.K, _, _ = self.dlqr(self.A, self.B, self.Q, self.R)
        print("Q:", self.Q)
        print("R:", self.R)
        print("K:", self.K)

    def main(self):
        x0 = np.array([
            [0.0],
            [0.0],
            [math.radians(10)],
            [0.0]
        ])

        X = np.copy(x0)
        time_elapsed = 0.0

        time_list = []
        x_list = []
        u_list = []
        theta_list = []
        while self.sim_time > time_elapsed:
        # while True:
            time_elapsed += self.delta_t
            
            # calculate control input
            u = self.lqr_control(X)
            # print(u,'N')
            # print(x[0],'\n')
            # simulate inverted pendulum cart
            # if u[0,0] > 0.142:
            #         u[0, 0] = 0.142
            # elif u[0, 0] < -0.142:
            #     u[0, 0] = -0.142
            X = self.simulation(X, u)

            # time_list.append(time_elapsed)
            time_list.append(time_elapsed)
            # u_list.append(u[0, 0])
            x_list.append(X[0, 0])
            theta_list.append(math.degrees(X[2, 0]))
            u_list.append(u[0, 0])
            # x_list.append(X[0, 0])
            # theta_list.append(math.degrees(X[2, 0]))


            # if self.show_animation:
            #     plt.clf()
            #     px = float(x[0, 0])
            #     theta = float(x[2, 0])
            #     self.plot_cart(px, theta)
            #     plt.xlim([-5.0, 2.0])
            #     plt.pause(0.001)

        print("Finish")
        print(f"x={float(X[0, 0]):.2f} [m] , theta={math.degrees(X[2, 0]):.2f} [deg]")
        if self.show_animation:
            plt.subplot(311)
            plt.plot(time_list, x_list, label = f"Q[theta]={Q[2,2]:.1f}")
            plt.subplot(312)
            plt.plot(time_list, theta_list, label = f"Q[theta]={Q[2,2]:.1f}")
            plt.subplot(313)
            plt.plot(time_list, u_list, label = f"Q[theta]={Q[2,2]:.1f}")


    def simulation(self, x, u):
        X_dot = self.A @ x + self.B @ u

        return X_dot

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

    def plot_cart(self, xt, theta):
        cart_w = 1.0
        cart_h = 0.5
        radius = 0.1

        cx = np.array([-cart_w / 2.0, cart_w / 2.0, cart_w /
                       2.0, -cart_w / 2.0, -cart_w / 2.0])
        cy = np.array([0.0, 0.0, cart_h, cart_h, 0.0])
        cy += radius * 2.0

        cx = cx + xt

        bx = np.array([0.0, self.l_bar * math.sin(-theta)])
        bx += xt
        by = np.array([cart_h, self.l_bar * math.cos(-theta) + cart_h])
        by += radius * 2.0

        angles = np.arange(0.0, math.pi * 2.0, math.radians(3.0))
        ox = np.array([radius * math.cos(a) for a in angles])
        oy = np.array([radius * math.sin(a) for a in angles])

        rwx = np.copy(ox) + cart_w / 4.0 + xt
        rwy = np.copy(oy) + radius
        lwx = np.copy(ox) - cart_w / 4.0 + xt
        lwy = np.copy(oy) + radius

        wx = np.copy(ox) + bx[-1]
        wy = np.copy(oy) + by[-1]

        plt.plot(cx.flatten(), cy.flatten(), "-b")
        plt.plot(bx.flatten(), by.flatten(), "-k")
        plt.plot(rwx.flatten(), rwy.flatten(), "-k")
        plt.plot(lwx.flatten(), lwy.flatten(), "-k")
        plt.plot(wx.flatten(), wy.flatten(), "-k")
        plt.title(f"x: {xt:.2f} , theta: {math.degrees(theta):.2f}")

        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])

        plt.axis("equal")

if __name__ == '__main__':
    
    # for i in range(1, 11):
    Q = np.diag([0.0, 1.0, 150.0, 100.0])
    R = np.diag([1.0]) 
    lqr = InvertedPendulumLQR(72.42, 125, Q = Q, R=R)
    lqr.main()
    plt.suptitle('Q = [0.001, 1.0, theta, 0.001], R=[1.0]')
    plt.subplot(311)
    plt.xlabel('time')
    plt.ylabel('x')
    plt.legend()
    plt.grid(True)
    plt.subplot(312)
    plt.xlabel('time')
    plt.ylabel('theta')
    plt.legend()
    plt.grid(True)
    plt.subplot(313)
    plt.xlabel('time')
    plt.ylabel('u')
    plt.legend()
    plt.grid(True)
    plt.show()
    
