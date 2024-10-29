import numpy as np
import cvxpy as cp
# from scipy import sparse
import matplotlib.pyplot as plt
from math import sin, cos
import matplotlib.animation as animation
from crazydog_ws.src.mpc_control.mpc_control.utils.biped_robot_dynamics import l, dt, sys_discrete, A_zoh, B_zoh
import time


def plot_results():
    f = plt.figure()
    ax = f.add_subplot(211)
    plt.plot(cart_pos, label='cart pos')
    plt.plot(cart_vel, label='cart vel')
    plt.plot(pend_ang, label='pend ang')
    plt.plot(pend_ang_vel, label='pend ang vel')
    plt.ylabel(r"$(x_t)_1$", fontsize=16)
    plt.xticks([t_step for t_step in range(nsim) if t_step % 10 == 0])
    plt.legend()
    plt.grid()

    # plt.subplot(4, 1, 3)
    # plt.plot(ctrl_effort)
    # plt.ylabel(r"$(u_t)_1$", fontsize=16)
    # plt.xticks([t_step for t_step in range(nsim) if t_step % 10 == 0])
    # plt.grid()

    plt.tight_layout()
    plt.savefig('mpc_test.png')
    plt.show()



def run_mpc():
    cost = 0.
    constr = [x[:, 0] == x_init]
    for t in range(N):
        cost += cp.quad_form(xr - x[:, t], Q) + cp.quad_form(u[:, t], R)
        constr += [cp.norm(u[:, t], 'inf') <= 5.]
        constr += [x[:, t + 1] == A_zoh @ x[:, t] + B_zoh @ u[:, t]]

    cost += cp.quad_form(x[:, N] - xr, Q)
    problem = cp.Problem(cp.Minimize(cost), constr)
    return problem


[nx, nu] = B_zoh.shape

Q = np.diag([10., 5., 120., 5.])
R = np.array([[.1]])

x0 = np.array([0., 0., 0.3, 0.])  # Initial conditions
xr = np.array([0., 0., 0., 0.])  # Desired states
xr *= -1

N = 10  # MPC Horizon length

x = cp.Variable((nx, N+1))
u = cp.Variable((nu, N))
x_init = cp.Parameter(nx)

if __name__=='__main__':
    nsim = 100  # Number of simulation timesteps
    t = [0.]
    cart_pos = [x0[0]]
    cart_vel = [x0[1]]
    pend_ang = [x0[2]]
    pend_ang_vel = [x0[3]]
    ctrl_effort = [u[:, 0].value]
    t0 = time.time()
    for i in range(1, nsim+1):
        prob = run_mpc()
        x_init.value = x0
        print('TIME: ', round(i*dt, 2), 'STATES: ', [round(state, 2) for state in x0])
        print(u[:, 0].value)
        prob.solve(solver=cp.OSQP, warm_start=True)
        x0 = A_zoh.dot(x0) + B_zoh.dot(u[:, 0].value)
        t.append(i)
        cart_pos.append(x0[0])
        cart_vel.append(x0[1])
        pend_ang.append(x0[2])
        pend_ang_vel.append(x0[3])
        ctrl_effort.append(u[:, 0].value)
        t1 = time.time()
        print('freq:', 1/(t1-t0))
        t0 = t1

    plot_results()
