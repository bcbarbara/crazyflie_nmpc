from acados_template import *
import acados_template as at
from export_ode_model import *
import numpy as np
import scipy.linalg
from ctypes import *

# create render arguments
ra = acados_ocp_nlp()

# export model
model = export_ode_model()

# export nonlinear path constraint
path_constraint = export_path_constraint()

# set model_name
ra.model = model

# set nonlinear path constraint name
ra.con_h = path_constraint

Tf = 0.75
N = 50
nx = model.x.size()[0]
nu = model.u.size()[0]
ny = nx + nu
ny_e = nx

# set ocp_nlp_dimensions
nlp_dims        = ra.dims
nlp_dims.nx     = nx
nlp_dims.ny     = ny
nlp_dims.ny_e   = ny_e
nlp_dims.nbx    = 0
nlp_dims.nbu    = nu
nlp_dims.nbx_e  = 0
nlp_dims.nu     = model.u.size()[0]
nlp_dims.nh     = path_constraint.nc
#nlp_dims.ns     = path_constraint.nc
#nlp_dims.nsh    = path_constraint.nc
nlp_dims.N      = N

# parameters
g0  = 9.8066    # [m.s^2]
mq  = 33e-3     # [kg] with one marker
Ct  = 3.25e-4   # [N/krpm^2]

# bounds
hov_w = np.sqrt((mq*g0)/(4*Ct))
max_thrust = 22

dmin = 0.2 # safe ditance [m]

# set weighting matrices
nlp_cost = ra.cost
Q = np.eye(nx)
Q[0,0] = 120.0      # x
Q[1,1] = 100.0      # y
Q[2,2] = 125.0      # z
Q[3,3] = 1.0e-3     # q1
Q[4,4] = 1.0e-3     # q2
Q[5,5] = 1.0e-3     # q3
Q[6,6] = 1.0e-3     # q4
Q[7,7] = 1.0        # vbx
Q[8,8] = 3.0        # vby
Q[9,9] = 15.0       # vbz
Q[10,10] = 1e-5     # wx
Q[11,11] = 1e-5     # wy
Q[12,12] = 10.0     # wz

R = np.eye(nu)
R[0,0] = 0.2    # w1
R[1,1] = 0.2    # w2
R[2,2] = 0.2    # w3
R[3,3] = 0.2    # w4
R[4,4] = 1e+3   # slack

nlp_cost.W = scipy.linalg.block_diag(Q, R)

Vx = np.zeros((ny, nx))
Vx[0,0] = 1.0
Vx[1,1] = 1.0
Vx[2,2] = 1.0
Vx[3,3] = 1.0
Vx[4,4] = 1.0
Vx[5,5] = 1.0
Vx[6,6] = 1.0
Vx[7,7] = 1.0
Vx[8,8] = 1.0
Vx[9,9] = 1.0
Vx[10,10] = 1.0
Vx[11,11] = 1.0
Vx[12,12] = 1.0
nlp_cost.Vx = Vx

Vu = np.zeros((ny, nu))
Vu[13,0] = 1.0
Vu[14,1] = 1.0
Vu[15,2] = 1.0
Vu[16,3] = 1.0
Vu[17,4] = 1.0
nlp_cost.Vu = Vu

nlp_cost.W_e = 10*Q

Vx_e = np.zeros((ny_e, nx))
Vx_e[0,0] = 1.0
Vx_e[1,1] = 1.0
Vx_e[2,2] = 1.0
Vx_e[3,3] = 1.0
Vx_e[4,4] = 1.0
Vx_e[5,5] = 1.0
Vx_e[6,6] = 1.0
Vx_e[7,7] = 1.0
Vx_e[8,8] = 1.0
Vx_e[9,9] = 1.0
Vx_e[10,10] = 1.0
Vx_e[11,11] = 1.0
Vx_e[12,12] = 1.0

nlp_cost.Vx_e = Vx_e

# seting references (for regulation)
nlp_cost.yref   = np.array([0.0, 0.0, 0.4, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, hov_w, hov_w, hov_w, hov_w, 0.0])
nlp_cost.yref_e = np.array([0.0, 0.0, 0.4, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0])

# setting bounds
nlp_con = ra.constraints
# for the control inputs
nlp_con.lbu = np.array([0,0,0,0,0]) # lower bound
nlp_con.ubu = np.array([+max_thrust,+max_thrust,+max_thrust,+max_thrust,100000]) # upper bound
nlp_con.idxbu = np.array([0, 1, 2, 3, 4]) # indexes of bounds
nlp_con.lh = dmin*np.array([1])
nlp_con.uh = 10000*np.array([1])

# initial state
nlp_con.x0  = np.array([0.0,0.0,0.0,1,0,0,0,0,0,0,0,0,0])

# set QP solver
#ra.solver_config.qp_solver = 'FULL_CONDENSING_QPOASES'
ra.solver_config.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
ra.solver_config.hessian_approx = 'GAUSS_NEWTON'
ra.solver_config.integrator_type = 'ERK'

# set prediction horizon
ra.solver_config.tf = Tf
ra.solver_config.nlp_solver_type = 'SQP_RTI'
#ra.solver_config.nlp_solver_type = 'SQP'

# set header path
ra.acados_include_path  = '/usr/local/include'
ra.acados_lib_path      = '/usr/local/lib'

acados_solver = generate_solver(ra, json_file = 'acados_ocp.json')

print('>> NMPC exported')

Nsim = 300

simX = np.ndarray((Nsim, nx))
simU = np.ndarray((Nsim, nu))

for i in range(Nsim):
    status = acados_solver.solve()
    # get solution
    x0 = acados_solver.get(0, "x")
    u0 = acados_solver.get(0, "u")

    for j in range(nx):
        simX[i,j] = x0[j]

    for j in range(nu):
        simU[i,j] = u0[j]

    # update initial condition
    x0 = acados_solver.get(1, "x")

    acados_solver.set(0, "lbx", x0)
    acados_solver.set(0, "ubx", x0)

# # plot results
# import matplotlib.pyplot as plt
#
# Tsim = 3
# t = np.linspace(0.0, Tsim, Nsim)
#
# plt.figure(1)
# plt.subplot(5, 1, 1)
# plt.step(t, simU[:, 0], 'r')
# plt.ylabel('w1')
# plt.xlabel('t')
# plt.grid(True)
# plt.subplot(5, 1, 2)
# plt.step(t, simU[:, 1], 'r')
# plt.ylabel('w2')
# plt.xlabel('t')
# plt.grid(True)
# plt.subplot(5, 1, 3)
# plt.step(t, simU[:, 2], 'r')
# plt.ylabel('w3')
# plt.xlabel('t')
# plt.grid(True)
# plt.subplot(5, 1, 4)
# plt.step(t, simU[:, 3], 'r')
# plt.ylabel('w4')
# plt.xlabel('t')
# plt.subplot(5, 1, 5)
# plt.step(t, simU[:, 4], 'r')
# plt.ylabel('s')
# plt.xlabel('t')
# plt.grid(True)
#
# fig, (x, y, z) = plt.subplots(3)
# fig.suptitle('Inertial positions')
# x.plot(t, simX[:, 0], 'r')
# y.plot(t, simX[:, 1], 'r')
# z.plot(t, simX[:, 2], 'r')
#
# fig, (q1, q2, q3, q4) = plt.subplots(4)
# fig.suptitle('Quaternion')
# q1.plot(t, simX[:, 3], 'r')
# q2.plot(t, simX[:, 4], 'r')
# q3.plot(t, simX[:, 5], 'r')
# q4.plot(t, simX[:, 6], 'r')
#
# fig, (h, v, w) = plt.subplots(3)
# fig.suptitle('Body-frame linear velocities')
# h.plot(t, simX[:, 7], 'r')
# v.plot(t, simX[:, 8], 'r')
# w.plot(t, simX[:, 9], 'r')
#
# fig, (wx, wy, wz) = plt.subplots(3)
# fig.suptitle('Body-frame angular rates')
# wx.plot(t, simX[:, 10], 'r')
# wy.plot(t, simX[:, 11], 'r')
# wz.plot(t, simX[:, 12], 'r')
#
#
# plt.show()
