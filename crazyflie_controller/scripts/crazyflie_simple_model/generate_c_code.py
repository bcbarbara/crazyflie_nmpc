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

# set model_name
ra.model_name = model.name

Tf = 1*0.75
N = 50
nx = model.x.size()[0]
nu = model.u.size()[0]
ny = nx + nu
ny_e = nx

# set ocp_nlp_dimensions
nlp_dims     = ra.dims
nlp_dims.nx  = nx
nlp_dims.ny  = ny
nlp_dims.ny_e = ny_e
nlp_dims.nbx = 0
nlp_dims.nbu = nu
nlp_dims.nbx_e = 0
nlp_dims.nu  = model.u.size()[0]
nlp_dims.N   = N

# parameters
g0  = 9.8066    # [m.s^2]
mq  = 33e-3     # [Kg] with one marker

# bounds
max_thrust   = 1.5*mq*g0;
max_droll    = 3.0
max_dpitch   = 3.0
max_dyaw     = 3.0

# set weighting matrices
nlp_cost = ra.cost
Q = np.eye(nx)
Q[0,0] = 30.0     # x
Q[1,1] = 30.0     # y
Q[2,2] = 70.0     # z
Q[3,3] = 15.0     # dx
Q[4,4] = 15.0     # dy
Q[5,5] = 30.0     # dz
Q[6,6] = 1.0e-1     # roll
Q[7,7] = 1.0e-1     # pitch
Q[8,8] = 8.0     # yaw


R = np.eye(nu)
R[0,0] = 40.0    # u1
R[1,1] = 5.0     # droll
R[2,2] = 5.0     # dpitch
R[3,3] = 10.0    # dyaw

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

nlp_cost.Vx = Vx

Vu = np.zeros((ny, nu))
Vu[9,0] = 1.0
Vu[10,1] = 1.0
Vu[11,2] = 1.0
Vu[12,3] = 1.0

nlp_cost.Vu = Vu

nlp_cost.W_e = Q

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

nlp_cost.Vx_e = Vx_e

nlp_cost.yref   = np.array([0, 0, 0.3, 0, 0, 0, 0, 0, 0, mq*g0, 0, 0, 0])
nlp_cost.yref_e = np.array([0, 0, 0.3, 0, 0, 0, 0, 0, 0])

nlp_con = ra.constraints

nlp_con.lbu = np.array([0,-max_droll,-max_dpitch,-max_dyaw])
nlp_con.ubu = np.array([+max_thrust,+max_droll,+max_dpitch,+max_dyaw])
nlp_con.x0  = np.array([0.1,0.1,0.0,0.0,0.0,0.0,0.0,0.0,0.0])
nlp_con.idxbu = np.array([0, 1, 2, 3])

## set QP solver
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

acados_solver = generate_solver(model, ra, json_file = 'acados_ocp.json')

print('>> NMPC exported')

#Nsim = 400

#simX = np.ndarray((Nsim, nx))
#simU = np.ndarray((Nsim, nu))


#for i in range(Nsim):
#    status = acados_solver.solve()

    # get solution
#    x0 = acados_solver.get(0, "x")
#    u0 = acados_solver.get(0, "u")

#    for j in range(nx):
#        simX[i,j] = x0[j]

#    for j in range(nu):
#        simU[i,j] = u0[j]

    # update initial condition
#    x0 = acados_solver.get(1, "x")

#    acados_solver.set(0, "lbx", x0)
#    acados_solver.set(0, "ubx", x0)

# plot results
#import matplotlib.pyplot as plt

#Tsim = 4
#t = np.linspace(0.0, Tsim, Nsim)

#plt.figure(1)
#plt.subplot(4, 1, 1)
#plt.step(t, simU[:, 0], 'r')
#plt.ylabel('u1')
#plt.xlabel('t')
#plt.grid(True)
#plt.subplot(4, 1, 2)
#plt.step(t, simU[:, 1], 'r')
#lt.ylabel('droll')
#plt.xlabel('t')
#plt.grid(True)
#plt.subplot(4, 1, 3)
#plt.step(t, simU[:, 2], 'r')
#plt.ylabel('dpitch')
#plt.xlabel('t')
#plt.grid(True)
#plt.subplot(4, 1, 4)
#plt.step(t, simU[:, 3], 'r')
#plt.ylabel('dyaw')
#plt.xlabel('t')
#plt.grid(True)

#fig, (x, y, z) = plt.subplots(3)
#fig.suptitle('Inertial positions')
#x.plot(t, simX[:, 0], 'r')
#y.plot(t, simX[:, 1], 'r')
#z.plot(t, simX[:, 2], 'r')

#fig, (vix, viy, viz) = plt.subplots(3)
#fig.suptitle('Linear velocities')
#vix.plot(t, simX[:, 3], 'r')
#viy.plot(t, simX[:, 4], 'r')
#viz.plot(t, simX[:, 5], 'r')

#fig, (roll, pitch, yaw) = plt.subplots(3)
#fig.suptitle('Euler angles')
#roll.plot(t, simX[:, 6], 'r')
#pitch.plot(t, simX[:, 7], 'r')
#yaw.plot(t, simX[:, 8], 'r')

#plt.show()
