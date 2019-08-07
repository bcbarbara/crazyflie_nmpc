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

Tf = 1
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
g0  = 9.8066            # [m.s^2]
mq  = 33e-3             # [Kg] with one marker
Ct  = 1e+6*3.1582e-10   # [N/Krpm^2]

# bounds
max_lv_body = 1.0
max_av_body = 1.0
box_bounds = 0.5
hov_w = np.sqrt((mq*g0)/(4*Ct))
max_thrust = 22

# set weighting matrices
nlp_cost = ra.cost
Q = np.eye(nx)
Q[0,0] = 5.0    # x
Q[1,1] = 5.0    # y
Q[2,2] = 10.0   # z
Q[3,3] = 1e-1   # q1
Q[4,4] = 1e-1   # q2
Q[5,5] = 1e-1   # q3
Q[6,6] = 1e-1   # q4
Q[7,7] = 1.0    # vbx
Q[8,8] = 1.0    # vby
Q[9,9] = 2.0    # vbz
Q[10,10] = 1.5  # wx
Q[11,11] = 1.5  # wy
Q[12,12] = 5.0  # wz

R = np.eye(nu)
R[0,0] = 2.0    # w1
R[1,1] = 2.0    # w2
R[2,2] = 2.0    # w3
R[3,3] = 2.0    # w4

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
Vx_e[9,9] = 1.0
Vx_e[10,10] = 1.0
Vx_e[11,11] = 1.0
Vx_e[12,12] = 1.0

nlp_cost.Vx_e = Vx_e

nlp_cost.yref   = np.array([0, 0, 0.5, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, hov_w, hov_w, hov_w, hov_w])
nlp_cost.yref_e = np.array([0, 0, 0.5, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0])

nlp_con = ra.constraints
#nlp_con.lbx = np.array([-dxq,-dzq])
#nlp_con.ubx = np.array([+dxq,+dzq])
#nlp_con.idxbx = np.array([4,5])
#nlp_con.lbx_e = np.array([-dxq,-dzq])
#nlp_con.ubx_e= np.array([+dxq,+dzq])
#nlp_con.idxbx_e = np.array([4,5])
#nlp_con.lbx = np.array([-xq,-zq,-pitch,-alpha,-dxq,-dzq,-dpitch,-dalpha])
#nlp_con.ubx = np.array([+xq,+zq,+pitch,+alpha,+dxq,+dzq,+dpitch,+dalpha])
#nlp_con.idxbx = np.array([0,1,2,3,4,5,6,7])
#nlp_con.lbx_e = np.array([-xq,-zq,-pitch,-alpha,-dxq,-dzq,-dpitch,-dalpha])
#nlp_con.ubx_e = np.array([+xq,+zq,+pitch,+alpha,+dxq,+dzq,+dpitch,+dalpha])
#nlp_con.idxbx_e = np.array([0,1,2,3,4,5,6,7])
nlp_con.lbu = np.array([0,0,0,0])
nlp_con.ubu = np.array([+max_thrust,+max_thrust,+max_thrust,+max_thrust])
nlp_con.x0  = np.array([0,0,0,1,0,0,0,0,0,0,0,0,0])
nlp_con.idxbu = np.array([0, 1, 2, 3])

## set QP solver
#ra.solver_config.qp_solver = 'FULL_CONDENSING_QPOASES'
ra.solver_config.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
ra.solver_config.hessian_approx = 'GAUSS_NEWTON'
ra.solver_config.integrator_type = 'ERK'

# set prediction horizon
ra.solver_config.tf = Tf
ra.solver_config.nlp_solver_type = 'SQP_RTI'

# set header path
ra.acados_include_path  = '/usr/local/include'
ra.acados_lib_path      = '/usr/local/lib'

acados_solver = generate_solver(model, ra, json_file = 'acados_ocp.json')

print('>> NMPC exported')

#def quatern2euler(q):
#      R11 = 2*q[0]*q[0]-1+2*q[1]*q[1]
#      R21 = 2*(q[1]*q[2]+q[0]*q[3])
#      R31 = 2*(q[1]*q[3]-q[0]*q[2])
#      R32 = 2*(q[2]*q[3]+q[0]*q[1])
#      R33 = 2*q[0]*q[0]-1+2*q[3]*q[3]

#      phi   = np.arctan2(R32, R33)
#      theta = -np.arctan(R31/np.sqrt(1-R31*R31))
#      psi   = np.arctan2(R21, R11)

#      return phi,theta,psi

#def euler2quatern(e):

#      phi    = e[0]
#      theta  = e[1]
#      psi    = e[2]
#      cosPhi = np.cos(phi*0.5)
#      sinPhi = np.sin(phi*0.5)

#      cosTheta = np.cos(theta*0.5)
#      sinTheta = np.sin(theta*0.5)

#      cosPsi = np.cos(psi*0.5)
#      sinPsi = np.sin(psi*0.5)

      # Convention according the firmware of the crazyflie
#      w = cosPsi*cosTheta*cosPhi + sinPsi*sinTheta*sinPhi
#      x = cosPsi*cosTheta*sinPhi - sinPsi*sinTheta*cosPhi
#      y = cosPsi*sinTheta*cosPhi + sinPsi*cosTheta*sinPhi
#      z = sinPsi*cosTheta*cosPhi - cosPsi*sinTheta*sinPhi

#      return w,x,y,z

#PI = 3.14159

#Nsim = 400

#simX = np.ndarray((Nsim, nx))
#simU = np.ndarray((Nsim, nu))
#angles = np.ndarray((Nsim,3))

#euler = np.array([0,0,0])

#for i in range(Nsim):
#    status = acados_solver.solve()
    # get solution
#    x0 = acados_solver.get(0, "x")
#    u0 = acados_solver.get(0, "u")

    #print(u0)

#    for j in range(nx):
#        simX[i,j] = x0[j]

#    for j in range(nu):
#        simU[i,j] = u0[j]

#    for j in range(3):
#        angles[i,j] = euler[j]

    # update initial condition
#    x0 = acados_solver.get(1, "x")

    #roll,pitch,yaw = quatern2euler(x0[3:7])
    #euler = np.array([roll,pitch,yaw])
    #w,x,y,z = euler2quatern(euler)
    #x0[3] = w
    #x0[4] = x
    #x0[5] = y
    #x0[6] = z
    #print(euler*180/PI)
#    acados_solver.set(0, "lbx", x0)
#    acados_solver.set(0, "ubx", x0)

# plot results
#import matplotlib.pyplot as plt

#Tsim = 4
#t = np.linspace(0.0, Tsim, Nsim)

#plt.figure(1)
#plt.subplot(4, 1, 1)
#plt.step(t, simU[:, 0], 'r')
#plt.ylabel('w1')
#plt.xlabel('t')
#plt.grid(True)
#plt.subplot(4, 1, 2)
#plt.step(t, simU[:, 1], 'r')
#plt.ylabel('w2')
#plt.xlabel('t')
#plt.grid(True)
#plt.subplot(4, 1, 3)
#plt.step(t, simU[:, 2], 'r')
#plt.ylabel('w3')
#plt.xlabel('t')
#plt.grid(True)
#plt.subplot(4, 1, 4)
#plt.step(t, simU[:, 3], 'r')
#plt.ylabel('w4')
#plt.xlabel('t')
#plt.grid(True)

#fig, (x, y, z) = plt.subplots(3)
#fig.suptitle('Inertial positions')
#x.plot(t, simX[:, 0], 'r')
#y.plot(t, simX[:, 1], 'r')
#z.plot(t, simX[:, 2], 'r')

#fig, (q1, q2, q3, q4) = plt.subplots(4)
#fig.suptitle('Quaternion')
#q1.plot(t, simX[:, 3], 'r')
#q2.plot(t, simX[:, 4], 'r')
#q3.plot(t, simX[:, 5], 'r')
#q4.plot(t, simX[:, 6], 'r')

#fig, (r, p, y) = plt.subplots(3)
#fig.suptitle('Euler angles')
#r.plot(t, angles[:, 0], 'r')
#p.plot(t, angles[:, 1], 'r')
#y.plot(t, angles[:, 2], 'r')

#fig, (h, v, w) = plt.subplots(3)
#fig.suptitle('Body-frame linear velocities')
#h.plot(t, simX[:, 7], 'r')
#v.plot(t, simX[:, 8], 'r')
#w.plot(t, simX[:, 9], 'r')

#fig, (wx, wy, wz) = plt.subplots(3)
#fig.suptitle('Body-frame angular rates')
#wx.plot(t, simX[:, 10], 'r')
#wy.plot(t, simX[:, 11], 'r')
#wz.plot(t, simX[:, 12], 'r')


#plt.show()
