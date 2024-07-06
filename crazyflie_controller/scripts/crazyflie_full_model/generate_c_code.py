# The MIT License (MIT)
#
# Copyright 2020 Barbara Barros Carlos, Tommaso Sartor
#
# This file is part of crazyflie_nmpc.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.


from acados_template import AcadosOcp, AcadosOcpSolver
from export_ode_model import *
import numpy as np
import scipy.linalg
from ctypes import *
from os.path import dirname, join, abspath

# ACADOS_PATH = join(dirname(abspath(__file__)), "../../../acados")

# create render arguments
ocp = AcadosOcp()

# export model
model = export_ode_model()

Tf = 0.75
N = 50
nx = model.x.size()[0]
nu = model.u.size()[0]
ny = nx + nu
ny_e = nx

# set ocp_nlp_dimensions
nlp_dims     = ocp.dims
nlp_dims.N   = N

# parameters
g0  = 9.8066    # [m.s^2] accerelation of gravity
mq  = 33e-3     # [kg] total mass (with one marker)
Ct  = 3.25e-4   # [N/krpm^2] Thrust coef

# bounds
hov_w = np.sqrt((mq*g0)/(4*Ct))
max_thrust = 22

# set weighting matrices
nlp_cost = ocp.cost
Q = np.eye(nx)
Q[0,0] = 120.0      # x
Q[1,1] = 100.0      # y
Q[2,2] = 100.0      # z
Q[3,3] = 1.0e-3     # qw
Q[4,4] = 1.0e-3     # qx
Q[5,5] = 1.0e-3     # qy
Q[6,6] = 1.0e-3     # qz
Q[7,7] = 7e-1       # vbx
Q[8,8] = 1.0        # vby
Q[9,9] = 4.0        # vbz
Q[10,10] = 1e-5     # wx
Q[11,11] = 1e-5     # wy
Q[12,12] = 10.0     # wz

R = np.eye(nu)
R[0,0] = 0.06    # w1
R[1,1] = 0.06    # w2
R[2,2] = 0.06    # w3
R[3,3] = 0.06    # w4

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

nlp_cost.W_e = 50*Q

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

nlp_con = ocp.constraints

nlp_con.lbu = np.array([0,0,0,0])
nlp_con.ubu = np.array([+max_thrust,+max_thrust,+max_thrust,+max_thrust])
nlp_con.x0  = np.array([0,0,0,1,0,0,0,0,0,0,0,0,0])
nlp_con.idxbu = np.array([0, 1, 2, 3])

## set QP solver
#ocp.solver_options.qp_solver = 'FULL_CONDENSING_QPOASES'
ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
ocp.solver_options.integrator_type = 'ERK'

# set prediction horizon
ocp.solver_options.tf = Tf
ocp.solver_options.nlp_solver_type = 'SQP_RTI'
#ocp.solver_options.nlp_solver_type = 'SQP'

# set header path
# ocp.acados_include_path  = f'{ACADOS_PATH}/include'
# ocp.acados_lib_path      = f'{ACADOS_PATH}/lib'

ocp.model = model

acados_solver = AcadosOcpSolver(ocp, json_file = 'acados_ocp.json')

print('>> NMPC exported')
