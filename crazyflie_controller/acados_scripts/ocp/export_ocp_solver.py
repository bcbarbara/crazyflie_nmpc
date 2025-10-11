# MIT License

# Copyright (c) 2025 Barbara Barros Carlos, Tommaso Sartor

# This file is part of the crazyflie_nmpc project.

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the “Software”), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

from acados_template import AcadosOcp, AcadosOcpSolver
from casadi import vertcat, diagcat
import numpy as np


def export_ocp_solver(model, ocp_config):
    """Build and exports the OCP solver for the Crazyflie NMPC."""

    # Create OCP object
    ocp = AcadosOcp()

    # Set model
    ocp.model = model

    # Get model dimensions
    nx = model.x.rows()
    nu = model.u.rows()

    # Short alieses for OCP settings
    N, tf = (ocp_config.N, ocp_config.tf)

    # Set prediction horizon
    ocp.solver_options.N_horizon = N
    ocp.solver_options.tf = tf

    # Cost matrices
    Q = np.diag([120.0, 100.0, 100.0, 1.0e-3, 1.0e-3, 1.0e-3, 1.0e-3, 7.0e-1, 1.0, 4.0, 1.0e-5, 1.0e-5, 10.0])
    R = np.diag([0.06, 0.06, 0.06, 0.06])

    # Set tage cost
    ocp.cost.cost_type = 'NONLINEAR_LS'
    ocp.model.cost_y_expr = vertcat(model.x, model.u)
    ocp.cost.yref = np.zeros((nx + nu,)) # (it will be changed in the closed-loop)
    ocp.cost.W = diagcat(Q, R).full()

    # Set terminal cost
    ocp.cost.cost_type_e = 'NONLINEAR_LS'
    ocp.cost.yref_e = np.zeros((nx,)) # (it will be changed in the closed-loop)
    ocp.model.cost_y_expr_e = model.x
    ocp.cost.W_e = 50 * Q

    # Set constraints 
    # Box constraints on the inputs
    ocp.constraints.lbu = np.array([0.0, 0.0, 0.0, 0.0])
    ocp.constraints.ubu = np.array([22.0, 22.0, 22.0, 22.0])
    ocp.constraints.idxbu = np.arange(4)
    # Initial condition (it will be changed in the closed-loop)
    ocp.constraints.x0 = np.zeros((nx,))

    # Set solver options
    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    ocp.solver_options.qp_solver_cond_N = int(N / 7)
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
    ocp.solver_options.nlp_solver_type = 'SQP_RTI'
    ocp.solver_options.integrator_type = 'ERK'
    ocp.solver_options.sim_method_num_stages = 4
    ocp.solver_options.sim_method_num_steps = 2

    # Create solver
    ocp_solver = AcadosOcpSolver(ocp)
    print("OCP solver successfully exported for model: {}".format(model.name))

    return ocp_solver
