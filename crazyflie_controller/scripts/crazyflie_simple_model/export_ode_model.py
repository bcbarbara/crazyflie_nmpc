from acados_template import *
def export_ode_model():

    model_name = 'crazyflie_simple_model'

    # parameters
    g0  = 9.8066     # [m.s^2]
    mq  = 33e-3      # [Kg] with one marker

    # states (f_exp)
    xq = SX.sym('xq')
    yq = SX.sym('yq')
    zq = SX.sym('zq')
    dxq = SX.sym('dxq')
    dyq = SX.sym('dyq')
    dzq = SX.sym('dzq')
    roll = SX.sym('roll')
    pitch = SX.sym('pitch')
    yaw = SX.sym('yaw')

    x = vertcat(xq, yq, zq, dxq, dyq, dzq, roll, pitch, yaw)

    # controls
    u1 = SX.sym('u1')
    droll = SX.sym('droll')
    dpitch = SX.sym('dpitch')
    dyaw = SX.sym('dyaw')

    u = vertcat(u1, droll, dpitch, dyaw)

    # for f_impl
    xq_dot = SX.sym('xq_dot')
    yq_dot = SX.sym('yq_dot')
    zq_dot = SX.sym('zq_dot')
    dxq_dot = SX.sym('dxq_dot')
    dyq_dot = SX.sym('dyq_dot')
    dzq_dot = SX.sym('dzq_dot')
    droll_dot = SX.sym('droll_dot')
    dpitch_dot = SX.sym('dpitch_dot')
    dyaw_dot = SX.sym('dyaw_dot')

    xdot = vertcat(xq_dot, yq_dot, zq_dot, dxq_dot, dyq_dot, dzq_dot, droll_dot, dpitch_dot, dyaw_dot)

    # Model equations
    dxq = dxq
    dyq = dyq
    dzq = dzq
    ddxq = (u1*(sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch)))/mq
    ddyq = -(u1*(cos(yaw)*sin(roll)-cos(roll)*sin(pitch)*sin(yaw)))/mq
    ddzq = -(g0*mq - u1*cos(pitch)*cos(roll))/mq
    droll = droll
    dpitch = dpitch
    dyaw = dyaw


    # Explicit and Implicit functions
    f_expl = vertcat(dxq, dyq, dzq, ddxq, ddyq, ddzq, droll, dpitch, dyaw)
    f_impl = xdot - f_expl

    # algebraic variables
    z = []

    # parameters
    p = []

    # dynamics
    model = acados_dae()

    model.f_impl_expr = f_impl
    model.f_expl_expr = f_expl
    model.x = x
    model.xdot = xdot
    model.u = u
    model.z = z
    model.p = p
    model.name = model_name

    return model
