from acados_template import *
def export_ode_model():

    model_name = 'crazyflie'

    # parameters
    g0  = 9.8         # [m.s^2]
    mq  = 33e-3      # [Kg] with one marker
    Ixx = 1.395e-5   # [Kg.m^2]
    Iyy = 1.395e-5   # [Kg.m^2]
    Izz = 2.173e-5   # [Kg.m^2]
    Cd  = 7.9379e-06 # [N/Krpm^2]
    Ct  = 3.25e-4    # [N/rpm^2]
    dq  = 65e-3      # [m] distance between motors center
    l   = dq/2       # [m] distance between motor center and the axis of rotation

    # states (f_exp)
    xq = SX.sym('xq')
    yq = SX.sym('yq')
    zq = SX.sym('zq')
    q1 = SX.sym('q1')
    q2 = SX.sym('q2')
    q3 = SX.sym('q3')
    q4 = SX.sym('q4')
    vbx = SX.sym('vbx')
    vby = SX.sym('vby')
    vbz = SX.sym('vbz')
    wx = SX.sym('wx')
    wy = SX.sym('wy')
    wz = SX.sym('wz')
    x = vertcat(xq, yq, zq, q1, q2, q3, q4, vbx, vby, vbz, wx, wy, wz)

    # controls
    w1 = SX.sym('w1')
    w2 = SX.sym('w2')
    w3 = SX.sym('w3')
    w4 = SX.sym('w4')

    # slack
    s  = SX.sym('s')
    u  = vertcat(w1, w2, w3, w4, s) # motor torques

    # for f_impl
    xq_dot = SX.sym('xq_dot')
    yq_dot = SX.sym('yq_dot')
    zq_dot = SX.sym('zq_dot')
    q1_dot = SX.sym('q1_dot')
    q2_dot = SX.sym('q2_dot')
    q3_dot = SX.sym('q3_dot')
    q4_dot = SX.sym('q4_dot')
    vbx_dot = SX.sym('vbx_dot')
    vby_dot = SX.sym('vby_dot')
    vbz_dot = SX.sym('vbz_dot')
    wx_dot = SX.sym('wx_dot')
    wy_dot = SX.sym('wy_dot')
    wz_dot = SX.sym('wz_dot')
    xdot = vertcat(xq_dot, yq_dot, zq_dot, q1_dot, q2_dot, q3_dot, q4_dot, vbx_dot, vby_dot, vbx_dot, wx_dot, wy_dot, wz_dot)

    # Model equations
    dxq = vbx*(2*q1**2 + 2*q2**2 - 1) - vby*(2*q1*q4 - 2*q2*q3) + vbz*(2*q1*q3 + 2*q2*q4)
    dyq = vby*(2*q1**2 + 2*q3**2 - 1) + vbx*(2*q1*q4 + 2*q2*q3) - vbz*(2*q1*q2 - 2*q3*q4)
    dzq = vbz*(2*q1**2 + 2*q4**2 - 1) - vbx*(2*q1*q3 - 2*q2*q4) + vby*(2*q1*q2 + 2*q3*q4)
    dq1 = - (q2*wx)/2 - (q3*wy)/2 - (q4*wz)/2
    dq2 = (q1*wx)/2 - (q4*wy)/2 + (q3*wz)/2
    dq3 = (q4*wx)/2 + (q1*wy)/2 - (q2*wz)/2
    dq4 = (q2*wy)/2 - (q3*wx)/2 + (q1*wz)/2
    dvbx = vby*wz - vbz*wy + g0*(2*q1*q3 - 2*q2*q4)
    dvby = vbz*wx - vbx*wz - g0*(2*q1*q2 + 2*q3*q4)
    dvbz = vbx*wy - vby*wx - g0*(2*q1**2 + 2*q4**2 - 1) + (Ct*(w1**2 + w2**2 + w3**2 + w4**2))/mq
    dwx = -(Ct*l*(w1**2 + w2**2 - w3**2 - w4**2) - Iyy*wy*wz + Izz*wy*wz)/Ixx
    dwy = -(Ct*l*(w1**2 - w2**2 - w3**2 + w4**2) + Ixx*wx*wz - Izz*wx*wz)/Iyy
    dwz = -(Cd*(w1**2 - w2**2 + w3**2 - w4**2) - Ixx*wx*wy + Iyy*wx*wy)/Izz

    # Explicit and Implicit functions
    f_expl = vertcat(dxq, dyq, dzq, dq1, dq2, dq3, dq4, dvbx, dvby, dvbz, dwx, dwy, dwz)
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

def export_path_constraint():

    # obstacles position
    bx  = 0.96286
    by  = -0.9395
    bz  = 0.9759
    X_obst = DM([bx,by,bz])

    crazyflie_model = export_ode_model()

    #distanceF_obs1 = norm_2(crazyflie_model.x[0:3]-o1p)
    #distanceF_obs2 = norm_2(crazyflie_model.x[0:3]-o2p)
    #distanceF_obs3 = norm_2(crazyflie_model.x[0:3]-o3p)

    #distance_function = vertcat(distanceF_obs1, distanceF_obs2, distanceF_obs3)
    distance_function = norm_2(crazyflie_model.x[0:3]-X_obst) + crazyflie_model.u[4]

    path_constraint = acados_constraint()
    path_constraint.expr = distance_function
    path_constraint.x = crazyflie_model.x
    path_constraint.u = crazyflie_model.u
    path_constraint.nc = 1
    path_constraint.name = 'distance_function'

    return path_constraint
