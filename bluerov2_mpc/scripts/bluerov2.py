from acados_template import AcadosModel
from casadi import SX, vertcat, sin, cos
import numpy as np
from scipy.linalg import block_diag
import math

def export_bluerov2_model() -> AcadosModel:

    model_name = 'bluerov2'

    # states
    x = SX.sym('x')                 # earth position x
    y = SX.sym('y')                 # earth position y
    z = SX.sym('z')                 # earth position z
    phi = SX.sym('phi')             # roll angle
    theta = SX.sym('theta')         # pitch angle
    psi = SX.sym('psi')             # yaw angle
    u = SX.sym('u')                 # earth velocity x
    v = SX.sym('v')                 # earth velocity y
    w = SX.sym('w')                 # earth velocity z
    p = SX.sym('p')                 # roll velocity
    q = SX.sym('q')                 # pitch velocity
    r = SX.sym('r')                 # yaw velocity
    sym_x = vertcat(x,y,z,phi,theta,psi,u,v,w,p,q,r)

    # controls
    u1 = SX.sym('u1')               # control signal regard to surge
    u2 = SX.sym('u2')               # control signal regard to sway
    u3 = SX.sym('u3')               # control signal regard to heave
    u4 = SX.sym('u4')               # control signal regard to yaw
    sym_u = vertcat(u1,u2,u3,u4)

    # xdot for f_impl
    x_dot = SX.sym('x_dot')
    y_dot = SX.sym('y_dot')
    z_dot = SX.sym('z_dot')
    phi_dot = SX.sym('phi_dot')
    theta_dot = SX.sym('theta_dot')
    psi_dot = SX.sym('psi_dot')
    u_dot = SX.sym('u_dot')
    v_dot = SX.sym('v_dot')
    w_dot = SX.sym('w_dot')
    p_dot = SX.sym('p_dot')
    q_dot = SX.sym('q_dot')
    r_dot = SX.sym('r_dot')
    sym_xdot = vertcat(x_dot,y_dot,z_dot,phi_dot,theta_dot,psi_dot,u_dot,v_dot,w_dot,p_dot,q_dot,r_dot)

    # system parameters
    m = 11.26                                       # mass
    Ix = 0.3                                        # inertial
    Iy = 0.63
    Iz = 0.58
    ZG = 0.02
    g = 9.81
    added_mass = np.array([-5.5,-5.5,-5.5,-0.12,-0.12,-0.12])
    M = np.diag([m+added_mass[0], m+added_mass[1], m+added_mass[2], Ix+added_mass[3], Iy+added_mass[4], Iz+added_mass[5]]) # M_RB + M_A
    M_inv = np.linalg.inv(M)

    # dynamics -2.828*(80/(1+(-4*math.e**((u1**3))))-40)
    du = M_inv[0,0]*(-2.828*u1+m*r*v-m*q*w)
    dv = M_inv[1,1]*(-2.828*u2-m*r*u+m*p*w)
    dw = M_inv[2,2]*(-2*u3+m*q*u-m*p*v)
    dp = M_inv[3,3]*((Iy-Iz)*q*r-m*ZG*g*cos(theta)*sin(phi))
    dq = M_inv[4,4]*((Iz-Ix)*p*r-m*ZG*g*sin(theta))
    dr = M_inv[5,5]*(-0.668*u4-(Iy-Ix)*p*q)
    dx = (cos(psi)*cos(theta))*u + (-sin(psi)*cos(phi)+cos(psi)*sin(theta)*sin(phi))*v + (sin(psi)*sin(phi)+cos(psi)*cos(phi)*sin(theta))*w
    dy = (sin(psi)*cos(theta))*u + (cos(psi)*cos(phi)+sin(phi)*sin(theta)*sin(psi))*v + (-cos(psi)*sin(phi)+sin(theta)*sin(psi)*cos(phi))*w
    dz = (-sin(theta))*u + (cos(theta)*sin(phi))*v + (cos(theta)*cos(phi))*w
    dphi = p + (sin(psi)*sin(theta)/cos(theta))*q + cos(phi)*sin(theta)/cos(theta)*r
    dtheta = (cos(phi))*q + (sin(phi))*r
    dpsi = (sin(phi)/cos(theta))*q + (cos(phi)/cos(theta))*r

    f_expl = vertcat(dx,dy,dz,dphi,dtheta,dpsi,du,dv,dw,dp,dq,dr)
    f_impl = sym_xdot - f_expl
  
    # constraints
    h_expr = sym_u

    # nonlinear least sqares
    cost_y_expr = vertcat(sym_x, sym_u)
    
    model = AcadosModel()
    model.f_impl_expr = f_impl
    model.f_expl_expr = f_expl
    model.x = sym_x
    model.xdot = sym_xdot
    model.u = sym_u
    model.cost_y_expr = cost_y_expr
    model.cost_y_expr_e = sym_x
    #model.con_h_expr = h_expr
    model.name = model_name
    #model.cost_expr_ext_cost = expr_ext_cost
    #model.cost_expr_ext_cost_e = expr_ext_cost_e 

    return model