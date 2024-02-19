from acados_template import AcadosModel
from casadi import SX, vertcat, sin, cos, fabs
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
    u1 = SX.sym('u1')               # control force regard to surge
    u2 = SX.sym('u2')               # control signal regard to sway
    u3 = SX.sym('u3')               # control signal regard to heave
    u4 = SX.sym('u4')               # control signal regard to yaw
    sym_u = vertcat(u1,u2,u3,u4)

    # parameters
    disturbance_x = SX.sym('disturbance_x')
    disturbance_y = SX.sym('disturbance_y')
    disturbance_z = SX.sym('disturbance_z')
    # disturbance_phi = SX.sym('disturbance_phi')
    # disturbance_theta = SX.sym('disturbance_theta')
    disturbance_psi = SX.sym('disturbance_psi')

    added_mass_x = SX.sym('added_mass_x')
    added_mass_y = SX.sym('added_mass_y')
    added_mass_z = SX.sym('added_mass_z')
    added_mass_n = SX.sym('added_mass_n')

    linear_d_x = SX.sym('linear_d_x')
    linear_d_y = SX.sym('linear_d_y')
    linear_d_z = SX.sym('linear_d_z')
    linear_d_n = SX.sym('linear_d_n')

    nonlinear_d_x = SX.sym('nonlinear_d_x')
    nonlinear_d_y = SX.sym('nonlinear_d_y')
    nonlinear_d_z = SX.sym('nonlinear_d_z')
    nonlinear_d_n = SX.sym('nonlinear_d_n')
    
    sym_p = vertcat(disturbance_x,disturbance_y,disturbance_z,disturbance_psi,
                    added_mass_x,added_mass_y,added_mass_z,added_mass_n,
                    linear_d_x,linear_d_y,linear_d_z,linear_d_n,
                    nonlinear_d_x,nonlinear_d_y,nonlinear_d_z,nonlinear_d_n)

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
    bouyancy = 0.66                                # net bouyancy forcy
    rotor_constant = 0.026546960744430276
    
    # added_mass = np.array([1.7182,0,5.468,0,0,0.4006])
    # M = np.diag([m+added_mass[0], m+added_mass[1], m+added_mass[2], Ix, Iy, Iz+added_mass[5]]) # M_RB + M_A
    # M = np.diag([m+added_mass_x, m+added_mass_y, m+added_mass_z, Ix, Iy, Iz+added_mass_n]) # M_RB + M_A
    # M[0,4] = m*ZG
    # M[1,3] = -m*ZG
    # M[3,1] = -m*ZG
    # M[4,0] = m*ZG
    # M_inv = np.linalg.inv(M)
    
    K = np.array([[0.707,0.707,-0.707,-0.707,0,0],
                  [0.707,-0.707,0.707,-0.707,0,0],
                  [0,0,0,0,1,1],                    # propulsion matrix
                  [0,0,0,0,0,0],
                  [0,0,0,0,0,0],
                  [0.167,-0.167,-0.175,0.175,0,0]])
    
    
    t0 = (-u1+u2+u4)/rotor_constant
    t1 = (-u1-u2-u4)/rotor_constant
    t2 = (u1+u2-u4)/rotor_constant                                 # control inputs to thrusts of each propeller
    t3 = (u1-u2+u4)/rotor_constant
    t4 = -u3/rotor_constant
    t5 = -u3/rotor_constant
    
    Kt0 = K[0,0]*t0+K[0,1]*t1+K[0,2]*t2+K[0,3]*t3+K[0,4]*t4+K[0,5]*t5
    Kt1 = K[1,0]*t0+K[1,1]*t1+K[1,2]*t2+K[1,3]*t3+K[1,4]*t4+K[1,5]*t5
    Kt2 = K[2,0]*t0+K[2,1]*t1+K[2,2]*t2+K[2,3]*t3+K[2,4]*t4+K[2,5]*t5   # thrusts to forces and moments
    Kt3 = K[3,0]*t0+K[3,1]*t1+K[3,2]*t2+K[3,3]*t3+K[3,4]*t4+K[3,5]*t5
    Kt4 = K[4,0]*t0+K[4,1]*t1+K[4,2]*t2+K[4,3]*t3+K[4,4]*t4+K[4,5]*t5
    Kt5 = K[5,0]*t0+K[5,1]*t1+K[5,2]*t2+K[5,3]*t3+K[5,4]*t4+K[5,5]*t5
    

    # dynamics
    
    du = 1/(m+added_mass_x)*(Kt0+m*r*v-m*q*w-bouyancy*sin(theta)+disturbance_x+linear_d_x*u+nonlinear_d_x*fabs(u)*u)
    dv = 1/(m+added_mass_y)*(Kt1-m*r*u+m*p*w+bouyancy*cos(theta)*sin(phi)+disturbance_y+linear_d_y*v+nonlinear_d_y*fabs(v)*v)
    dw = 1/(m+added_mass_z)*(Kt2+m*q*u-m*p*v+bouyancy*cos(theta)*cos(phi)+disturbance_z+linear_d_z*w+nonlinear_d_z*fabs(w)*w)
    dp = 1/Ix*(Kt3+(Iy-Iz)*q*r-m*ZG*g*cos(theta)*sin(phi))
    dq = 1/Iy*(Kt4+(Iz-Ix)*p*r-m*ZG*g*sin(theta))
    dr = 1/(Iz+added_mass_n)*(Kt5-(Iy-Ix)*p*q+disturbance_psi+linear_d_n*r+nonlinear_d_n*fabs(r)*r)
   
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
    model.p = sym_p
    model.cost_y_expr = cost_y_expr
    model.cost_y_expr_e = sym_x
    #model.con_h_expr = h_expr
    model.name = model_name
    #model.cost_expr_ext_cost = expr_ext_cost
    #model.cost_expr_ext_cost_e = expr_ext_cost_e 

    return model