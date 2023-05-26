from acados_template import AcadosOcp, AcadosOcpSolver
from bluerov2 import export_bluerov2_model
import numpy as np
import casadi
#from utils import plot_pendulum
import math
from scipy.linalg import block_diag

def main():
    # create ocp object to formulate the OCP
    ocp = AcadosOcp()

    # set model
    model = export_bluerov2_model()
    ocp.model = model

    Tf = 1.0
    
    nx = model.x.size()[0]
    nu = model.u.size()[0]
    ny = nx + nu                # y is x and u concatenated for compactness of the loss function
    
    N = 100

    # set dimensions
    ocp.dims.N = N

    # set cost
    W_x = np.diag([100, 100, 150, 10, 10, 100, 10, 10, 10, 10, 10, 10])    #Q_mat
    W_u = np.diag([0.008, 0.008, 0.0001, 0.0001])                           #R_mat
    W = block_diag(W_x, W_u)
    ocp.cost.W_e = W_x
    ocp.cost.W = W

    # the 'EXTERNAL' cost type can be used to define general cost terms
    # NOTE: This leads to additional (exact) hessian contributions when using GAUSS_NEWTON hessian.
    ocp.cost.cost_type = 'NONLINEAR_LS'                 # weights times states (nonlinear relationship)
    ocp.cost.cost_type_e = 'NONLINEAR_LS'               # end states cost
    #ocp.model.cost_expr_ext_cost = model.x.T @ W_x @ model.x + model.u.T @ W_u @ model.u
    #ocp.model.cost_expr_ext_cost_e = model.x.T @ W_x @ model.x
    
    # Optimization costs
    ocp.cost.Vx = np.zeros((ny,nx))                     # raise dim of x to the dim of y
    ocp.cost.Vx[:nx,:nx] = np.eye(nx)                   # weight only x
    ocp.cost.Vx_e = np.eye(nx)                          # end x cost
    ocp.cost.Vu = np.zeros((ny,nu))                     # raise the dim of u to the dim of y
    ocp.cost.Vu[-nu:,-nu:] = np.eye(nu)                 # weight only u
    

    # set constraints
    u_min = np.array([-300, -300, -300, -300])
    u_max = np.array([300, 300, 300, 300])
    ocp.constraints.lbu = u_min
    ocp.constraints.ubu = u_max
    ocp.constraints.idxbu = np.array([0,1,2,3])         # indices of bounds on u

    ocp.constraints.x0 = np.array([0.0, 0.0, -34.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    # reference trajectory (will be overwritten later)
    x_ref = np.zeros(nx)
    ocp.cost.yref = np.concatenate((x_ref, np.array([0.0, 0.0, 0.0, 0.0])))
    ocp.cost.yref_e = x_ref

    # set options
    ocp.solver_options.qp_solver = 'FULL_CONDENSING_HPIPM' # FULL_CONDENSING_QPOASES
    # PARTIAL_CONDENSING_HPIPM, FULL_CONDENSING_QPOASES, FULL_CONDENSING_HPIPM,
    # PARTIAL_CONDENSING_QPDUNES, PARTIAL_CONDENSING_OSQP, FULL_CONDENSING_DAQP
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON' # 'GAUSS_NEWTON', 'EXACT'
    ocp.solver_options.integrator_type = 'ERK'
    #ocp.solver_options.qp_solver_cond_N = 5
    ocp.solver_options.print_level = 0
    ocp.solver_options.nlp_solver_type = 'SQP_RTI' # SQP_RTI, SQP


    # set prediction horizon
    ocp.solver_options.tf = Tf

    ocp_solver = AcadosOcpSolver(ocp, json_file = 'acados_ocp.json')

    simX = np.ndarray((N+1, nx))
    simU = np.ndarray((N, nu))

    status = ocp_solver.solve()
    ocp_solver.print_statistics() # encapsulates: stat = ocp_solver.get_stats("statistics")

    if status != 0:
        raise Exception(f'acados returned status {status}.')

    # get solution
    for i in range(N):
        simX[i,:] = ocp_solver.get(i, "x")
        simU[i,:] = ocp_solver.get(i, "u")
    simX[N,:] = ocp_solver.get(N, "x")

    #plot_pendulum(np.linspace(0, Tf, N+1), Fmax, simU, simX, latexify=False)


if __name__ == '__main__':
    main()