function MPC_ctr = getMPC(A, B, C, d ,Q ,R, QN ,N, Ulb, Uub, Xlb, Xub, solver, ulin, qlin)
% Returns an MPC controller MPC_ctr. The control input is then generated using the
% syntax MPC_ctr(x0,yr), where x0 is the current state and yr the reference to be tracked (can be zero)

% OUTPUT:
% Koopman MPC controller MPC_ctr(x0,yr)

% Dynamics:
% x^+ = A*x + B*u + d
% y   = C*x

% Cost:
% J = (y_N - yr_N)'*Q_N*(y_N - yr_N) + sum_{i=0:N-1} [ (y_i - yr_i)'*Q*(y_i - yr_i) + u_i'*R*u_i + ulin_i'u + qlin'*y ]

% INPUTS:

% Xlb, Xub:  n x 1 or n x N matrix
% Ulb, Uub: m x 1 vector or m x N matrix
% ** If elements of Xlb, Xub, Ulb, Uub not present, set them to NaN or to +-Inf **

% yr - n_outputs x 1 vector or n_outputs x N matrix


% ulin - linear term in the cost
% qlin - linear term in the cost

% Example: MPC_ctr = getKoopmanMPC(A,B,C,d,Q,R,Q,N,umin, umax, xmin, xmax);
% MPC_ctr(x0,yr) generates the control input

if(~exist('solver','var') || isempty(solver))
    solver = 'qpoases'; %cplex or quadprog
end


n = size(A,1); % Number of states
m = size(B,2); % Number of control inputs


if (~exist('C','var') || isempty(C))
    C = eye(n,n);
end
p = size(C,1); % Number of outputs


x0 = zeros(n,1); % Dummy variable


if (~exist('Xlb','var'))
    Xlb = [];
end
if (~exist('Xub','var'))
    Xub = [];
end
if (size(Xub,2)  == 1 || size(Xlb,2)  == 1)
    if( numel(Xub) ~= n || numel(Xlb) ~= n)
        error('The dimension of Xub or Xlb seems to be wrong')
    end
    Xlb = repmat(Xlb,1,N); Xub = repmat(Xub,1,N);
end
if (size(Uub,2)  == 1 || size(Ulb,2)  == 1)
    if( numel(Uub) ~= m || numel(Ulb) ~= m)
        error('The dimension of Uub or Ulb seems to be wrong')
    end
    Ulb = repmat(Ulb,1,N); Uub = repmat(Uub,1,N);
end
Xub(Xub == Inf) = NaN;
Xlb(Xlb == -Inf) = NaN;


% Affine term in the dynamics - handled by state inflation
if( exist('d','var') && ~isempty(d) && norm(d) ~= 0 )
    A = [A speye(n,n) ; sparse(n,n) speye(n,n)];
    B = [B ; sparse(n,m)];
    C = [C sparse(p,n)];
    if(~isempty(Xlb))
        Xlb = [Xlb ; NaN(n,N)];
    end
    if(~isempty(Xub))
        Xub = [Xub ; NaN(n,N)];
    end
    n = size(A,1);
    x0 = [x0;d];
else
    d = NaN;
end


% Linear term in the cost
if(~exist('ulin','var') || isempty(ulin))
    ulin = zeros(m*N,1);
else
    if(numel(ulin) == m)
        if (size(ulin,2) > size(ulin,1))
            ulin = ulin';
        end
        ulin = repmat(ulin,N,1);
    end
end


if(~exist('qlin','var') || isempty(qlin))
    qlin = zeros(p*N,1);
else
    warning('Functionality of qlin not tested properly!')
    if(numel(qlin) == p)
        qlin = repmat(qlin(:),N,1);
    elseif(numel(qlin) == N*p)
        qlin = qlin(:);
    else
        error('Wrong size of qlin')
    end
end



[Ab, Bb] = createMPCmatrices(A,{B},N);

Bb = Bb{1};

Qb = sparse(p*N,p*N);
Qb(1:p*(N-1),1:p*(N-1)) = bdiag(Q,N-1); Qb(end-p+1:end,end-p+1:end) = QN;
Cb = bdiag(C,N);
Rb = bdiag(R,N);



% Bounds on the states
Aineq = []; bineq = [];
Xub = Xub(:); Xlb = Xlb(:);
if (~isempty(Xub))
    Aineq = [Aineq; Bb];
    bineq = [bineq; Xub - Ab*x0];
end
if (~isempty(Xlb))
    Aineq = [Aineq; -Bb];
    bineq = [bineq;-Xlb + Ab*x0];
end

Aineq = Aineq(~isnan(bineq),:);
bineq = bineq(~isnan(bineq),:);


H = 2*(Bb'*Cb'*Qb*Cb*Bb + Rb);
f = (2*x0'*Ab'*Cb'*Qb*Cb*Bb)' + ulin + Bb'*(Cb'*qlin); % Can be sped up by eleminating the transpose

Ulb = Ulb(:);
Uub = Uub(:);
H = (H+H')/2; % symetrize (in case of numerical errors)


% Build the controller
M1 = 2*( (Bb'*(Cb'*Qb*Cb))*Ab );
M2 = (-2*(Qb*Cb)*Bb)';

disp('Building controller with qp-Oases')
% Solve & initialize (Here used just for initialization of the QPoases
% homothopy based QP solvers. The solution is thrown away.)
[QP,res,optval] = qpOASES_sequence( 'i',H,f,Aineq,Ulb,Uub,[],bineq );

if( all(isnan(Xlb)) )
    Xlb = [];
end
if( all(isnan(Xub)) )
    Xub = [];
end

MPC_ctr = @(x00,yrr)(mpcController_qpoases_sequence(x00,yrr, QP,N,Ab,Xlb,Xub,M1,M2,ulin,d,Ulb,Uub,p,C,Q));



end




function [U,optval,flag] = mpcController_qpoases_sequence(x0,yr, QP,N,Ab,Xlb,Xub,M1,M2,ulin,d,Ulb,Uub,p,C,Q)
% INPUTS: x0, yr

% PARAMETERS: the rest

% M1 = (2*Ab'*Cb'*Qb*Cb*Bb)';
% M2 = (-2*Qb*Cb*Bb)';


if(~exist('yr','var') || isempty(yr))
    yr = zeros(p*N,1);
else if(size(yr,2) == 1)
        yr = repmat(yr,N,1);
    else
        yr = yr(:);
    end
end

if(~isnan(d))
    x0 = [x0 ; d];
end

% Linear part of the constratints
% Can be significantly sped up by selecting by carrying out the
% multiplication Ab*x0 only for those rows of Ab corresponding to non-nan
% entries of Xub or Xlb
bineq = [];
if (~isempty(Xub))
    bineq = [bineq; Xub - Ab*x0];
end
if (~isempty(Xlb))
    bineq = [bineq; -Xlb + Ab*x0];
end
bineq = bineq(~isnan(bineq),:);

% Linear part of the objective function
f = M1*x0 + M2*yr + ulin;

[U,optval,flag] = qpOASES_sequence( 'h',QP,f,Ulb,Uub,[],bineq );

%U = reshape(U,numel(ulin)/N,N); Return the whole predicted sequence
U = U(1:numel(ulin)/N); % Return just the first input
y = C*x0; % Should be y - yr, but yr adds just a constant term
optval = optval + y'*Q*y;
end

