function [U, X, optval] = solveMPCprob(A, B, C, d, Q, R, QN, N, Ulb, Uub, Xlb, Xub, x0, yr, ulin, qlin)
% Solves MPC problem in dense formulation and returns the predicted
% control sequence U, the predicted state sequence X as well as the
% optimal value optval.

%If you want to use MPC in closed loop with the same data (A,B,C,...),
%then use getMPC.m instead (it's faster since all fixed data is precomputed and
%the optimization problem is warm started).

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

solver = 'qpoases'; %or quadprog


n = size(A,1); % Number of states
m = size(B,2); % Numbet of control inputs

if (~exist('C','var') || isempty(C))
    C = eye(n,n);
end
p = size(C,1); % Number of outputs


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
        error('The dimension of Xub or Xlb seems to be wrong')
    end
    Ulb = repmat(Ulb,1,N); Uub = repmat(Uub,1,N);
end
Xub(Xub == Inf) = NaN;
Xlb(Xlb == -Inf) = NaN;


% Affine term in the dynamics - handled by state inflation
if( exist('d','var') && ~isempty(d) && norm(d) ~= 0 )
    A = [A eye(n,n) ; zeros(n,n) eye(n,n)];
    B = [B ; zeros(n,m)];
    C = [C zeros(p,n)];
    if(~isempty(Xlb))
        Xlb = [Xlb ; NaN(n,N)];
    end
    if(~isempty(Xub))
        Xub = [Xub ; NaN(n,N)];
    end
    n = size(A,1);
    x0 = [x0;d];
end



% Output reference
if(~exist('yr','var') || isempty(yr))
    yr = zeros(p*N,1);
else if(size(yr,2) == 1)
        yr = repmat(yr,N,1);
    else
        yr = yr(:);
    end
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

Qb = zeros(p*N,p*N);
Qb(1:p*(N-1),1:p*(N-1)) = bdiag(Q,N-1); Qb(end-p+1:end,end-p+1:end) = QN;
Cb = bdiag(C,N);
Rb = bdiag(R,N);



% Bounds on the states
Aineq = []; bineq = [];
if (~isempty(Xub))
    Aineq = [Aineq; Bb];
    bineq = [bineq; Xub(:)-Ab*x0];
end
if (~isempty(Xlb))
    Aineq = [Aineq; -Bb];
    bineq = [bineq;-Xlb(:)+Ab*x0];
end

Aineq = Aineq(~isnan(bineq),:);
bineq = bineq(~isnan(bineq),:);


H = 2*(Bb'*Cb'*Qb*Cb*Bb + Rb);
f = (2*x0'*Ab'*Cb'*Qb*Cb*Bb - 2*yr'*Qb*Cb*Bb)' + ulin + + Bb'*(Cb'*qlin);



if(isequal(solver,'quadprog'))
    [res, optval, flag] = quadprog((H+H')/2,f,Aineq,bineq,[],[],Ulb(:),Uub(:));
    optval = optval + x0'*Q*x0;
    if (flag == -2)
        optval = Inf;
    end
end


if(isequal(solver,'qpoases'))
    [res,optval,flag,iter,lambda,auxOutput] = qpOASES( (H+H')/2,f,Aineq,Ulb(:),Uub(:),[],bineq);
    optval = optval + x0'*Q*x0;
    if (flag == -2)
        optval = Inf;
    end
end



res = res(1:m*N);
X = [x0;Ab*x0+Bb*res];
U = reshape(res,m,N);

end