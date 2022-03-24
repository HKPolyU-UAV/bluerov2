% KDV y_t + c_kdv*yy_x + y_xxx = u, where u is the control input
% Based on https://www.wikiwaves.org/Numerical_Solution_of_the_KdV
function y_end = kdv_solver(y0,u,simPar)

if(size(y0,1) > size(y0,2))
    y0 = y0';
    transp = 1;
end
if(size(u,1) > size(u,2))
    u = u';
end

if(~isfield(simPar,'timeStepConst'))
    simPar.timeStepConst = 1;
end

c_kdv = 1;

N = simPar.N; % 128, 256, 512
tmax = simPar.T;
timeStepConst = simPar.timeStepConst;

xm = pi;
x = linspace(-xm,xm,N);
delta_x = x(2) - x(1);
delta_k = 2*pi/(N*delta_x); % fourier domain
delta_t = timeStepConst / N^2; % time discretization % timeStepConst was 0.4
nmax = round(tmax/delta_t); % number of iteration

k = [0:delta_k:N/2*delta_k,-(N/2-1)*delta_k:delta_k:-delta_k];
c=16;


Y = fft(y0);

for n = 1:nmax
% first we solve the linear part
Y = Y.*exp(1i*k.^3*delta_t) + fft(u).* [delta_t, (exp(1i*k(2:end).^3*delta_t)-1) ./ (1i*k(2:end).^3) ];

%then we solve the non linear part
Y = Y - delta_t*(0.5*c_kdv*1i*k.*fft(real(ifft(Y)).^2));
end

y_end = real(ifft(Y));

if(transp == 1)
    y_end = y_end';
end



