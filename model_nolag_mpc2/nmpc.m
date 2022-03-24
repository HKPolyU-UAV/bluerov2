tic
clear all;close all;clc
load('A.mat')
load('B.mat')

addpath('./Resources')
addpath('./Resources/qpOASES-3.1.0/interfaces/matlab') 

lagSize = 1;
outputSize = 12;
inputSize = 7;
originalOutputSize = 8;
ns = 800; % time step

A_position = zeros(outputSize,outputSize);
B_position = zeros(outputSize,inputSize);
offset_position = zeros(outputSize,1);

% write original A matrix into A matrix with positions
A_position(1:originalOutputSize, 1:originalOutputSize) = A(:,1:originalOutputSize);
B_position(1:originalOutputSize, 1:inputSize) = A(:,originalOutputSize+1:inputSize+originalOutputSize);
A_position(9:12, 9:12) = eye(4);
A_position(12,6) = 0.2; % heading dependence on heading rate

offset_position(1:originalOutputSize,1) = B(1:originalOutputSize,1);


% Initialize rotation matrix
rotation_matrix = zeros(3,3);


% initial AUV state
y0 = zeros(1,outputSize);
yHistory = y0;

% Initialize state vector
xn = [];
u_mpc = ones(7,1);
for output = 1:outputSize
    xn = [xn;y0(end:-1:1,output)];
end

% Initialize array stores X, Y, Z position
X = [];
Y = [];
Z = [];
error = [];

% dynamic positioning aim setting
X_aim = 50*ones(1,ns);
Y_aim = 50*ones(1,ns);
Z_aim = -5*ones(1,ns);

% Trajectory tracking aim setting
% X_aim(1:350) = 40*ones(1,350);
% X_aim(351:700) = 60*ones(1,350);
% X_aim(701:1000) = 80*ones(1,300);
% Y_aim =10+0.1*(1:ns);

% X_base = linspace(-0.5*pi,2*pi,ns);
% Y_base = linspace(0*pi,2*pi,ns);
% X_aim = 25*cos(X_base);
% Y_aim = 25*sin(Y_base);

X_base = linspace(0*pi,2*pi,ns-100);
Y_base = linspace(0*pi,2*pi,ns-100);
X_aim(1:100) = 0.2*(1:100);
X_aim(101:800) = 20*cos(X_base);
Y_aim(1:100) = zeros(1,100);
Y_aim(101:800) = 20*sin(Y_base);

% X_aim(1:300) = 0.25*(1:300);
% X_aim(301:600) = 75+0.1*(1:300);
% X_aim(601:1200) = 105*ones(1,600);
%X_aim(901:ns) = 105-0.05*(1:300);

% X_aim(1:200) = 0.35*(1:200);
% X_aim(201:500) = 70+0.15*(1:300);
% X_aim(501:800) = 115+0.05*(1:300);
% X_aim(801:ns) = 130*ones(1,400);


% X_base = linspace(0.5*pi,1*pi,400);
% X_aim = zeros(1,ns);
% X_aim(1,1:200) = 0.5*(1:200);
% X_aim(201:400) = 100;
% X_aim(401:ns) = 100-0.5*(1:200);
% 
% Y_aim = (1:ns)*0.2;

% mpc parameters
d_mpc = offset_position;
Q_mpc = 800 * eye(3);
R_mpc = 0.1 * eye(7);
N_mpc = 35;

% state and control input's constriants 
x_min_mpc = [-1.5;-1.5;-1.5;-inf;-inf;-inf;-inf;-inf;-inf;-inf;-inf;-inf];
x_max_mpc = [1.5;1.5;1.5;inf;inf;inf;inf;inf;inf;inf;inf;inf];
u_min_mpc = [-1000;-1000;-1000;-1000;0;-12;-12];
u_max_mpc = [1000;1000;1000;1000;500;12;12];


%%
for n = 1:ns
    % quick way to get current heading rate, pitch & roll
    % will recompute once the updated rotation model is in place
    
    yn = A_position*xn(1:12,:) + B_position*u_mpc;
    
    headingRate = yHistory(end, 6);
    pitch = yHistory(end, 4);
    roll = yHistory(end, 7);
    
    heading = yHistory(end, 12) + headingRate * 0.2;
    
    % put current heading, pitch, roll value into rotation matrix formula
    rotation_11 = 0.2*(cos(heading)*cos(pitch));
    rotation_12 = 0.2*(cos(heading)*sin(pitch)*sin(roll)-sin(heading)*cos(roll));
    rotation_13 = 0.2*(cos(heading)*sin(pitch)*cos(roll)+sin(heading)*sin(roll));
    rotation_21 = 0.2*(sin(heading)*cos(pitch));
    rotation_22 = 0.2*(sin(heading)*sin(pitch)*sin(roll)+cos(heading)*cos(roll));
    rotation_23 = 0.2*(sin(heading)*sin(pitch)*cos(roll)-cos(heading)*sin(roll));
    rotation_31 = 0.2*(-sin(pitch));
    rotation_32 = 0.2*(cos(pitch)*sin(roll));
    rotation_33 = 0.2*(cos(pitch)*cos(roll));
    rotation_matrix = [rotation_11 rotation_12 rotation_13;
        rotation_21 rotation_22 rotation_23;
        rotation_31 rotation_32 rotation_33];
    % write current rotation matrix into A_position
    A_position(9:11,1:lagSize:3*lagSize) = rotation_matrix(1:3,1:3);
    
    % simulate the model
    A_mpc = A_position;
    B_mpc = B_position;
    C_mpc = [A_position(9,:); A_position(10,:);A_position(11,:)];
    
    koopmanMPC  = getMPC(A_mpc,B_mpc,C_mpc,d_mpc,Q_mpc,R_mpc,Q_mpc,N_mpc,u_min_mpc, u_max_mpc, x_min_mpc, x_max_mpc,'qpoases');
    yr = [X_aim(n);Y_aim(n);Z_aim(n)];
    u_mpc= koopmanMPC(xn(1:12,:),yr);
    
    
    yn = A_position*xn(1:12,:) + B_position * u_mpc;
    
    yHistory = circshift(yHistory,-1,1);
    yHistory(end,:) = yn';
%     uHistory = circshift(uHistory,-1,1);
%     uHistory(end,:) = u_mpc';
    
    
    % store the current coordinates into array
    X(n) = yHistory(end,9);
    Y(n) = yHistory(end,10);
    Z(n) = yHistory(end,11);
    
    error(n) = sqrt((X_aim(n)-X(n))^2+(Y_aim(n)-Y(n))^2);
    
    % store current vx, vy, vz
    vx_vector(n) = yn(1);
    vy_vector(n) = yn(2);
    vz_vector(n) = yn(3);
    
    % store current coefficients
    heading_vector(n) = yn(12);
    pitch_vector(n) = yn(4);
    roll_vector(n) = yn(7);
    lat_aft(n) = u_mpc(1);
    lat_for(n) = u_mpc(2);
    ver_aft(n) = u_mpc(3);
    ver_for(n) = u_mpc(4);
    rpm(n) = u_mpc(5);
    elevator(n) = u_mpc(6);
    rudder(n) = u_mpc(7);
    
    % reset state matrix
    xn = [];
    for output = 1:outputSize
        xn = [xn;yHistory(end:-1:1,output)];    
    end

end


figure(1);
subplot(3,1,1)
plot(X);hold on
plot(X_aim)
legend('Actual AUV trajectory','Generated reference path')
xlabel('timestep')
title('x coordinate versus timestep')

subplot(3,1,2)
plot(Y);hold on
plot(Y_aim)
legend('Actual AUV trajectory','Generated reference path')
xlabel('timestep')
title('y coordinate versus timestep')

subplot(3,1,3)
plot(Z);hold on
plot(Z_aim)
legend('Actual AUV trajectory','Generated reference path')
xlabel('timestep')
title('z coordinate versus timestep')

% % plot vx, vy, vz
% figure(2);
% plot(vx_vector);hold on
% plot(vy_vector);hold on
% plot(vz_vector);hold on
% legend('vx','vy','vz');
% 

% % plot heading, pitch, roll
% figure(3);
% subplot(3,1,1)
% plot(heading_vector);
% xlabel('timestep')
% title('heading')
% subplot(3,1,2)
% plot(pitch_vector);
% xlabel('timestep')
% title('pitch')
% subplot(3,1,3)
% plot(roll_vector);
% xlabel('timestep')
% title('roll')

% % plot control inputs
% figure(4);
% subplot(7,1,1)
% plot(lat_aft);
% xlabel('timestep')
% title('lateral aft(rpm)')
% subplot(7,1,2)
% plot(lat_for);
% xlabel('timestep')
% title('lateral fore(rpm)')
% subplot(7,1,3)
% plot(ver_aft);
% xlabel('timestep')
% title('vertical aft(rpm)')
% subplot(7,1,4)
% plot(ver_for);
% xlabel('timestep')
% title('vertical fore(rpm)')
% subplot(7,1,5)
% plot(rpm)
% xlabel('timestep')
% title('tail thrust(rpm)')
% subplot(7,1,6)
% plot(elevator)
% xlabel('timestep')
% title('elevator(degree)')
% subplot(7,1,7)
% plot(rudder)
% xlabel('timestep')
% title('rudder(degree)')

% Plot trajectory tracking result
figure(5)
plot(X,Y,'LineWidth',1);hold on
plot(X_aim,Y_aim,'LineWidth',1);
xlabel('x')
ylabel('y')
legend('Actual AUV trajectory','Generated reference path')
title('Tracking results')

figure(6)
plot(error)
xlabel('timestep')
ylabel('error')


toc



