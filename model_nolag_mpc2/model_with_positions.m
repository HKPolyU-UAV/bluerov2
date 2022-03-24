clear all;close all;clc
load('A.mat')
load('B.mat')

lagSize = 1;
outputSize = 12;
inputSize = 7;
originalOutputSize = 8;
ns = 800; % time step

% create a 12*12 null matrix A with positions
% contains variables: vx, vy, vz, pitch, pitch rate, heading rate, roll,
% roll rate, x, y, z, heading
A_position = zeros(outputSize,inputSize + outputSize);

% create a 12*1 null matrix B with positions
B_position = zeros(outputSize,1);

% write original A matrix into A matrix with positions
A_position(1:originalOutputSize, 1:originalOutputSize) = A(:,1:originalOutputSize);
A_position(1:8, 13:19) = A(:,originalOutputSize+1:inputSize+originalOutputSize);
A_position(9:12, 9:12) = eye(4);
A_position(12,6) = 0.2; % heading dependence on heading rate

% write original B matrix into B matrix with positions
B_position(1:originalOutputSize,1) = B(1:originalOutputSize,1);


% Initialize rotation matrix
rotation_matrix = zeros(3,3);

% initial AUV state
% y0 = [0,0,0,0,0,0,0,0,0,0,0,0];
y0 = zeros(1,outputSize);

yHistory = y0;

% lat_aft, lat_fore, vert_aft, vert_fore, current_rpm,current_elevator, current_rudder
u_initial = [0,0,0,0,400,5,-2.8];
u0 = repmat(u_initial,lagSize,1);
uHistory = u0;
% u = randi(rand_range,[200,inputSize]);
u = repmat(u_initial', 1, ns);
u = u';


% Initialize state vector
xn = [];
for output = 1:outputSize
    xn = [xn;y0(end:-1:1,output)];
end
for input = 1:inputSize
    xn = [xn;u0(end:-1:1,input)];
end

% Initialize array stores X, Y, Z position
X = [];
Y = [];
Z = [];




for n = 1:ns
    
    % quick way to get current heading rate, pitch & roll
    % will recompute once the updated rotation model is in place
    yn = A_position*xn + B_position;
    disp(yn)
    
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
    % across 'first time' for each of the first three variables (vx,
    % vy, vz)
    A_position(9:11,1:lagSize:3*lagSize) = rotation_matrix(1:3,1:3);

    
    % simulate the model
    yn = A_position*xn + B_position;
    
    yHistory = circshift(yHistory,-1,1);
    yHistory(end,:) = yn';
    uHistory = circshift(uHistory,-1,1);
    uHistory(end,:) = u(n,:);
    
    
    % store the current coordinates into array
    X(n) = yn(9);% A_lastState(9,1:12)*xn(169:180,1);
    Y(n) = yn(10);%A_lastState(10,1:12)*xn(169:180,1);
    Z(n) = yn(11);%A_lastState(11,1:12)*xn(169:180,1);
    
    % reset state matrix
    xn = [];
    for output = 1:outputSize
        xn = [xn;yHistory(end:-1:1,output)];    
    end
    for input = 1:inputSize
        xn = [xn;uHistory(end:-1:1,input)];
    end
end
AB = ctrb(A_position(1:12,1:12),A_position(1:12,13:19));
rank(AB)
CA = ctrb(A_position(1:12,1:12)',eye(12)');
rank(CA)
figure(1);
plot3(X,Y,Z,'-')
axis equal
xlabel('x(t)')
ylabel('y(t)')
zlabel('z(t)')
title('Simulated AUV trajectory without control')



