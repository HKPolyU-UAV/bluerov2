% x1 - current in [-1,1]
% x2 - angular velocity in [-1,1]
% u  - control input in [-1,1]

function f = dyn_motor_scaled( t,x,u )
f = [   19.10828025-39.3153*x(1,:)-32.2293*x(2,:).*u;
       -3.333333333-1.6599*x(2,:)+22.9478*x(1,:).*u ];
end

%Model from ([1], Eq. (9))
%   dx1 = -(Ra/La)*x(1) - (km/La)*x(2)*u + ua/La
%   dx2   = -(B/J)*x(2) + (km/J)*x(1)*u - taul/J;
%   y = x1
% Parameters La = 0.314; Ra = 12.345; km = 0.253; J = 0.00441; B = 0.00732; taul = 1.47; ua = 60;
% Constraints (scaled to [-1,1] in the final model)
% x1min = -10; x1max = 10;
% x2min = -100; x2max = 100;
% umin = -4; umax = 4;
% [1] S. Daniel-Berhe and H. Unbehauen. Experimental physical parameter
% estimation of a thyristor driven DC-motor using the HMF-method.
% Control Engineering Practice, 6:615?626, 1998.