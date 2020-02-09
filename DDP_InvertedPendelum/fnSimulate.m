function [x] = fnSimulate(xo,u_new,Horizon,dt,sigma)

global I1;
global b1;
global g;
global l1;

x = xo;

% Create rollout as a function of new controls plus some added noise.
for k = 1:(Horizon-1)
    
    A = zeros(2,1);
    B = zeros(2,1);

    % Fx
    A(1,1) = x(2,k);
    A(2,1) = -(g / l1) * sin(x(1,k)) - (b1 / I1) * x(2,k);

    % Fu
    B(1,1) = 0;
    B(2,1) = 1 / I1;
    
    x(:,k+1) = x(:,k) + A * dt + B * u_new(:,k) * dt;
    
%     det = d1 * d3 - d3^2 - (d2 * cos(x(2,k)))^2; 
%          M(1,1) = d1 + 2 * d2 * cos(x(2,k));
%          M(1,2) = d3 + d2 * cos(x(2,k));
%          M(2,1) = d3 + d2 * cos(x(2,k));
%          M(2,2) = d3;
% 
% 
%          C(1,1) = -x(4,k) * (2 * x(3,k)  + x(4,k) );
%          C(2,1) = x(3,k) ^2;
%          C = C * d2 * sin(x(2,k));
% 
%           det = d1 * d3 - d3^2 - (d2 * cos(x(2,k)))^2;
% 
%           Fx(1,1) = x(3,k);
%           Fx(2,1) = x(4,k);
%           Fx(3,1) = 1.0/det * (-d2 * d3 * (x(3,k) + x(4,k))^2 * sin(x(2,k)) - d2^2 * x(3,k)^2 * sin(x(2,k)) * cos(x(2,k)) - d2 * (b2_1 * x(3,k) + b2 * x(4,k))*cos(x(2,k))...
%                            + (d3 * b1 - d3 * b2_1) * x(3,k) + (d3 * b1_2 - d3 * b2) * x(4,k));
% 
%           Fx(4,1) = 1.0/det * ( d2 * d3 * x(4,k) * (2 * x(3,k) + x(4,k)) * sin(x(2,k)) + d1 * d2 * x(3,k)^2 * sin(x(2,k)) + d2^2 * (x(3,k) + x(4,k))^2 * sin(x(2,k))* cos(x(2,k))...
%                 + d2* ((2 * b2_1 - b1) * x(3,k) + (2 * b2 - b1_2)* x(4,k)) * cos(x(2,k))...
%                 +(d1 * b2_1 - d3 * b1) * x(3,k) + (d1 * b2 - d3 * b1_2) * x(4,k));
% 
% 
%           G_x(3,1) = d3;
%           G_x(3,2) = -(d3 + d2 * cos(x(2,k)));
%           G_x(4,1) = G_x(3,2);
%           G_x(4,2) = d1 + 2 *d2 * cos(x(2,k));
% 
% 
%           G_x = 1.0/det *G_x;


    
%     x(:,k+1) = x(:,k) + Fx * dt + G_x * u_new(:,k) * dt  + G_x * u_new(:,k) * sqrt(dt) * sigma * randn ;
end