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
    
    % Compute the new state using the dynamics.
    x(:,k+1) = x(:,k) + A * dt + B * u_new(:,k) * dt;
    x(:,k+1) = x(:,k+1) + B * u_new(:,k) * sqrt(dt) * sigma * randn;
end