function [x] = fnSimulate(xo,u_new,Horizon,dt,sigma)

global F;
global Fb;

x = xo;

% Create rollout as a function of new controls plus some added noise.
for k = 1:(Horizon-1)    
    x(:,k+1) = x(:,k) + F(u_new(1, k), x(2,k), x(3,k), x(4,k)) * dt;
    x(:,k+1) = x(:,k+1) + Fb(u_new(1,k), x(3,k)) * sqrt(dt) * sigma * randn;
end