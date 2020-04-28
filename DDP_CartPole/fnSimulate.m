function [x] = fnSimulate(xo,u_new,Horizon,dt,sigma, dynamics, num_timesteps)

if nargin < 7
    num_steps = Horizon - 1;
else
    num_steps = num_timesteps;
end

x = xo;

% Create rollout as a function of new controls plus some added noise.
for k = 1:num_steps    
    x(:,k+1) = x(:,k) + dynamics.F(u_new(1, k), x(2,k), x(3,k), x(4,k)) * dt;
    x(:,k+1) = x(:,k+1) + dynamics.Fb(u_new(1,k), x(3,k)) * sqrt(dt) * sigma * randn;
end