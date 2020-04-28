function [x] = fnSimulate(xo,u_new,Horizon,dt,sigma, dynamics, num_timesteps)

if nargin < 7
    num_steps = Horizon - 1;
else
    num_steps = num_timesteps;
end

x = xo;

% Create rollout as a function of new controls plus some added noise.
for k = 1:num_steps
    x(:, k+1) = x(:, k) + dynamics.F(u_new(1, k), u_new(2, k), u_new(3, k), u_new(4, k), ...
        x(4, k), x(5, k), x(6, k), x(7, k), x(8, k), x(10, k), x(11, k), x(12, k)) * dt;
    x(:, k+1) = x(:, k+1) + dynamics.Fb(u_new(1, k), u_new(2, k), u_new(3, k), ...
        u_new(4, k), x(7, k), x(8, k)) * sqrt(dt) * sigma * randn;
end