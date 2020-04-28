function [x] = fnSimulate(xo,u_new,Horizon,dt,sigma, env_params, num_timesteps)

if nargin < 7
    num_steps = Horizon - 1;
else
    num_steps = num_timesteps;
end

x = xo;

% Create rollout as a function of new controls plus some added noise.
for k = 1:num_steps
    
    A = zeros(2,1);
    B = zeros(2,1);

    % Fx
    A(1,1) = x(2,k);
    A(2,1) = -(env_params.g / env_params.l1) * sin(x(1,k))...
        - (env_params.b1 / env_params.I1) * x(2,k);

    % Fu
    B(1,1) = 0;
    B(2,1) = 1 / env_params.I1;
    
    % Compute the new state using the dynamics.
    x(:,k+1) = x(:,k) + A * dt + B * u_new(:,k) * dt;
    x(:,k+1) = x(:,k+1) + B * u_new(:,k) * sqrt(dt) * sigma * randn;
end