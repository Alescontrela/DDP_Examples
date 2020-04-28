%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%    MPC-DDP Inverted Pendulum     %%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%  Course: Robotics and Autonomy   %%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%  AE4803  Spring  2020            %%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%  Author: Alejandro Escontrela    %%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

global m1;
global I1;
global b1;
global g;
global l1;
global Horizon;
global time;

% Environment parameters.
m1 = 1.4; % Link mass in Kg.
g = 9.8; % Gravity in m/s^2.
b1 = 0.5; % Friction coefficient.
l1 = 1.0; % Link length in meters.
I1 = m1 * l1^2; % Inertia in Kg*m^2.

env_params_nominal = struct('g', g, 'b1', b1, 'l1',  l1, 'I1', I1);

% "Actual" environment params, which are affected by measurement error,
% which is modeled as gaussian distributed noise.
param_sigma = 0.5;
env_params_noisy = struct('g', g + g * param_sigma * randn,...
                          'b1', b1 + b1 * param_sigma * randn,...
                          'l1', l1 + l1 * param_sigma * randn,...
                          'I1', I1 + I1 * param_sigma * randn);

% Solver parameters.
Horizon = 50; % Time Horizon.
num_iter = 10; % Number of Iterations
dt = 0.01; % Discretization.

% Costs.
Q_f = zeros(2,2); % State cost. 2x2 since state is 2-dimensional.
Q_f(1,1) = 6400;
Q_f(2,2) = 400;

R = 100 * eye(1,1); % Control cost. 1x1 since control is 1-dimensional.

% Initialize solution.
xo = zeros(2,1);
xo(1,1) = 0; % Initial theta.
xo(2,1) = 0; % Initial theta_dot.

p_target(1,1) = pi; % Target theta.
p_target(2,1) = 0.0; % Target theta_dot.

% Add noise.
sigma_nominal = 0.1;
sigma_real = 0.15;

% Learning Rate
gamma = 0.5;

% Run the MPC.
x = xo;
Cost = []; % Cost history.
x_traj = []; % Initial trajectory
u_init = zeros(1,Horizon-1);

i = 1;
max_num_iters = 600;

while 1
    [u, cost] = fnDDP(x,num_iter, dt, Q_f, R, p_target, gamma, sigma_nominal, u_init, env_params_nominal);
    u_init = u;
    Cost = [Cost cost];
    [x_new] = fnSimulate(x,u,Horizon,dt,sigma_real, env_params_noisy, 2);
    x = x_new(:,2);
    x_traj = [x_traj x];
    
    if (norm(x - p_target) < 1e-3) || (i > max_num_iters)
        break;
    end
    
    if mod(i, 100) == 0
        fprintf('MPC Iteration %i,  Current Cost = %e \n',i,...
            norm(x - p_target));
    end
    
    i = i + 1;
end

Horizon = length(x_traj);

time = zeros(1, Horizon);
time(1)=0;
for i= 2:(Horizon - 1)
    time(i) =time(i-1) + dt;  
end

visualize
