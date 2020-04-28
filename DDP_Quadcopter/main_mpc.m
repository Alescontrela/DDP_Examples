%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%        MPC-DDP Cart Pole         %%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%  Course: Robotics and Autonomy   %%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%  AE4803  Spring  2020            %%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%  Author: Alejandro Escontrela    %%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear;
close all;

global Horizon;
global time;
global p_target
global dt;

% Environment parameters.
mc = 1.0;  % Cart mass in Kg.
mp = 0.01; % Pole mass in Kg.
l = 0.25;  % Length of the pole in m.
g = 9.8;   % Gravity in m/s^2.

% Obtain expressions for F, Fx, Fu.
dynamics_nominal = fnDynamics(mc, mp, l, g);

% Environment parameters with measurement error.
dynamics_sigma = 0.2;

mc_noisy = -1;
while mc_noisy <= 0
    mc_noisy = mc + mc * dynamics_sigma * randn;
end

mp_noisy = -1;
while mp_noisy <= 0
    mp_noisy = mp + mp * dynamics_sigma * randn;
end

l_noisy = -1;
while l_noisy <= 0
    l_noisy = l + l * dynamics_sigma * randn;
end

g_noisy = -1;
while g_noisy <= 0
    g_noisy = g + g * dynamics_sigma * randn;
end
dynamics_actual = fnDynamics(mc_noisy, mp_noisy, l_noisy, g_noisy);

% Solver parameters.
Horizon = 50;  % Time Horizon.
num_iter = 10; % Number of Iterations
dt = 0.01;     % Discretization.

% Costs.
Q_f = zeros(4,4); % State cost. 4x4 since state is 4-dimensional.
Q_f(1,1) = 2000;     % X position cost.
Q_f(2,2) = 100;   % X velocity cost.
Q_f(3,3) = 2000;  % Pole angle cost.
Q_f(4,4) = 100;   % Pole angular velocity cost.

if ~(all(eig(Q_f) >= 0))
    error('Cost matrix Q_f not positive semi-definite.')
end

R = 10 * eye(1,1); % Control cost. 1x1 since control is 1-dimensional.

% Initialize solution.
% State represented as [x, x_dot, theta, theta_dot].
xo = zeros(4,1);
x_dim = length(xo);
u_dim = 1;

% Goal state:
p_target = zeros(x_dim, 1);
p_target(1,1) = 0.0; % Target x.
p_target(2,1) = 0.0;  % Target x_dot.
p_target(3,1) = pi;   % Target theta.
p_target(4,1) = 0.0;  % Target theta_dot.

% Add noise.
sigma_nominal = 0.0;
sigma_real = 0.1;

% Learning Rate
gamma = 0.5;

% Run the MPC.
x = xo;
residuals = []; % Residual history.
Cost = [];      % Cost history.
x_traj = [];    % Initial trajectory
u_init = zeros(u_dim, Horizon-1);

i = 1;
max_num_iters = 600;
while 1
    [u, cost] = fnDDP(x,num_iter, dt, Q_f, R, p_target, gamma,...
      sigma_nominal, x_dim, u_dim, u_init, dynamics_nominal);
    u_init = u;
    Cost = [Cost cost];
    [x_new] = fnSimulate(x,u,Horizon,dt,sigma_real, dynamics_actual, 2);
    x = x_new(:,2);
    x_traj = [x_traj x];
    if i ~= 1
        residuals = [residuals abs(Cost(:, i - 1) - Cost(:, i))];
    end
    if mod(i, 100) == 0
        fprintf('MPC Iteration %i,  Current Cost = %e \n',i,...
            norm(x - p_target));
    end
    
    if (norm(x - p_target) < 1e-3) || (i > max_num_iters)
        break;
    end
    i = i + 1;
end

Horizon = length(x_traj);
time = zeros(1, Horizon);
for i= 2:(Horizon - 1)
    time(i) =time(i-1) + dt;
end

visualize