function [u, cost] = fnDDP(x,num_iter, dt, Q_f, R, p_target, gamma,...
    sigma, x_dim, u_dim, u_init, dynamics)

global Horizon;

xo = x;
x_traj = zeros(x_dim,Horizon); % Initial trajectory.
if nargin < 11
 u_k = zeros(u_dim,Horizon-1); % Initial control.
else
 u_k = u_init;
end
du_k = zeros(u_dim,Horizon-1); % Initial control variation.
    
for k = 1:num_iter
    % Preallocate cost memory.
    q0 = zeros(Horizon-1);
    q_k = zeros(x_dim, Horizon-1);
    Q_k = zeros(x_dim, x_dim, Horizon-1);
    r_k = zeros(u_dim, Horizon-1);
    R_k = zeros(u_dim, u_dim, Horizon-1);
    P_k = zeros(u_dim, x_dim, Horizon-1);
    A = zeros(x_dim, x_dim, Horizon-1);
    B = zeros(x_dim, u_dim, Horizon-1);
    for  j = 1:(Horizon-1)    
        [l0,l_x,l_xx,l_u,l_uu,l_ux] = fnCost(x_traj(:,j), u_k(:,j),...
            j,R,dt);
        % Compute loss function gradients for the current timestep.
        % Quadratic Approximations of the cost function.
        q0(j) = dt * l0;        % L.
        q_k(:,j) = dt * l_x;    % Lx.
        Q_k(:,:,j) = dt * l_xx; % Lxx.
        r_k(:,j) = dt * l_u;    % Lu.
        R_k(:,:,j) = dt * l_uu; % Luu.
        P_k(:,:,j) = dt * l_ux; % Lux.

        % Linearize the dynamics using first order taylor expansion.
        [fx,fu] = fnState_And_Control_Transition_Matrices(x_traj(:,j),...
            u_k(:,j),du_k(:,j),dt, dynamics);
        A(:,:,j) = eye(x_dim,x_dim) + fx * dt;
        B(:,:,j) = fu * dt;  
    end

    % Preallocate value function memory.
    Vxx = zeros(x_dim,x_dim,Horizon);
    Vx = zeros(x_dim, Horizon);
    V = zeros(1, Horizon);
 
    % Compute value function at final timestep, its gradient, and its
    % jacobian.
    Vxx(:,:,Horizon)= Q_f;
    Vx(:,Horizon) = Q_f * (x_traj(:,Horizon) - p_target); 
    V(Horizon) = 0.5 * (x_traj(:,Horizon) - p_target)' * Q_f *...
        (x_traj(:,Horizon) - p_target);
    
    % Backpropagation of the Value Function.
    for j = (Horizon-1):-1:1
        % Quu = Luu + B^T * Vxx * B
        H = R_k(:,:,j) + B(:,:,j)' * Vxx(:,:,j+1) * B(:,:,j);
        % Qux = Lux + B^T * Vxx * A
        G = P_k(:,:,j) + B(:,:,j)' * Vxx(:,:,j+1) * A(:,:,j);
        % Qu = Lu + B^T * Vx
        g_ = r_k(:,j) +  B(:,:,j)' * Vx(:,j+1);

        inv_H = inv(H); % Quu^-1
        L_k(:,:,j)= - inv_H * G; % Feedback term = -Quu^-1 * Qux.
        l_k (:,j) = - inv_H *g_; % Feedforward term = -Quu^-1 * Qu.

        % Vxx = (Lxx + A^T * Vxx * A) + (-Qxu * Quu^-1) * Quu *
        % (-Quu^-1 * Qux) + (-Qxu * Quu^-1 * Qux) + (Qxu * -Quu^-1 * Qux) 
        Vxx(:,:,j) = Q_k(:,:,j)+ A(:,:,j)' * Vxx(:,:,j+1) * A(:,:,j) +...
            L_k(:,:,j)' * H * L_k(:,:,j) + L_k(:,:,j)' * G + G' * L_k(:,:,j);
        % Vx = (Lx + A^T * Vx') + (-Qxu * Quu^-1 * Qu) + 
        % (Qxu * -Quu^-1 * Qu) + (-Qxu * Quu^-1 * Qu * Quu * -Quu^-1 * Qu)
        Vx(:,j)= q_k(:,j) +  A(:,:,j)' *  Vx(:,j+1) + L_k(:,:,j)' * g_...
            + G' * l_k(:,j) + L_k(:,:,j)'*H * l_k(:,j);
        % V = L + V' + (0.5 * Qu^T * Quu^-1 * Quu * Quu^-1 * Qu) +
        % (-Qu^T * Quu^-1 * Qu)
        V(:,j) = q0(j) + V(j+1) + 0.5 *  l_k (:,j)' * H * l_k (:,j) +...
            l_k (:,j)' * g_;
    end

    % Preallocate control variation memory.
    u_new = zeros(u_dim, Horizon-1);
    dx = zeros(x_dim,1);
    for i=1:(Horizon-1)
        % Find the controls.
        du = l_k(:,i) + L_k(:,:,i) * dx;
        dx = A(:,:,i) * dx + B(:,:,i) * du;  
        u_new(:,i) = u_k(:,i) + gamma * du;
    end
    u_k = u_new;

    % Create new rollout.
    [x_traj] = fnSimulate(xo,u_new,Horizon,dt,sigma, dynamics);
end
cost = fnCostComputation(x_traj,u_k,p_target,dt,Q_f,R);
u = u_k;
end