function [A,B] = fnState_And_Control_Transition_Matrices(x,u,du,dt, env_params)

A = zeros(2,2);
B = zeros(2,1);

% Fx
A(1,1) = 0;
A(1,2) = 1;
A(2,1) = -(env_params.g / env_params.l1) * cos(x(1,1));
A(2,2) = -(env_params.b1 / env_params.I1);

% Fu
B(1,1) = 0;
B(2,1) = 1 / env_params.I1;




