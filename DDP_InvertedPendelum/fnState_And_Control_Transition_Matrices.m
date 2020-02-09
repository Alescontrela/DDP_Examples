function [A,B] = fnState_And_Control_Transition_Matrices(x,u,du,dt)

global b1;
global I1;
global g;
global l1;

A = zeros(2,2);
B = zeros(2,1);

% Fx
A(1,1) = 0;
A(1,2) = 1;
A(2,1) = -(g / l1) * cos(x(1,1));
A(2,2) = -(b1 / I1);

% Fu
B(1,1) = 0;
B(2,1) = 1 / I1;




