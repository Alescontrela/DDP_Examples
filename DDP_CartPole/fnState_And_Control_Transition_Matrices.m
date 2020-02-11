function [A,B] = fnState_And_Control_Transition_Matrices(x,u,du,dt)

global Fx;
global Fu;

A = Fx(u(1), x(3,1), x(4,1));
B = Fu(x(3,1));
