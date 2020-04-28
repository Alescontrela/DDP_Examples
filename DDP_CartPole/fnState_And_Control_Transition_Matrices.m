function [A,B] = fnState_And_Control_Transition_Matrices(x,u,du,dt, dynamics)
    A = dynamics.Fx(u(1), x(3,1), x(4,1));
    B = dynamics.Fu(x(3,1));
end