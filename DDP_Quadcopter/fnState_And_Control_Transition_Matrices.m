function [fx,fu] = fnState_And_Control_Transition_Matrices(x,u, du, dt, dynamics)
    fx = dynamics.Fx(u(1,1), u(2,1), u(3,1), u(4,1),...
        x(7,1), x(8,1), x(10,1), x(11,1), x(12,1));
    fu = dynamics.Fu(x(7,1), x(8,1));
end