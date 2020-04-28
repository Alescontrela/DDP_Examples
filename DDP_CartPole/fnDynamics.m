function [dynamics] = fnDynamics(mc, mp, l, g)
x = sym('x', [1 4]); % State vector.
u = sym('u');

den = (mc + mp * sin(x(3)) ^ 2);
Fa = [
    x(2);
    (mp * sin(x(3)) * (l * (x(4) ^ 2) + g * cos(x(3)))) / den;
    x(4);
    (- mp * l * (x(4) ^ 2) * cos(x(3)) * sin(x(3)) - (mc + mp) * g * sin(x(3))) / (l * den)
];

Fb = [0; u / den; 0; (-u * cos(x(3))) / (l * den)];

% System dynamics F(x,u).
F = Fa + Fb;

dynamics = struct();
dynamics.F = matlabFunction(F);
dynamics.Fx = matlabFunction(jacobian(F, x));
dynamics.Fu = matlabFunction(jacobian(F, u));
dynamics.Fb = matlabFunction(Fb); % Used to compute added noise due to stochastic controls.
end