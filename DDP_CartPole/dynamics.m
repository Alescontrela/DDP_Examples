global mc;
global mp;
global l;
global g;

% Defined in this file.
global F;
global Fx;
global Fu;
global Fb; % Used to compute added noise due to stochastic controls.

x = sym('x', [1 4]); % State vector.
u = sym('u');

den = (mc + mp * sin(x(3)) ^ 2);
Fa = [
    x(2);
    (mp * sin(x(3)) * (l * (x(4) ^ 2) + g * cos(x(3)))) / den;
    x(4);
    (- mp * l * (x(4) ^ 2) * cos(x(3)) * sin(x(3)) - (mc + mp) * g * sin(x(3))) / (l * den)
];

Fb = [
    0;
    u / den;
    0;
    (-u * cos(x(3))) / (l * den)
];

% System dynamics F(x,u).
F = Fa + Fb;

F = matlabFunction(F)
Fx = matlabFunction(jacobian(F, x))
Fu = matlabFunction(jacobian(F, u))
Fb = matlabFunction(Fb)