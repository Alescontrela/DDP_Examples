function [dynamics] = fnDynamics(m, Ixx, Iyy, Izz, l, kt, g)
x = sym('x', [1 12]); % State vector.
u = sym('u', [1 4]);  % Control vector.

% Rotation matrix from inertial frame to the body frame.
R_11 = cos(x(8)) * cos(x(9));
R_12 = cos(x(8)) * sin(x(9));
R_13 = -sin(x(8));
R_21 = sin(x(8)) * sin(x(7)) * cos(x(9)) - cos(x(7)) * sin(x(9));
R_22 = sin(x(8)) * sin(x(7)) * sin(x(9)) + cos(x(7)) * cos(x(9));
R_23 = sin(x(7)) * cos(x(8));
R_31 = sin(x(7)) * sin(x(9)) + cos(x(7)) * sin(x(8)) * cos(x(9));
R_32 = sin(x(8)) * sin(x(9)) * cos(x(7)) - sin(x(7)) * sin(x(9));
R_33 = cos(x(8)) * cos(x(7));

R = [R_11, R_12, R_13; R_21, R_22, R_23; R_31, R_32, R_33];

 % Matrix converting body rates to euler rates.
br2ur = [1, tan(x(8)) * sin(x(7)), tan(x(8)) * cos(x(7));
         0, cos(x(7)),             -sin(x(7));
         0, sin(x(7)) / cos(x(8)), cos(x(7)) / cos(x(8))]; 
     
Fa = [
	x(4); % x_dot
	x(5); % y_dot
	x(6); % z_dot
	0;    % x_ddot
	0;    % y_ddot
	-g;   % z_ddot
    br2ur(1,1) * x(10) + br2ur(1, 2) * x(11) + br2ur(1, 3) * x(12);
    br2ur(2,1) * x(10) + br2ur(2, 2) * x(11) + br2ur(2, 3) * x(12);
    br2ur(3,1) * x(10) + br2ur(3, 2) * x(11) + br2ur(3, 3) * x(12);
	- (Izz - Iyy) * x(11) * x(12) / Ixx;    
	(Izz - Ixx) * x(10) * x(12) / Iyy;
	0
];

Fb = [
	0;
	0;
	0;
    R_13 * (u(1) + u(2) + u(3) + u(4)) / m;
    R_23 * (u(1) + u(2) + u(3) + u(4)) / m;
    R_33 * (u(1) + u(2) + u(3) + u(4)) / m;
	0;
	0;
	0;
	(sqrt(2) / 2) * (u(1) + u(3) - u(2) - u(4)) * l / Ixx;
	(sqrt(2) / 2) * (u(3) + u(4) - u(1) - u(2)) * l / Iyy;
	kt * (u(1) + u(4) - u(2) - u(3)) / Izz;
];

% System dynamics F(x,u).
F = Fa + Fb;

dynamics = struct();
dynamics.F = matlabFunction(F);
dynamics.Fx = matlabFunction(jacobian(F, x));
dynamics.Fu = matlabFunction(jacobian(F, u));
dynamics.Fb = matlabFunction(Fb); % Used to compute added noise due to stochastic controls.
end