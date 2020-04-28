function  [l0,l_x,l_xx,l_u,l_uu,l_ux] = fnCost(x, u, k,R,dt)

x_dim = size(x, 1);
u_dim = size(u, 1);

l0 = u' *R *u;
l_x =zeros(x_dim,1);
l_xx = zeros(x_dim,x_dim);
l_u = R * u;
l_uu = R;
l_ux = zeros(u_dim,x_dim);

end