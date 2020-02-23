function  [l0,l_x,l_xx,l_u,l_uu,l_ux] = fnCost(x, u, k,R,dt)

global mus;
global sigmas;

l0 = u' *R *u;
l_x =zeros(4,1);
l_xx = zeros(4,4);
l_u = R * u;
l_uu = R;
l_ux = zeros(1,4);

x_pos = [x(1); 0; 0; 0];

for i = 1:1:length(mus)
    mu = [mus(i); 0; 0; 0];
    sigma = zeros(4,4);
    sigma(1,1) = sigmas(i);
    
    l_obs = -0.5 * (x_pos - mu)' * sigma * (x_pos - mu);
    l_obsx = -sigma * (x_pos - mu);
    l_obsxx = -sigma;
        
    l0 = l0 + l_obs;
    l_x = l_x + l_obsx;
    l_xx = l_xx + l_obsxx;
end
end