global Horizon;
global time;
global visualizing_bundles;
global num_iter;

clear all;
close all;

visualizing_bundles = 1;
bundle_iters = 20;
x_trajs = zeros(bundle_iters, 2, Horizon);
costs = zeros(bundle_iters, num_iter);

for bundle_iter=1:bundle_iters
   main;
   x_trajs(bundle_iter, :, :) = x_traj;
   costs(bundle_iter,:) = Cost;
end

figure(1);
subplot(3,1,1)
hold on
plot(time,p_target(1,1)*ones(1,Horizon), 'r--','linewidth',2)
for bundle_iter = 1:bundle_iters
    plot(time,reshape(x_trajs(bundle_iter, 1,:), [1, Horizon]),'linewidth',1);  
end
title('Theta','fontsize',20); 
xlabel('Time in sec','fontsize',20)
hold off;
grid;


subplot(3,1,2);
hold on;
plot(time,p_target(2,1)*ones(1,Horizon),'r--','linewidth',2)
for bundle_iter = 1:bundle_iters
    plot(time,reshape(x_trajs(bundle_iter, 2,:), [1, Horizon]),'linewidth',1);  
end
title('Theta dot','fontsize',20);
xlabel('Time in sec','fontsize',20)
hold off;
grid;

subplot(3,1,3);
hold on
for bundle_iter = 1:bundle_iters
    plot(costs(bundle_iter, :),'linewidth',1);
end
xlabel('Iterations','fontsize',20)
title('Cost','fontsize',20);
pos1 = get(gcf,'Position'); % get position of Figure(1) 
set(gcf,'Position', pos1 - [pos1(3)/2,0,0,0]) % Shift position of Figure(1) 
visualizing_bundles = 0;