%% Plot Convergence Information.
figure('Renderer', 'painters', 'Position', [10 10 1000 600], ...
       'NumberTitle', 'off', 'Name', 'Cart Pole Convergence')

subplot(3,2,1)
hold on
plot(time,x_traj(1,:),'linewidth',4);  
plot(time,p_target(1,1)*ones(1,Horizon),'red','linewidth',4)
title('$X$','fontsize',20, 'Interpreter', 'latex');
xlabel('$t$','fontsize',14, 'Interpreter', 'latex');
hold off;
grid;


subplot(3,2,2);
hold on;
plot(time,x_traj(2,:),'linewidth',4); 
plot(time,p_target(2,1)*ones(1,Horizon),'red','linewidth',4)
title('$\dot{X}$','fontsize',20, 'Interpreter', 'latex');
xlabel('$t$','fontsize',14, 'Interpreter', 'latex');
hold off;
grid;

subplot(3,2,3);
hold on;
plot(time,x_traj(3,:),'linewidth',4); 
plot(time,p_target(3,1)*ones(1,Horizon),'red','linewidth',4)
title('$\theta$','fontsize',20, 'Interpreter', 'latex');
xlabel('$t$','fontsize',14, 'Interpreter', 'latex');
hold off;
grid;

subplot(3,2,4);
hold on;
plot(time,x_traj(4,:),'linewidth',4); 
plot(time,p_target(4,1)*ones(1,Horizon),'red','linewidth',4)
title('$\dot{\theta}$','fontsize',20, 'Interpreter', 'latex');
xlabel('$t$','fontsize',14, 'Interpreter', 'latex');
hold off;
grid;

subplot(3,2,5);hold on
plot(residuals,'linewidth',2); 
xlabel('Iterations','fontsize',20)
title('Residuals','fontsize',14);
pos1 = get(gcf,'Position'); % get position of Figure(1) 
set(gcf,'Position', pos1 - [pos1(3)/2,0,0,0]) % Shift position of Figure(1) 
% set(gca, 'YScale', 'log')
hold off;
grid;

subplot(3,2,6);hold on
plot(Cost,'linewidth',2); 
xlabel('Iterations','fontsize',14)
title('Cost (log scale)','fontsize',14);
pos1 = get(gcf,'Position'); % get position of Figure(1) 
set(gcf,'Position', pos1 - [pos1(3)/2,0,0,0]) % Shift position of Figure(1) 
set(gca, 'YScale', 'log')
hold off;
grid;

%% Animate Cart Pole Trajectory.
% Visualize inverted pendelum trajectory.
% fh = figure();
% pos2 = get(gcf,'Position');  % get position of Figure(2) 
% set(gcf,'Position', pos2 + [pos1(3)/2,0,0,0]) % Shift position of Figure(2)
% 
% x = [0, l1 * sin(xo(1))];
% y = [0, -l1 * cos(xo(1))];
% 
% 
% h1 = plot(x, y, '-o', 'MarkerSize', 10, 'MarkerFaceColor', 'black',...
%     'LineWidth', 2, 'Color', [0, 0, 0]);
% % hold on;
% 
% t = text(-1.5 * l1, 1.5 * l1, 'Time: 0\Theta: 0');
% axis([-2 * l1, 2 * l1, -2 * l1, 2 * l1])
% axis square
% alpha scaled
% 
% theta_dot_max = max(abs(x_traj(1, :)));
% 
% for i = 1:length(x_traj)
%   theta = x_traj(1, i);
%   theta_dot = x_traj(2, i);
%   
%   t.String = sprintf('Time: %.2fs\nTheta: %.2frad', time(i), theta);
%   h1.XData = [0, l1 * sin(theta)];
%   h1.YData = [0, -l1 * cos(theta)];
%   
%   hold on;
%   h2 = plot(l1 * sin(theta), -l1 * cos(theta), 'bo', 'MarkerSize',...
%        (abs(theta_dot) / theta_dot_max) + 1e-4);
%   hold off;
%   
%   drawnow()  
%   pause(dt)
% end