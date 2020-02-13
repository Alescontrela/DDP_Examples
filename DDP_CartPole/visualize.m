global mc;
global mp;
global l;
global g;
global Horizon;
global time;
global p_target;
global dt;

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
xlabel('Iterations','fontsize',14, 'Interpreter', 'latex')
title('Residuals','fontsize',20, 'Interpreter', 'latex');
pos1 = get(gcf,'Position'); % get position of Figure(1) 
set(gcf,'Position', pos1 - [pos1(3)/2,0,0,0]) % Shift position of Figure(1) 
% set(gca, 'YScale', 'log')
hold off;
grid;

subplot(3,2,6);hold on
plot(Cost,'linewidth',2); 
xlabel('Iterations','fontsize',14, 'Interpreter', 'latex')
title('Cost (log scale)','fontsize',20, 'Interpreter', 'latex');
pos1 = get(gcf,'Position'); % get position of Figure(1) 
set(gcf,'Position', pos1 - [pos1(3)/2,0,0,0]) % Shift position of Figure(1) 
set(gca, 'YScale', 'log')
hold off;
grid;

%% Animate Cart Pole Trajectory.
fh = figure('Renderer', 'painters', 'Position', [10 10 2560 1600], ...
       'NumberTitle', 'off', 'Name', 'Cart Pole Animation');
pos2 = get(gcf,'Position');  % get position of Figure(2) 
set(gcf,'Position', pos2 + [pos1(3)/2,0,0,0]) % Shift position of Figure(2)

cart_w = 0.2;
cart_h = 0.1;

min_x = min(x_traj(1, :));
max_x = max(x_traj(1, :));

lims = [min_x - 2 * l, max_x + 2 * l, - 1.2 * l, 1.5 * l];

h1 = rectangle('Position', [xo(1) - cart_w / 2, 0, cart_w, cart_h],...
               'Curvature', 0.2, 'FaceColor',[0 .5 .5]);

x = [xo(1), xo(1) + l * sin(xo(3))];
y = [cart_h / 2, cart_h / 2 - l * cos(xo(3))];

hold on;
h2 = plot(x, y, '-o', 'MarkerSize', 10, 'MarkerFaceColor', 'black',...
    'LineWidth', 2, 'Color', [0, 0, 0]); 
plot([lims(1), lims(2)], [0, 0], 'k');

if (Q_f(1,1) ~= 0)
    rectangle('Position', [p_target(1) - cart_w / 2, 0, cart_w, cart_h], 'LineStyle', '--',...
               'Curvature', 0.2);
    plot([p_target(1), p_target(1) + l * sin(p_target(3))],...
        [cart_h / 2, cart_h / 2 - l * cos(p_target(3))], 'k--', 'LineWidth', 4);    
else
    h3 = plot([p_target(1), p_target(1) + l * sin(p_target(3))],...
        [cart_h / 2, cart_h / 2 - l * cos(p_target(3))], 'k--', 'LineWidth', 4);
end

hold off;

% t = text(-1.5 * l1, 1.5 * l1, 'Time: 0\Theta: 0');

axis(lims);
axis square
axis equal
% alpha scaled

x_dot_max = max(abs(x_traj(2, :)));

% Save video.
save_video = false;

if (save_video)
    video_filepath = 'cart_pole_ddp';
    myVideo = VideoWriter(video_filepath, 'MPEG-4'); %open video file
    myVideo.FrameRate = round(1 / dt);  %can adjust this, 5 - 10 works well for me
    myVideo.Quality = 99;
    open(myVideo)
end

for i = 1:length(x_traj)
  x = x_traj(1, i);
  x_dot_abs = abs(x_traj(2, i));
  theta = x_traj(3, i);
  
  gc = x_dot_abs / x_dot_max;
  bc = 1. - x_dot_abs / x_dot_max;
  rc = i / length(x_traj) ;
  
%   t.String = sprintf('Time: %.2fs\nTheta: %.2frad', time(i), theta);
  h1.Position = [x - cart_w / 2, 0, cart_w, cart_h];
  hold on;
  h2.XData = [x, x + l * sin(theta)];
  h2.YData = [cart_h / 2, cart_h / 2 - l * cos(theta)];
  plot(h2.XData(2), h2.YData(2), '-mo', 'LineWidth',2, 'MarkerEdgeColor',[rc, gc, bc], 'MarkerSize', 3, 'MarkerFaceColor', [rc, gc, bc]);
  axis(lims);
  
  if (Q_f(1,1) == 0)        
      h3.XData = [x, x + l * sin(p_target(3))];
      h3.YData = [cart_h / 2, cart_h / 2 - l * cos(p_target(3))];
  end
  
  hold off;  
  
  drawnow()  
  
  if (save_video)
      frame = getframe(gcf); %get frame
      writeVideo(myVideo, frame);
  end
  
  pause(dt)
end

if (save_video)
    close(myVideo)
end