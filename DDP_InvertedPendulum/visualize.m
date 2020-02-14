figure(1);
subplot(3,1,1)
hold on
plot(time,x_traj(1,:),'linewidth',4);  
plot(time,p_target(1,1)*ones(1,Horizon),'red','linewidth',4)
title('Theta','fontsize',20); 
xlabel('Time in sec','fontsize',20)
hold off;
grid;


subplot(3,1,2);
hold on;
plot(time,x_traj(2,:),'linewidth',4); 
plot(time,p_target(2,1)*ones(1,Horizon),'red','linewidth',4)
title('Theta dot','fontsize',20);
xlabel('Time in sec','fontsize',20)
hold off;
grid;

subplot(3,1,3);hold on
plot(Cost,'linewidth',2); 
xlabel('Iterations','fontsize',20)
title('Cost','fontsize',20);
pos1 = get(gcf,'Position'); % get position of Figure(1) 
set(gcf,'Position', pos1 - [pos1(3)/2,0,0,0]) % Shift position of Figure(1) 
hold off;
grid;

% Visualize inverted pendelum trajectory.
fh = figure();
pos2 = get(gcf,'Position');  % get position of Figure(2) 
set(gcf,'Position', pos2 + [pos1(3)/2,0,0,0]) % Shift position of Figure(2)

x = [0, l1 * sin(xo(1))];
y = [0, -l1 * cos(xo(1))];

h1 = plot(x, y, '-o', 'MarkerSize', 10, 'MarkerFaceColor', 'black',...
    'LineWidth', 2, 'Color', [0, 0, 0]);
% hold on;

t = text(-1.5 * l1, 1.5 * l1, 'Time: 0\Theta: 0');
axis([-2 * l1, 2 * l1, -2 * l1, 2 * l1])
axis square
alpha scaled

theta_dot_max = max(abs(x_traj(2, :)));

% Save video.
save_video = false;

if (save_video)
    video_filepath = 'ip_ddp';
    myVideo = VideoWriter(video_filepath, 'MPEG-4'); %open video file
    myVideo.FrameRate = round(1 / dt);  %can adjust this, 5 - 10 works well for me
    myVideo.Quality = 99;
    open(myVideo)
end

for i = 1:length(x_traj)
  theta = x_traj(1, i);
  theta_dot = abs(x_traj(2, i));
  
  gc = 1 - i / length(x_traj);
  bc = theta_dot / theta_dot_max;
  rc = i / length(x_traj);
  
  t.String = sprintf('Time: %.2fs\nTheta: %.2frad', time(i), theta);
  h1.XData = [0, l1 * sin(theta)];
  h1.YData = [0, -l1 * cos(theta)];
  
  hold on;
  plot(h1.XData(2), h1.YData(2), '-mo', 'LineWidth',2, 'MarkerEdgeColor',...
      [rc, gc, bc], 'MarkerSize', 3, 'MarkerFaceColor', [rc, gc, bc]);
%   h2 = plot(l1 * sin(theta), -l1 * cos(theta), 'bo', 'MarkerSize',...
%        (abs(theta_dot) / theta_dot_max) + 1e-4);
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