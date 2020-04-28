global Horizon;
global time;
global p_target;
global dt;

%% Plot Convergence Information.
figure('Renderer', 'painters', 'Position', [10 10 1000 600], ...
    'NumberTitle', 'off', 'Name', 'Cart Pole Convergence')

subplot(4,3,1)
hold on
plot(time,x_traj(1,:),'linewidth',4);
plot(time,p_target(1,1)*ones(1,Horizon),'red','linewidth',4)
title('$x (m.)$','fontsize',20, 'Interpreter', 'latex');
xlabel('$t$','fontsize',14, 'Interpreter', 'latex');
hold off;
grid;


subplot(4,3,2);
hold on;
plot(time,x_traj(2,:),'linewidth',4);
plot(time,p_target(2,1)*ones(1,Horizon),'red','linewidth',4)
title('$y (m.)$','fontsize',20, 'Interpreter', 'latex');
xlabel('$t$','fontsize',14, 'Interpreter', 'latex');
hold off;
grid;

subplot(4,3,3);
hold on;
plot(time,x_traj(3,:),'linewidth',4);
plot(time,p_target(3,1)*ones(1,Horizon),'red','linewidth',4)
title('$z (m.)$','fontsize',20, 'Interpreter', 'latex');
xlabel('$t$','fontsize',14, 'Interpreter', 'latex');
hold off;
grid;

subplot(4,3,4);
hold on;
plot(time,x_traj(4,:),'linewidth',4);
plot(time,p_target(4,1)*ones(1,Horizon),'red','linewidth',4)
title('$\dot{x}$','fontsize',20, 'Interpreter', 'latex');
xlabel('$t$','fontsize',14, 'Interpreter', 'latex');
hold off;
grid;

subplot(4,3,5);
hold on;
plot(time,x_traj(5,:),'linewidth',4);
plot(time,p_target(5,1)*ones(1,Horizon),'red','linewidth',4)
title('$\dot{y}$','fontsize',20, 'Interpreter', 'latex');
xlabel('$t$','fontsize',14, 'Interpreter', 'latex');
hold off;
grid;

subplot(4,3,6);
hold on;
plot(time,x_traj(6,:),'linewidth',4);
plot(time,p_target(6,1)*ones(1,Horizon),'red','linewidth',4)
title('$\dot{z}$','fontsize',20, 'Interpreter', 'latex');
xlabel('$t$','fontsize',14, 'Interpreter', 'latex');
hold off;
grid;

subplot(4,3,7);
hold on;
plot(time,x_traj(7,:),'linewidth',4);
plot(time,p_target(7,1)*ones(1,Horizon),'red','linewidth',4)
title('$\phi$','fontsize',20, 'Interpreter', 'latex');
xlabel('$t$','fontsize',14, 'Interpreter', 'latex');
hold off;
grid;

subplot(4,3,8);
hold on;
plot(time,x_traj(8,:),'linewidth',4);
plot(time,p_target(8,1)*ones(1,Horizon),'red','linewidth',4)
title('$\theta$','fontsize',20, 'Interpreter', 'latex');
xlabel('$t$','fontsize',14, 'Interpreter', 'latex');
hold off;
grid;

subplot(4,3,9);
hold on;
plot(time,x_traj(9,:),'linewidth',4);
plot(time,p_target(9,1)*ones(1,Horizon),'red','linewidth',4)
title('$\psi$','fontsize',20, 'Interpreter', 'latex');
xlabel('$t$','fontsize',14, 'Interpreter', 'latex');
hold off;
grid;

subplot(4,3,10);
hold on;
plot(time,x_traj(10,:),'linewidth',4);
plot(time,p_target(10,1)*ones(1,Horizon),'red','linewidth',4)
title('$p$','fontsize',20, 'Interpreter', 'latex');
xlabel('$t$','fontsize',14, 'Interpreter', 'latex');
hold off;
grid;

subplot(4,3,11);
hold on;
plot(time,x_traj(11,:),'linewidth',4);
plot(time,p_target(11,1)*ones(1,Horizon),'red','linewidth',4)
title('$q$','fontsize',20, 'Interpreter', 'latex');
xlabel('$t$','fontsize',14, 'Interpreter', 'latex');
hold off;
grid;

subplot(4,3,12);
hold on;
plot(time,x_traj(12,:),'linewidth',4);
plot(time,p_target(12,1)*ones(1,Horizon),'red','linewidth',4)
title('$r$','fontsize',20, 'Interpreter', 'latex');
xlabel('$t$','fontsize',14, 'Interpreter', 'latex');
hold off;
grid;


% subplot(4,3,5);hold on
% plot(residuals,'linewidth',2);
% xlabel('Iterations','fontsize',14, 'Interpreter', 'latex')
% title('Residuals','fontsize',20, 'Interpreter', 'latex');
% pos1 = get(gcf,'Position'); % get position of Figure(1)
% set(gcf,'Position', pos1 - [pos1(3)/2,0,0,0]) % Shift position of Figure(1)
% % set(gca, 'YScale', 'log')
% hold off;
% grid;
%
% subplot(4,3,6);hold on
% plot(Cost,'linewidth',2);
% xlabel('Iterations','fontsize',14, 'Interpreter', 'latex')
% title('Cost (log scale)','fontsize',20, 'Interpreter', 'latex');
pos1 = get(gcf,'Position'); % get position of Figure(1)
% set(gcf,'Position', pos1 - [pos1(3)/2,0,0,0]) % Shift position of Figure(1)
% set(gca, 'YScale', 'log')
% hold off;
% grid;

%% Animate Quadcopter Trajectory.
fh = figure('Renderer', 'painters', 'Position', [10 10 2560 1600], ...
    'NumberTitle', 'off', 'Name', 'Cart Pole Animation');
pos2 = get(gcf,'Position');  % get position of Figure(2)
set(gcf,'Position', pos2 + [pos1(3)/2,0,0,0]) % Shift position of Figure(2)

l = 0.17;

padding = 0.3;
x_lim = [min(x_traj(1, :)) - padding, max(x_traj(1, :)) + padding];
y_lim = [min(x_traj(2, :)) - padding, max(x_traj(2, :)) + padding];
z_lim = [min(x_traj(3, :)) - padding, max(x_traj(3, :)) + padding];

% Draw the ground;
fill3([x_lim(1), x_lim(1), x_lim(2), x_lim(2)], [y_lim(1), y_lim(2), y_lim(2), y_lim(1)], 2 * [-l, -l, -l, -l], [0.86, 0.86, 0.86]);
hold on;

x = xo(1);
y = xo(2);
z = xo(3);

phi = xo(7);
theta = xo(8);
psi = xo(9);

pos = [x, y, z];
R = (rotx(phi * 180 / pi) * roty(theta * 180 / pi) * rotz(psi * 180 / pi))';
[poly_x, poly_y, poly_z] = drone_poly_coords(R, l, pos);
h = fill3(poly_x, poly_y, poly_z, [0 1 1]);
view([0 15])

l1 = [l, 0, 0];
l2 = [0, l, 0];
l3 = [0, 0, l];

v1 = pos + (R * l1')';
v2 = pos + (R * l2')';
v3 = pos + (R * l3')';

pts1 = [pos; v1];
pts2 = [pos; v2];
pts3 = [pos; v3];


h1 = plot3(pts1(:,1), pts1(:,2), pts1(:,3), 'r');
h2 = plot3(pts2(:,1), pts2(:,2), pts2(:,3), 'g');
h3 = plot3(pts3(:,1), pts3(:,2), pts3(:,3), 'b');


xlim(x_lim);
ylim(y_lim);
zlim(z_lim);
% axis square
axis equal

% Save video.
save_video = true;

if (save_video)
    video_filepath = 'Quadcopter_ddp';
    myVideo = VideoWriter(video_filepath, 'MPEG-4'); %open video file
    myVideo.FrameRate = round(3 / dt);  %can adjust this, 5 - 10 works well for me
    myVideo.Quality = 99;
    open(myVideo)
end

max_speed = 0;
for i=1:length(x_traj)
    speed = norm([x_traj(4, i), x_traj(5, i), x_traj(6, i)]);
    if (speed > max_speed)
        max_speed = speed;
    end
end

for i = 1:length(x_traj)
    x = x_traj(1, i);
    y = x_traj(2, i);
    z = x_traj(3, i);
    
    speed = norm([x_traj(4, i), x_traj(5, i), x_traj(6, i)]);
    
    gc = speed / max_speed;
    bc = 1. - speed / max_speed;
    rc = i / length(x_traj) ;
    
    phi = x_traj(7, i);
    theta = x_traj(8, i);
    psi = x_traj(9, i);
    
    pos = [x, y, z];
    R = (rotx(phi * 180 / pi) * roty(theta * 180 / pi) * rotz(psi * 180 / pi))';
    [poly_x, poly_y, poly_z] = drone_poly_coords(R, l, pos);

    h.XData = poly_x;
    h.YData = poly_y;
    h.ZData = poly_z;
    
    pts1 = [pos; pos + (R * l1')'];
    pts2 = [pos; pos + (R * l2')'];
    pts3 = [pos; pos + (R * l3')'];
    
    h1.XData = pts1(:,1);
    h1.YData = pts1(:,2);
    h1.ZData = pts1(:,3);
    
    h2.XData = pts2(:,1);
    h2.YData = pts2(:,2);
    h2.ZData = pts2(:,3);
    
    h3.XData = pts3(:,1);
    h3.YData = pts3(:,2);
    h3.ZData = pts3(:,3);

    hold on;
    plot3(x, y, z, '-mo', 'MarkerEdgeColor', [rc, gc, bc], 'MarkerSize', 3, 'MarkerFaceColor', [rc, gc, bc]);
    
    % Draw coordinate axes.
    scale = 0.2;
    if (mod(i, 5) == 0)
        pts1_perm = [pos; pos + scale * (R * l1')'];
        pts2_perm = [pos; pos + scale * (R * l2')'];
        pts3_perm = [pos; pos + scale * (R * l3')'];
        plot3(pts1_perm(:,1), pts1_perm(:,2), pts1_perm(:,3), 'r');
        plot3(pts2_perm(:,1), pts2_perm(:,2), pts2_perm(:,3), 'g');
        plot3(pts3_perm(:,1), pts3_perm(:,2), pts3_perm(:,3), 'b');
    end
    
    axis equal;
    xlim(x_lim);
    ylim(y_lim);
    zlim(z_lim);
    
    
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

% Visualization Utils.
function [x, y, z] = drone_poly_coords(R, l, pos)

    pts = [
        pos + (R * [l l/5 0]')';
        pos + (R * [l/5 l/5 0]')';
        pos + (R * [l/5 l 0]')';
        pos + (R * [-l/5 l 0]')';
        pos + (R * [-l/5 l/5 0]')';
        pos + (R * [-l l/5 0]')';
        pos + (R * [-l -l/5 0]')';
        pos + (R * [-l/5 -l/5 0]')';
        pos + (R * [-l/5 -l 0]')';
        pos + (R * [l/5 -l 0]')';
        pos + (R * [l/5 -l/5 0]')';
        pos + (R * [l -l/5 0]')';
    ];

    x = pts(:,1)';
    y = pts(:,2)';
    z = pts(:,3)';
end