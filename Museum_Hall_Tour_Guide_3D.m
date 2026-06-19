% =========================================================
%  Museum Hall Tour Guide - 3D Realistic Simulation
%  Features:
%   - Full 3D museum hall with walls, floor, ceiling, exhibits
%   - Realistic humanoid visitors (cylinder body + sphere head)
%   - 3D robot model (box body + sensor dome + direction arrow)
%   - Fixed isometric top-down camera view throughout simulation
% =========================================================
clc; clear; close all;

%% ---- Parameters ----------------------------------------
v_base     = 3;
dt         = 0.1;
safe_dist  = 3;
threshold  = 0.5;
sensor_range = 10;

hall_width  = 50;
hall_length = 75;
wall_h      = 8;    % wall height (ft)
floor_z     = 0;
ceil_z      = wall_h;

%% ---- Initial Robot Pose --------------------------------
x = 5; y = 5; theta = pi/4;
v = v_base;

%% ---- Waypoints & Exhibits ------------------------------
waypoints = [9, 9; 20, 30; 40, 30; 60, 40; 70, 10];
exhibits  = {'Entry', 'Exhibit A', 'Exhibit B', 'Exhibit C', 'Exit'};

%% ---- Static Obstacles (pillars) -----------------------
obstacles = [25, 25; 30, 32; 40, 20; 50, 30];

%% ---- Wall Blocks  [x_start, y_start, width, depth] ----
wall_blocks = [
    30,  0,  1, 20;
    15, 30,  1, 20;
     0,  0, hall_length, 1;          % south wall
     0, hall_width-1, hall_length, 1; % north wall
     0,  0,  1, hall_width;          % west wall
    hall_length-1, 0, 1, hall_width  % east wall
];

%% ---- Visitors ------------------------------------------
visitors      = [12,12; 20,40; 35,40; 55,10; 25,15];
num_visitors  = size(visitors, 1);
visitor_speeds = 0.5 + rand(num_visitors,1);
visitor_dirs   = 2*pi*rand(num_visitors,1);

%% ---- Path tracking ------------------------------------
path_x = []; path_y = [];

%% ---- Figure & Axes ------------------------------------
fig = figure('Name','3D Museum Tour Guide','Color','k', ...
             'Position',[50 50 1280 720]);
ax = axes('Parent', fig);
hold(ax,'on');
axis(ax,'equal');
set(ax,'Color',[0.08 0.06 0.10]);           % dark background
set(ax,'XLim',[0 hall_length], 'YLim',[0 hall_width], 'ZLim',[0 ceil_z+2]);
xlabel(ax,'X (ft)'); ylabel(ax,'Y (ft)'); zlabel(ax,'Z (ft)');
title(ax,'3D Autonomous Museum Tour Guide','Color','w','FontSize',14);
set(ax,'XColor','w','YColor','w','ZColor','w','GridColor',[0.3 0.3 0.3]);
grid(ax,'on');
lighting(ax,'gouraud');

% Lights
light(ax,'Position',[hall_length/2, hall_width/2, ceil_z+5],'Style','local','Color',[1 1 0.9]);
light(ax,'Position',[10, 10, ceil_z],'Style','local','Color',[0.6 0.6 1.0]);

%% ---- Draw Static Museum Environment -------------------
draw_museum(ax, hall_length, hall_width, wall_h, wall_blocks, obstacles, waypoints, exhibits);

%% ---- Fixed Isometric Camera ----------------------------
% Camera sits at a fixed 45-degree diagonal corner, high up,
% looking down at the centre of the museum — stays here forever.
iso_pos    = [hall_length/2 - 52, hall_width/2 - 52, 60];
iso_target = [hall_length/2,       hall_width/2,       0];
campos(ax,    iso_pos);
camtarget(ax, iso_target);
camup(ax, [0 0 1]);
camva(ax, 28);
drawnow;

%% ---- Video Writer Setup --------------------------------
video_file = 'museum_tour_simulation.mp4';
vid_writer = VideoWriter(video_file, 'MPEG-4');
vid_writer.FrameRate = 25;
vid_writer.Quality   = 95;
open(vid_writer);
disp(['Recording video to: ' video_file]);

%% ---- Navigation Loop ----------------------------------
for wp = 1:size(waypoints, 1)
    target_x = waypoints(wp,1);
    target_y = waypoints(wp,2);
    v = v_base;

    while true
        %% Distance to waypoint
        dist_to_wp = sqrt((x - target_x)^2 + (y - target_y)^2);
        if dist_to_wp < threshold
            % Announce exhibit
            text(ax, target_x, target_y, wall_h+1, exhibits{wp}, ...
                'Color','cyan','FontSize',11,'FontWeight','bold', ...
                'HorizontalAlignment','center');
            break;
        end

        %% Update Dynamic Visitors
        for i = 1:num_visitors
            nx = visitors(i,1) + visitor_speeds(i)*cos(visitor_dirs(i))*dt;
            ny = visitors(i,2) + visitor_speeds(i)*sin(visitor_dirs(i))*dt;
            if nx < 2 || nx > hall_length-2 || ny < 2 || ny > hall_width-2
                visitor_dirs(i) = visitor_dirs(i) + pi + 0.3*(rand-0.5);
            else
                hit_wall = false;
                for j = 1:size(wall_blocks,1)
                    b = wall_blocks(j,:);
                    if nx > b(1)-1 && nx < b(1)+b(3)+1 && ny > b(2)-1 && ny < b(2)+b(4)+1
                        hit_wall = true; break;
                    end
                end
                if hit_wall
                    visitor_dirs(i) = visitor_dirs(i) + pi + 0.5*(rand-0.5);
                else
                    visitors(i,1) = nx;
                    visitors(i,2) = ny;
                end
            end
        end

        %% Sensor Simulation
        sensor_angles   = [-pi/4, 0, pi/4];
        sensor_readings = zeros(size(sensor_angles));

        for s = 1:length(sensor_angles)
            angle   = wrapToPi(theta + sensor_angles(s));
            dir_vec = [cos(angle), sin(angle)];
            min_d   = sensor_range;
            all_obs = [obstacles; visitors];
            for i = 1:size(all_obs,1)
                rel  = all_obs(i,:) - [x y];
                proj = dot(rel, dir_vec);
                perp = norm(rel - proj*dir_vec);
                if proj > 0 && proj < min_d && perp < 1.2
                    min_d = proj;
                end
            end
            for i = 1:size(wall_blocks,1)
                b  = wall_blocks(i,:);
                dx = max([b(1)-x, 0, x-(b(1)+b(3))]);
                dy = max([b(2)-y, 0, y-(b(2)+b(4))]);
                dw = sqrt(dx^2 + dy^2);
                if dw < min_d, min_d = dw; end
            end
            sensor_readings(s) = min_d;
        end

        %% --- Potential Field Navigation -------------------
        % 1) Attractive force toward waypoint
        att_gain = 1.5;
        dx_att   = target_x - x;
        dy_att   = target_y - y;
        d_att    = sqrt(dx_att^2 + dy_att^2) + 1e-6;
        Fx =  att_gain * (dx_att / d_att);
        Fy =  att_gain * (dy_att / d_att);

        % 2) Repulsive forces from all obstacles + dynamic visitors + walls
        rep_gain   = 12.0;
        rep_radius = safe_dist + 1.5;   % influence radius

        all_obs = [obstacles; visitors];
        for i = 1:size(all_obs,1)
            ox = all_obs(i,1);  oy = all_obs(i,2);
            dx_r = x - ox;     dy_r = y - oy;
            d_r  = sqrt(dx_r^2 + dy_r^2) + 1e-6;
            if d_r < rep_radius
                mag  = rep_gain * (1/d_r - 1/rep_radius) / (d_r^2);
                Fx   = Fx + mag * (dx_r / d_r);
                Fy   = Fy + mag * (dy_r / d_r);
            end
        end

        % Repulsion from walls (push away from each wall block face)
        wall_rep_gain   = 15.0;
        wall_rep_radius = safe_dist;
        for i = 1:size(wall_blocks,1)
            b    = wall_blocks(i,:);
            cx_w = b(1) + b(3)/2;
            cy_w = b(2) + b(4)/2;
            dx_w = x - cx_w;
            dy_w = y - cy_w;
            d_w  = sqrt(dx_w^2 + dy_w^2) + 1e-6;
            % Clamp to half-extents to get distance to wall surface
            closest_x = max(b(1), min(x, b(1)+b(3)));
            closest_y = max(b(2), min(y, b(2)+b(4)));
            dx_s = x - closest_x;  dy_s = y - closest_y;
            d_s  = sqrt(dx_s^2 + dy_s^2) + 1e-6;
            if d_s < wall_rep_radius
                mag  = wall_rep_gain * (1/d_s - 1/wall_rep_radius) / (d_s^2);
                Fx   = Fx + mag * (dx_s / d_s);
                Fy   = Fy + mag * (dy_s / d_s);
            end
        end

        % 3) Desired heading from resultant force
        desired_angle = atan2(Fy, Fx);
        angular_error = wrapToPi(desired_angle - theta);

        % 4) Speed: slow down near obstacles, full speed in open space
        front_dist = sensor_readings(2);
        if front_dist < 1.5
            v = 0.0;   % emergency stop only — steering handles direction
        elseif front_dist < safe_dist
            v = v_base * (front_dist / safe_dist) * 0.5;
        else
            v = v_base;
        end

        % 5) Angular velocity — proportional to heading error
        w = 1.8 * angular_error;   % higher gain = faster heading correction
        w = max(-pi, min(pi, w));  % clamp

        %% Update Robot Pose
        x_new = x + v * cos(theta) * dt;
        y_new = y + v * sin(theta) * dt;
        theta = theta + w * dt;
        if x_new >= 1 && x_new <= hall_length-1 && y_new >= 1 && y_new <= hall_width-1
            x = x_new; y = y_new;
        else
            theta = theta + pi/2;
        end
        path_x(end+1) = x;  %#ok<SAGROW>
        path_y(end+1) = y;  %#ok<SAGROW>

        %% ---- Redraw Dynamic Objects --------------------
        cla(ax);
        draw_museum(ax, hall_length, hall_width, wall_h, wall_blocks, obstacles, waypoints, exhibits);

        % Path ribbon on floor
        if length(path_x) > 1
            plot3(ax, path_x, path_y, ones(size(path_x))*0.05, ...
                'Color',[0.2 0.8 1.0],'LineWidth',2);
        end

        % Target marker
        draw_target_marker(ax, target_x, target_y);

        % Visitors (humanoid)
        for i = 1:num_visitors
            draw_visitor(ax, visitors(i,1), visitors(i,2), visitor_dirs(i));
        end

        % Robot
        draw_robot3d(ax, x, y, theta);

        %% ---- Camera: lock to fixed isometric view ----------
        campos(ax,    iso_pos);
        camtarget(ax, iso_target);
        camup(ax, [0 0 1]);
        camva(ax, 38);

        drawnow;
        % Capture frame for video
        frame = getframe(fig);
        writeVideo(vid_writer, frame);
        pause(0.001);
    end  % while waypoint loop

    % Brief pause at each exhibit
    pause(0.5);
end  % for waypoints

% Finalize and save the video
close(vid_writer);
disp('Tour complete!');
disp(['Video saved to: ' video_file]);


%% =======================================================
%  HELPER FUNCTIONS
%% =======================================================

function draw_museum(ax, Lx, Ly, Lz, wall_blocks, obstacles, waypoints, exhibits)
% Draws static museum: floor, ceiling, walls, pillars, artworks

    % Floor (checkerboard-ish)
    [Xf,Yf] = meshgrid(0:Lx, 0:Ly);
    Zf = zeros(size(Xf));
    surf(ax, Xf, Yf, Zf, 'FaceColor',[0.25 0.22 0.18], ...
        'EdgeColor','none','FaceLighting','gouraud','AmbientStrength',0.6);

    % Ceiling (translucent)
    surf(ax, Xf, Yf, Zf+Lz, 'FaceColor',[0.15 0.15 0.2], ...
        'EdgeColor','none','FaceAlpha',0.25);

    % Walls from wall_blocks
    wall_colors = [0.45 0.40 0.35];
    for k = 1:size(wall_blocks,1)
        b = wall_blocks(k,:);
        draw_box(ax, b(1), b(2), 0, b(3), b(4), Lz, wall_colors);
    end

    % Pillars at obstacles
    for k = 1:size(obstacles,1)
        draw_cylinder_obj(ax, obstacles(k,1), obstacles(k,2), 0, 0.7, Lz, [0.5 0.45 0.4]);
    end

    % Exhibit panels on wall (simple colored boards)
    exhibit_colors = {[0.8 0.2 0.2],[0.2 0.7 0.3],[0.2 0.3 0.8],[0.8 0.6 0.1],[0.6 0.1 0.8]};
    for k = 1:size(waypoints,1)
        wx = waypoints(k,1); wy = waypoints(k,2);
        % Small glowing marker on floor
        draw_floor_circle(ax, wx, wy, 1.5, exhibit_colors{k});
        % Label above
        text(ax, wx, wy, Lz+0.5, exhibits{k}, ...
            'Color', exhibit_colors{k}, 'FontSize', 9, ...
            'FontWeight','bold','HorizontalAlignment','center', ...
            'BackgroundColor','none');
    end

    % Decorative ceiling lights
    light_positions = [15 15; 15 35; 40 10; 40 40; 60 25; 65 45];
    for k = 1:size(light_positions,1)
        lx = light_positions(k,1); ly = light_positions(k,2);
        % Light cone (just a small cylinder)
        draw_cylinder_obj(ax, lx, ly, Lz-0.5, 0.4, 0.5, [1.0 1.0 0.8]);
    end
end

% ---------------------------------------------------------
function draw_box(ax, bx, by, bz, bw, bd, bh, col)
% Draws a solid rectangular box
    vx = [0 1 1 0 0 1 1 0]*bw + bx;
    vy = [0 0 1 1 0 0 1 1]*bd + by;
    vz = [0 0 0 0 1 1 1 1]*bh + bz;

    faces = [1 2 3 4; 5 6 7 8; 1 2 6 5; 3 4 8 7; 1 4 8 5; 2 3 7 6];
    patch(ax,'Vertices',[vx(:) vy(:) vz(:)], ...
        'Faces', faces, ...
        'FaceColor', col, 'EdgeColor', col*0.6, ...
        'FaceLighting','gouraud','AmbientStrength',0.4, ...
        'DiffuseStrength',0.7);
end

% ---------------------------------------------------------
function draw_cylinder_obj(ax, cx, cy, cz, r, h, col)
% Draws a vertical cylinder
    [Xc,Yc,Zc] = cylinder(r, 16);
    Xc = Xc + cx;
    Yc = Yc + cy;
    Zc = Zc * h + cz;
    surf(ax, Xc, Yc, Zc, 'FaceColor', col, 'EdgeColor','none', ...
        'FaceLighting','gouraud','AmbientStrength',0.5);
    % Top cap
    theta_c = linspace(0,2*pi,17);
    patch(ax, cx+r*cos(theta_c), cy+r*sin(theta_c), ones(1,17)*(cz+h), ...
        col, 'EdgeColor','none','FaceLighting','gouraud');
end

% ---------------------------------------------------------
function draw_floor_circle(ax, cx, cy, r, col)
    theta_c = linspace(0, 2*pi, 40);
    patch(ax, cx + r*cos(theta_c), cy + r*sin(theta_c), ones(1,40)*0.02, ...
        col, 'EdgeColor','none','FaceAlpha',0.5, ...
        'FaceLighting','flat');
end

% ---------------------------------------------------------
function draw_visitor(ax, vx, vy, vdir)
% Humanoid: legs (2 cylinders) + torso (cylinder) + head (sphere) + arms

    skin  = [0.9 0.75 0.6];
    shirt_colors = {[0.8 0.1 0.1],[0.1 0.5 0.9],[0.2 0.7 0.2],[0.7 0.3 0.8],[0.9 0.6 0.1]};
    % pick color based on position hash
    cidx = mod(round(vx*7 + vy*3), 5) + 1;
    shirt = shirt_colors{cidx};
    pant  = [0.2 0.2 0.4];

    % Legs
    draw_cylinder_obj(ax, vx-0.15, vy, 0,    0.15, 0.9, pant);
    draw_cylinder_obj(ax, vx+0.15, vy, 0,    0.15, 0.9, pant);
    % Torso
    draw_cylinder_obj(ax, vx, vy, 0.9, 0.3,  1.0, shirt);
    % Arms (angled cylinders — approximated as thin cylinders offset)
    arm_angle = vdir + pi/2;
    ax_off = 0.35*cos(arm_angle); ay_off = 0.35*sin(arm_angle);
    draw_cylinder_obj(ax, vx+ax_off, vy+ay_off, 1.0, 0.1, 0.8, skin);
    draw_cylinder_obj(ax, vx-ax_off, vy-ay_off, 1.0, 0.1, 0.8, skin);
    % Head (sphere)
    [Xs,Ys,Zs] = sphere(10);
    Xs = Xs*0.3 + vx;
    Ys = Ys*0.3 + vy;
    Zs = Zs*0.3 + 2.2;
    surf(ax, Xs, Ys, Zs,'FaceColor',skin,'EdgeColor','none', ...
        'FaceLighting','gouraud','AmbientStrength',0.6);
end

% ---------------------------------------------------------
function draw_robot3d(ax, rx, ry, rtheta)
% 3D robot: flat box chassis + sensor dome + direction arrow + wheel arcs

    chassis_col = [0.15 0.5 0.9];   % blue
    dome_col    = [0.9 0.4 0.1];    % orange sensor dome
    wheel_col   = [0.2 0.2 0.2];

    ch = 0.5;   % chassis height
    cl = 2.0;   % chassis length
    cw = 1.2;   % chassis width

    R2 = [cos(rtheta) -sin(rtheta); sin(rtheta) cos(rtheta)];

    % Chassis corners in local frame
    corners_local = [-cl/2  cw/2;
                      cl/2  cw/2;
                      cl/2 -cw/2;
                     -cl/2 -cw/2];
    corners_world = (R2 * corners_local')';
    cx = corners_world(:,1) + rx;
    cy = corners_world(:,2) + ry;
    cz_bot = 0.2;

    % Bottom face
    patch(ax, cx, cy, ones(4,1)*cz_bot, chassis_col, ...
        'EdgeColor','k','FaceLighting','gouraud','AmbientStrength',0.5);
    % Top face
    patch(ax, cx, cy, ones(4,1)*(cz_bot+ch), chassis_col*0.8, ...
        'EdgeColor','k','FaceLighting','gouraud');
    % Side faces
    for s = 1:4
        ns = mod(s,4)+1;
        xs = [cx(s) cx(ns) cx(ns) cx(s)];
        ys = [cy(s) cy(ns) cy(ns) cy(s)];
        zs = [cz_bot cz_bot cz_bot+ch cz_bot+ch];
        patch(ax, xs, ys, zs, chassis_col*0.7, ...
            'EdgeColor','k','FaceLighting','gouraud');
    end

    % Wheels (4 small cylinders on sides)
    wheel_offsets_local = [ cl/2-0.3,  cw/2+0.05;
                             cl/2-0.3, -cw/2-0.05;
                            -cl/2+0.3,  cw/2+0.05;
                            -cl/2+0.3, -cw/2-0.05 ];
    for w = 1:4
        wpos = (R2 * wheel_offsets_local(w,:)')';
        draw_cylinder_obj(ax, wpos(1)+rx, wpos(2)+ry, 0.05, 0.15, 0.3, wheel_col);
    end

    % Sensor dome on top
    [Xd,Yd,Zd] = sphere(12);
    Xd = Xd*0.35 + rx;
    Yd = Yd*0.35 + ry;
    Zd = Zd*0.35 + (cz_bot+ch+0.35);
    surf(ax, Xd, Yd, Zd, 'FaceColor', dome_col, 'EdgeColor','none', ...
        'FaceLighting','gouraud','AmbientStrength',0.6,'FaceAlpha',0.85);

    % Direction arrow
    arr_end = [rx + 1.5*cos(rtheta), ry + 1.5*sin(rtheta), cz_bot+ch+0.1];
    plot3(ax, [rx arr_end(1)], [ry arr_end(2)], [cz_bot+ch+0.1 arr_end(3)], ...
        'r-','LineWidth',3);
end

% ---------------------------------------------------------
function draw_target_marker(ax, tx, ty)
    % Glowing ring on floor at target
    theta_t = linspace(0, 2*pi, 40);
    r1 = 1.2; r2 = 0.8;
    patch(ax, tx + r1*cos(theta_t), ty + r1*sin(theta_t), ones(1,40)*0.03, ...
        [0.1 1.0 0.4], 'EdgeColor','none','FaceAlpha',0.6,'FaceLighting','flat');
    patch(ax, tx + r2*cos(theta_t), ty + r2*sin(theta_t), ones(1,40)*0.04, ...
        [1 1 1], 'EdgeColor','none','FaceAlpha',0.8,'FaceLighting','flat');
end