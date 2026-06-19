% Museum Hall Tour Guide
clc; clear; close all;

% Parameters 
v = 3;                  
dt = 0.1;                
safe_dist = 3;         
threshold = 0.5;        
sensor_range = 10;      

hall_width = 50;
hall_length = 75;

% Initial Robot Pose
x = 5; y = 5; theta = pi/4;

% Waypoints
waypoints = [9, 9; 20, 30; 40, 30; 60, 40; 70, 10];
exhibits = {'Entry', 'Exhibit A', 'Exhibit B', 'Exhibit C', 'Exit'};

% Obstacles
obstacles = [25, 25; 30, 32; 40, 20; 50, 30];

% Walls
wall_blocks = [
    30 0 1 20;      
    15 30 1 20;     
    0 0 hall_length 1;             
    0 hall_width-1 hall_length 1;  
    0 0 1 hall_width;              
    hall_length-1 0 1 hall_width   
];

% Visitors (initial positions)
visitors = [12, 12; 20, 40; 35, 40; 55, 10; 25, 15];
num_visitors = size(visitors, 1);
visitor_speeds = 0.5 + rand(num_visitors, 1); 
visitor_dirs = 2 * pi * rand(num_visitors, 1); 

% Path tracking
path_x = []; path_y = [];

% 2D Visualization Setup
figure;
axis equal;
axis([0 hall_length 0 hall_width]);
xlabel('X (ft)'); ylabel('Y (ft)');
title('2D Autonomous Tour Guide in Museum Hall (Dynamic Visitors)');
grid on;
hold on;

% Navigation Loop
for wp = 1:size(waypoints, 1)
    target_x = waypoints(wp,1);
    target_y = waypoints(wp,2);

    while true
        dist = sqrt((x - target_x)^2 + (y - target_y)^2);
        if dist < threshold
            text(target_x, target_y + 1, exhibits{wp}, 'FontSize', 10, 'Color', 'g');
            break;
        end

        % --- Update Dynamic Visitors ---
        for i = 1:num_visitors
            % Predict the new visitor position
            new_x = visitors(i,1) + visitor_speeds(i)*cos(visitor_dirs(i))*dt;
            new_y = visitors(i,2) + visitor_speeds(i)*sin(visitor_dirs(i))*dt;
            
            % Check if the new position is inside the hall boundaries
            if new_x < 1 || new_x > hall_length-1 || new_y < 1 || new_y > hall_width-1
                visitor_dirs(i) = pi - visitor_dirs(i); 
            else
                % Check if the new position collides with any walls
                collides_with_wall = false;
                for j = 1:size(wall_blocks, 1)
                    b = wall_blocks(j,:);
                    % Check if the new position is within the bounding box of any wall block
                    if new_x > b(1) && new_x < (b(1) + b(3)) && new_y > b(2) && new_y < (b(2) + b(4))
                        collides_with_wall = true;
                        break;
                    end
                end
                
                if collides_with_wall
                    visitor_dirs(i) = pi - visitor_dirs(i);
                else
                    visitors(i,1) = new_x; 
                    visitors(i,2) = new_y;
                end
            end
        end

        % Basic navigation toward waypoint
        angle_to_target = atan2(target_y - y, target_x - x);
        angular_error = wrapToPi(angle_to_target - theta);
        w = 0.3 * angular_error;

        % Sensor simulation
        sensor_angles = [-pi/4, 0, pi/4]; 
        sensor_readings = zeros(size(sensor_angles));

        for s = 1:length(sensor_angles)
            angle = wrapToPi(theta + sensor_angles(s));
            dir = [cos(angle), sin(angle)];
            min_dist = sensor_range;

            % Check all obstacles and dynamic visitors
            all_obs = [obstacles; visitors];
            for i = 1:size(all_obs,1)
                rel = all_obs(i,:) - [x y];
                proj = dot(rel, dir);
                perp = norm(rel - proj*dir);
                if proj > 0 && proj < min_dist && perp < 1
                    min_dist = proj;
                end
            end

            % Wall detection
            for i = 1:size(wall_blocks, 1)
                b = wall_blocks(i,:);
                dx = max([b(1) - x, 0, x - (b(1) + b(3))]);
                dy = max([b(2) - y, 0, y - (b(2) + b(4))]);
                dist_wall = sqrt(dx^2 + dy^2);
                if dist_wall < min_dist
                    min_dist = dist_wall;
                end
            end

            sensor_readings(s) = min_dist;

            % Sensor rays (optional)
            ray_end = [x y] + min_dist * dir;
            plot([x ray_end(1)], [y ray_end(2)], '--c');
        end

        % Avoidance Logic - Improved
        front_dist = sensor_readings(2);
        left_dist = sensor_readings(1);
        right_dist = sensor_readings(3);

        % Predictive Collision Avoidance
        if front_dist < 1.5
            % Hard avoidance
            if left_dist > right_dist
                w = pi;  % Turn left sharply
            else
                w = -pi; % Turn right sharply
            end
            v = 0;  % Emergency stop
        elseif front_dist < safe_dist
            v = 0.2;  % Creep forward slowly
            if left_dist > right_dist
                w = w + pi/2;
            else
                w = w - pi/2;
            end
        elseif left_dist < safe_dist
            w = w + pi/6;
        elseif right_dist < safe_dist
            w = w - pi/6;
        else
            v = 1;  % Normal speed
        end


        % Update robot pose
        x_new = x + v * cos(theta) * dt;
        y_new = y + v * sin(theta) * dt;
        theta = theta + w * dt;

        if x_new >= 0 && x_new <= hall_length && y_new >= 0 && y_new <= hall_width
            x = x_new; y = y_new;
        else
            theta = theta + pi/2;
        end

        % Path tracking
        path_x(end+1) = x;
        path_y(end+1) = y;

        % Visualization
        cla;
        for i = 1:size(wall_blocks,1)
            b = wall_blocks(i,:);
            rectangle('Position', [b(1), b(2), b(3), b(4)], 'FaceColor', [0.3 0.3 0.3]);
        end
        scatter(obstacles(:,1), obstacles(:,2), 100, 'k', 'filled');
        scatter(visitors(:,1), visitors(:,2), 100, 'm', 'filled');
        plot(path_x, path_y, 'k-', 'LineWidth', 1.5);
        drawRobot2D(x, y, theta);
        plot(target_x, target_y, 'go', 'MarkerSize', 10, 'LineWidth', 2);
        xlim([0 hall_length]); ylim([0 hall_width]);
        pause(0.01);
    end
end
function drawRobot2D(x, y, theta)
    l = 2; w = 1;
    base = [ -l/2,  l/2,  l/2, -l/2;
             -w/2, -w/2,  w/2,  w/2 ];
    R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
    rot = R * base;
    bx = rot(1,:) + x;
    by = rot(2,:) + y;
    fill(bx, by, 'b');
    head = [x + cos(theta), y + sin(theta)];
    plot([x head(1)], [y head(2)], 'r-', 'LineWidth', 2);
end