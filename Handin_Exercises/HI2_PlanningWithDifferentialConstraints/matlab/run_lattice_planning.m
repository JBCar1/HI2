clear
close all
addpath Functions
% If you are in the student labs at campus, run the line below to get 
% access to CasADi
% addpath /courses/tsfs12/casadi  

%%
filename = 'mprims.mat';
if exist(filename, 'file')
    mp = MotionPrimitives(filename);
else
    
    % Define the initial states and desired goal states for the motion
    % primitives
    theta_init = [0 pi/4 pi/2 3*pi/4 pi -3*pi/4 -pi/2 -pi/4];
    x_vec = [3 2 3 3 3 1 3 3 3 2 3];
    y_vec = [2 2 2 1 1 0 -1 -1 -2 -2 -2];
    th_vec = [0 pi/4 pi/2 0 pi/4 0 -pi/4 0 -pi/2 -pi/4 0];
    lattice = [x_vec;y_vec; th_vec];

    % Vehicle parameters and constraints
    L = 1.5;        % Wheel base (m)
    v = 15;         % Constant velocity (m/s)
    u_max = pi/4;   % Maximum steering angle (rad)

    % Construct a MotionPrimitives object and generate the 
    % motion primitives using the constructed lattice and 
    % specification of the motion primitives
    mp = MotionPrimitives();
    mp.generate_primitives(theta_init, lattice, L, v, u_max);
    % Save the motion primitives to avoid unnecessary re-computation
    mp.save(filename);
end

%% Plot the computed primitives

figure(10)
clf()
mp.plot();
grid on;
xlabel('x');
ylabel('y');
axis('square');
box off
title('Motion Primitives');

%% Create world with obstacles using the BoxWorld class

xx = -2:1:12;
yy = -2:1:12;
th = [0 pi/4 pi/2 3*pi/4 pi -3*pi/4 -pi/2 -pi/4];
lattice = {xx, yy, th};

world = BoxWorld(lattice);

mission_nbr = 1;

% Example planning missions

switch mission_nbr
    case 1
        world.add_box(0, 1, 2, 4)
        world.add_box(0, 6, 6, 4)
        world.add_box(4, 1, 6, 4)
        world.add_box(7, 7, 3, 3)

        start = [0; 0; 0]; % Initial state
        goal = [7; 8; pi/2]; % Final state
    
    case 2
        world.add_box(0, 1, 3, 4)
        world.add_box(0, 7, 10, 3)
        world.add_box(4, 1, 6, 4)
        
        start = [0; 0; 0]; % Initial state
        goal = [8; 6; pi/2]; % Final state
    case 3
        world.add_box(-2, 0, 10, 5)
        world.add_box(-2, 6, 10, 4)
        
        start = [0; 5; 0]; % Initial state
        goal = [0; 6; pi]; % Final state
    case 4
        world.add_box(0, 3, 10, 2)
        world.add_box(0, 5, 4, 2)
        world.add_box(6, 5, 4, 2)

        start = [5; 7; 0]; % Initial state
        goal = [5; 6; 0]; % Final state
end

n = world.num_nodes();
eps = 1e-5;

% Define the initial and goal state for the graph search by finding the
% node number (column number in world.st_sp) in the world state space
mission.start.id = find(all(abs(world.st_sp - start) < eps, 1));
mission.goal.id = find(all(abs(world.st_sp - goal) < eps, 1));

arrow_length = 1.5;
arrow_width = 2;
start_arrow = [start(1:2), start(1:2) + arrow_length * [cos(start(3)); sin(start(3))]];
goal_arrow = [goal(1:2), goal(1:2) + arrow_length * [cos(goal(3)); sin(goal(3))]];

figure(20)
clf()
world.draw()
hold on
axis([world.xmin, world.xmax, world.ymin, world.ymax])
plot(start(1), start(2), 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 8)
plot(goal(1), goal(2), 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 8)
hold off
axis('manual')
arrow(start_arrow(:, 1), start_arrow(:, 2), 'Length', 15, 'Width', arrow_width, 'Color', 'b');
arrow(goal_arrow(:, 1), goal_arrow(:, 2), 'Length', 15, 'Width', arrow_width, 'Color', 'k');

xlabel('x');
ylabel('y');

%% Investigate the state-transition function
% The state-transition function is fully implemented
% Apply it to the initial state and interpret the result

[xi, u, d] = next_state(mission.start.id, world, mp, true); % Allow reversing
disp(xi)
disp(u)
disp(d)

[xi, u, d] = next_state(mission.start.id, world, mp, false);  % Do not allow reversing
disp(xi)
disp(u)
disp(d)

%%

% Define the function providing possible new states starting from x
f = @(x) next_state(x, world, mp);

% Define heuristics for cost-to-go
cost_to_go = @(x, xg) norm(world.st_sp(1:2, x) - world.st_sp(1:2, xg));

% Solve problem using graph-search strategies from Hand-in Exercise 1

fprintf('Planning ...');
% Start planning
plan = {};
plan{end+1} = breadth_first(n, mission, f, [], 3);
plan{end+1} = depth_first(n, mission, f, [], 3);
plan{end+1} = dijkstra(n, mission, f, [], 3);
plan{end+1} = astar(n, mission, f, cost_to_go, 3);
plan{end+1} = best_first(n, mission, f, cost_to_go, 3);
fprintf('Done!\n');

opt_length = plan{3}.length; % Dijkstra is optimal

%% YOUR CODE HERE

% Hint: For information on a useful function
% >> help mp.plan_to_path 

