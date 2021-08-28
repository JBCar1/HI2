%% Run the RRT with a kinematic car model (two translational and one
%% orientational degrees-of-freedom)

clear
close all
addpath Functions

%% Define world with obstacles

world = BoxWorld({[0, 10], [0, 10]});

world.add_box(0, 1, 2, 4)
world.add_box(0, 6, 6, 4)
world.add_box(4, 1, 6, 4)
world.add_box(7, 7, 3, 3)

figure(10)
clf()
world.draw()
axis([world.xmin, world.xmax, world.ymin, world.ymax])
xlabel('x')
ylabel('y');

start = [1; 0; pi/4]; % Start state (x,y,th)
goal = [6.5; 9; pi/2]; % Goal state (x,y,th)

% Define the possible control inputs
u_c = linspace(-pi/4, pi/4, 11);

% Define parameters and data structures

opts.beta = 0.05; % Probability of selecting goal state as target state
opts.lambda = 0.1; % Step size (in time)
opts.eps = -0.01; % Threshold for stopping the search (negative for full search)
opts.K = 4000;    % Maximum number of iterations

%% Solve problem

fprintf('Planning ...\n');
[goal_idx, nodes, parents, state_trajectories, T] = rrt_diff(start, goal, u_c, @sim_car, world, opts);
fprintf('Finished in %.2f sek\n', T);

%% YOUR CODE HERE

% Hint on plotting: To plot the path corresponding to the found solution,
% the following code could be useful (utilizing backtracking from the goal 
% node:
%
% idx = goal_idx;
% while idx > 1
%     p = state_trajectories{idx};
%     plot(p(1, :), p(2, :), 'b', 'LineWidth', 4)
%     idx = parents(idx);  
% end
