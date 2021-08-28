%% Run the RRT* for a particle moving in a plane (2D world)

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
xlabel('x');
ylabel('y');

start = [1; 0];     % Start state
goal = [6.5; 9];      % Goal state

% Define parameters and data structures

opts.beta = 0.01;       % Probability of selecting goal state as target state
opts.lambda = 0.1;       % Step size
opts.eps = -0.01;       % Threshold for stopping the search (negative for full search)
opts.r_neighbor = 0.5;  % Radius of circle for definition of neighborhood
opts.K = 10000;         % Maximum number of iterations

%% Solve problem

fprintf('Planning ...\n');
[goal_idx, nodes, parents, T] = rrt_star_particle(start, goal, world, opts);
fprintf('Finished in %.2f sek\n', T);

%% YOUR CODE HERE


