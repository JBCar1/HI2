%% TSFS12 Hand-in Exercise 5: Learning for autonomous vehicles --- 
% Reinforcement learning and value iteration

% Value iteration for solving an extension of Example 6.6 in
% Sutton, R. S., & A. G. Barto: Reinforcement learning: An introduction.
% MIT Press, 2018.

clear
close all
addpath Functions

%% Define the parameters for the Markov Decision Process
gamma = 0.99; % Discount factor
R_goal = 0.0; % Reward for reaching goal state
R_sink = -10.0; % Reward for reaching 'cliff' states
R_grid = -0.1; % Reward for remaining states

P_move_action = 0.99; % Probability of moving in the direction specified by action
P_dist = (1-P_move_action)/2; % Probability of moving sideways compared to intended
                              % because of disturbance

% Define size of grid world, goal, and cliff states
n_rows = 4;
n_cols = 5;

goal = [4, 5]; % Element index goal state
sink = [4, 2; 4, 3; 4, 4]; % Element indices for cliff states

% Setup reward matrix R
R = R_grid*ones(n_rows, n_cols);
R(goal(1), goal(2)) = R_goal;
R(sink(:, 1), sink(:, 2)) = R_sink;

% Occupancy grid defines states where there are obstacles (0 - no
% obstacles, 1 - obstacles)
occ_grid = zeros(n_rows, n_cols);
occ_grid(2,2) = 1;

% Save all parameters in a struct
params.gamma = gamma;
params.R_goal = R_goal;
params.R_sink = R_sink;
params.R_grid = R_grid;
params.P_move_action = P_move_action;
params.P_dist = P_dist;
params.n_rows = n_rows;
params.n_cols = n_cols;
params.goal = goal;
params.sink = sink;
params.R = R;
params.occ_grid = occ_grid;

%% Initialize variables for value function V and policy Pi for each state s
V = zeros(n_rows, n_cols);
% Actions - ['left', 'right', 'up', 'down'] counted as 1-4
Pi = -1*ones(n_rows, n_cols);

%% Main loop for value iteration
% Algorithm according to Section 4.4 in Sutton, R. S., & A. G. Barto: 
% Reinforcement learning: An introduction. MIT Press, 2018.

converged = false;
num_iterations = 0;

% Continue the iterations until convergence criterion is fulfilled
while ~converged
    Delta = 0;
    % Go through all states
    for row = 1:n_rows
        for col = 1:n_cols
            % Check if obstacle or sink states, then continue to next state
            if (occ_grid(row, col) == 1) || ...
                    (row == goal(1) && col == goal(2)) || ...
                    state_in_sink([row, col], params)
                continue;
            end
            v = V(row, col);
            V_a = zeros(4, 1);  
            for a = 1:4   % Go through all possible actions
                [s_prim, r, p_prim] = p_grid_world([row, col], a, params);
                % Three state transitions are possible in 
                % each state, sum the terms for each of them
                for i = 1:3
                    V_prim = V(s_prim(i, 1), s_prim(i, 2));
                    V_a(a) = V_a(a) + p_prim(i)*(r(i) + gamma*V_prim);
                end
            end
            
            % Compute new value function V for current state and update
            % policy Pi
            [V_a_max,max_a] = max(V_a);
            Pi(row, col) = max_a;
            V(row, col) = V_a_max;
            % Compute improvement criterion for V
            Delta = max(Delta, abs(v-V(row, col)));
        end
    end
    % Visualize the current value function V and policy Pi in the grid
    % world
    plot_value_and_policy(V, Pi, params)
    
    num_iterations = num_iterations + 1;
    
    % Display current value function V and policy Pi
    V
    Pi
    pause;
    
    % If improvement criterion below threshold, stop the value iteration
    % since convergence has been achieved
    if Delta < 1e-6
        converged = true;
        disp(sprintf("Converged after %d iterations", num_iterations));
    end
end

