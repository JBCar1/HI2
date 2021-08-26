%% TSFS12 Hand-in Exercise 5: Learning for autonomous vehicles --- 
% Reinforcement learning and Q-learning


% Q-learning for solving an extension of Example 6.6 in
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

alpha = 0.5; % Learning rate in Q-update
eps = 0.5; % Epsilon-greedy parameter

P_move_action = 1.0; % Probability of moving in the direction specified by action
P_dist = (1-P_move_action)/2; % Probability of moving sideways compared to intended
                              % because of disturbance

% Define size of grid world, goal, and cliff states
n_rows = 4;
n_cols = 5;

goal = [4, 5]; % Element index goal state
sink = [4, 2; 4, 3; 4, 4]; % Element indices for cliff states

%% Setup reward matrix R
R = R_grid*ones(n_rows, n_cols);
R(goal(1), goal(2)) = R_goal;
R(sink(:, 1), sink(:, 2)) = R_sink;

% Occupancy grid defines states where there are obstacles (0 - no
% obstacles, 1 - obstacles)
occ_grid = zeros(n_rows, n_cols);
occ_grid(2, 2) = 1;

% Save parameters in a struct
params.gamma = gamma;
params.R_goal = R_goal;
params.R_sink = R_sink;
params.R_grid = R_grid;
params.alpha = alpha;
params.eps = eps;
params.P_move_action = P_move_action;
params.P_dist = P_dist;
params.n_rows = n_rows;
params.n_cols = n_cols;
params.goal = goal;
params.sink = sink;
params.R = R;
params.occ_grid = occ_grid;

%% Initialize variable for value function V for each state s
V = zeros(n_rows, n_cols);

% Initialize cell object for Q-function with random values (function of
% state s and action a)
for i = 1:n_rows
    for j = 1:n_cols
        Q{i, j} = rand(4, 1);
    end
end

% Initialize Q for terminal states to zero
Q{goal(1), goal(2)} = zeros(4, 1);
Q{sink(1, 1), sink(1, 2)} = zeros(4, 1);
Q{sink(2, 1), sink(2, 2)} = zeros(4, 1);
Q{sink(3, 1), sink(3, 2)} = zeros(4, 1);

% Initialize vector for policy Pi
% Actions - ['left', 'right', 'up', 'down'] counted as 1-4
Pi = -1*ones(n_rows, n_cols);

%% Define number of iterations for Q-learning
nbr_iters = 2000;

% Initialize vector for sum of rewards for each episode
sum_r = zeros(nbr_iters, 1);

% Main loop for Q-learning
% Algorithm according to Section 6.5 in Sutton, R. S., & A. G. Barto: 
% Reinforcement learning: An introduction. MIT Press, 2018.

% Run nbr_iters episodes of Q-learning
for k = 1:nbr_iters
    % Start state
    s_curr = [n_rows, 1];
    
    terminal_state = false;
    
    % Continue Q-learning episode until terminal state reached
    while ~terminal_state
        % Select action according to epsilon-greedy strategy
        action = select_eps_greedy(s_curr, k, Q, params);
        
        % Perform the action and receive reward and next state s_prim
        [s_prim,r] = next_state(s_curr, action, params);
        
        % Q-learning update of action-value function
        Q{s_curr(1), s_curr(2)}(action) = Q{s_curr(1), s_curr(2)}(action) ...
            + alpha*(r + gamma * max(Q{s_prim(1), s_prim(2)}) ...
            - Q{s_curr(1), s_curr(2)}(action));
        
        % Update the sum of reward vector
        sum_r(k) = sum_r(k) + r;
        
        % Move to next state
        s_curr = s_prim;
        
        % Check if a terminal state has been reached (goal or sink states, 
        % closes an episode)
        if (s_curr(1) == goal(1) && s_curr(2) == goal(2)) || ...
                state_in_sink(s_curr, params)
            terminal_state = true;
            
            % Update value function V and policy Pi
            for i = 1:n_rows
                for j = 1:n_cols
                    if (occ_grid(i,j) == 1) || (i == goal(1) && j == goal(2)) || ...
                            state_in_sink([i, j], params)
                        continue;
                    end
                    % Compute value function V at current state from Q
                    % function
                    [V_ij, max_a] = max(Q{i, j});
                    V(i, j) = V_ij;
                    % Update policy Pi with the currently best action at
                    % current state (according to Q function)
                    Pi(i, j) = max_a;
                end
            end
            
            % Display current value function V and policy Pi
            % disp(V);
            % disp(Pi);
            
        end
    end
    k = k+1;
end

% Visualize the value function and policy after all iterations
plot_value_and_policy(V, Pi, params);

% Compute average of reward for N episodes for smoothing
N = 40;
for i = N:length(sum_r)
    mean_sum_r(i) = mean(sum_r(i-N+1:i));
end

% Visualize the evolution of the reward for each episode
figure(3)
plot(N:nbr_iters, mean_sum_r(N:end))
grid on
title(sprintf("Sum of rewards for each episode (average over %d)", N))
xlabel('Episode')
