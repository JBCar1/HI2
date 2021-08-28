%% RRT for a model with a differential constraint defined by the input sim

% Example usage:
%      [goal_idx, nodes, parents, state_trajectories, Tplan] = 
%            rrt_diff(start, goal, u_c, @sim_car, world, opts);

function [goal_idx, nodes, parents, state_trajectories, Tplan] = ...
          rrt_diff(start, goal, u_c, sim, world, opts)
      
    % Input arguments:
    % start - initial state
    % goal - desired goal state
    % u_c - vector with possible control actions (steering angles)
    % sim - function reference to the simulation model of the car motion
    % world - description of the map of the world
    %         using an object from the class BoxWorld
    % opts - structure with options for the RRT
    
    % Output arguments:
    % goal_idx - index of the node closest to the desired goal state
    % nodes - 2 x N matrix with each column representing a state j
    %         in the tree
    % parents - 1 x N vector with the node number for the parent of node j 
    %           at element j in the vector (node number counted as column
    %           in the matrix nodes)
    % state_trajectories - a struct with the trajectory segment for 
    %                 reaching node j at element j (node number counted 
    %                 as column in the matrix nodes)
    % Tplan - the time taken for computing the plan
    
    % Sample a state x in the free state space
    function x = sample_free()
        if rand < opts.beta
            x = goal;
        else
            found_random = false;
            th = rand*2*pi - pi;
            while ~found_random
                p = [rand*(world.xmax - world.xmin) + world.xmin;...
                    rand*(world.ymax - world.ymin) + world.ymin];
                if world.obstacle_free(p)
                    found_random = true;
                    x = [p; th];
                end
            end
        end
    end

    % Find index of state nearest to x in nodes
    function idx = nearest(x)
        [~, idx] = min(distance_fcn(nodes, x));
    end

    % Compute all possible paths for different steering control signals u_c
    % to move from x_nearest towards x_rand, without collision
    %
    % If no valid paths are found, the returned variables are empty
    function [valid_new_paths, dist_to_x_rand] = steer_candidates(x_nearest, x_rand)
        valid_new_paths = {};
        dist_to_x_rand = [];
        
        for k=1:length(u_c)
            p = sim(x_nearest, u_c(k), opts.lambda);
            if world.obstacle_free(p)
                valid_new_paths{end+1} = p;
                dist_to_x_rand(end+1) = distance_fcn(p(:, end), x_rand);
            end
        end
    end

    % Function for computing the distance between states x1 and x2, 
    % where x1 can be a matrix with several state vectors, treating  
    % all states equally
    function dist = distance_fcn(x1, x2)
        dist = sqrt(sum((x1 - x2).^2, 1));
    end

    % Start time measurement and define variables for nodes, parents, and 
    % associated trajectories
    tic;
    nodes = [start];
    parents = [1];
    state_trajectories = {0}; % No trajectory segment needed to reach start state

    % YOUR CODE HERE

    Tplan = toc;
    [~, goal_idx] = min(distance_fcn(nodes,goal));
end
