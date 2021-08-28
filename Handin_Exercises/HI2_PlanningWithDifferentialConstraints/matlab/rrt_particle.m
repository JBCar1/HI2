%% RRT for a particle moving in a plane (2D world)

function [goal_idx, nodes, parents, Tplan] = rrt_particle(start, goal, world, opts)

    % Input arguments:
    % start - initial state
    % goal - desired goal state
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
    % Tplan - the time taken for computing the plan
    
    % Sample a state x in the free state space
    function x = sample_free()
        if rand < opts.beta
            x = goal;
        else
            found_random = false;
            while ~found_random
                x = [rand*(world.xmax-world.xmin) + world.xmin;...
                    rand*(world.ymax-world.ymin) + world.ymin];
                if world.obstacle_free(x)
                    found_random = true;
                end
            end
        end
    end

    % Find index of state nearest to x in the matrix nodes
    function idx = nearest(x)
        [~, idx] = min(sum((nodes-x).^2, 1));
    end

    % Steer from x1 towards x2 with step size opts.lambda
    % 
    % If the distance to x2 is less than opts.lambda, return
    % state x2.
    function x_new = steer(x1, x2)
        if norm(x2 - x1) < opts.lambda
            x_new = x2;
        else
            step = opts.lambda;
            x_new = x1 + step*(x2 - x1)/norm(x2 - x1);
        end
    end

    % Start time measurement and define variables for nodes and parents
    tic;
    nodes = [start];
    parents = [1]; % Initial state has no parent

    % YOUR CODE HERE


    Tplan = toc;
    [~, goal_idx] = min(sum((nodes - goal).^2, 1));
end
