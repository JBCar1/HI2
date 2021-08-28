%% RRT* for a particle mass moving in a plane (2D world)

function [goal_idx, nodes, parents, Tplan] = rrt_star_particle(start, goal, world, opts)
    
    % Input arguments:
    % start - initial state
    % goal - desired goal state
    % world - description of the map of the world
    %         using an object from the class BoxWorld
    % opts - structure with options for the RRT*
    
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

    % Find index of state nearest to x in nodes
    function idx = nearest(x)
        [~, idx] = min(sum((nodes - x).^2, 1));
    end

    % Find the indices of the states in nodes within a neighborhood with
    % radius r from state x
    function idx = near(x)
        idx = find(sum((nodes - x).^2, 1) < opts.r_neighbor^2);
    end

    % Steering function for moving from x1 towards x2 with step size
    % opts.lambda. If the distance to x2 is less than opts.lambda, x_new
    % becomes state x2.
    function x_new = steer(x1, x2)
        if norm(x2 - x1) < opts.lambda
            x_new = x2;
        else
            step = opts.lambda;
            x_new = x1 + step*(x2 - x1)/norm(x2 - x1);
        end
    end

    % Function for connecting along a path from tree root to x_new with
    % minimum cost among the states in a neighborhood of x_new
    % described by the (column) indices near_idx in nodes. The variable
    % idx_nearest is the index (column in matrix nodes) for the node 
    % closest to x_new and cost_via_nearest is the cost to reach x_new 
    % via the nearest node.
    function [idx_min, cost_min] = ...
            connect_min_cost(x_new, near_idx, idx_nearest, cost_via_nearest)
        idx_min = idx_nearest;
        cost_min = cost_via_nearest;
        for ii=1:numel(near_idx)
            x_near = nodes(:, near_idx(ii));
            
            % Make path from x_near to x_new
            p_x = [linspace(x_near(1), x_new(1), 10);...
                linspace(x_near(2), x_new(2), 10)];
            
            cost_near = cost(near_idx(ii)) + norm(x_new - nodes(:, near_idx(ii)));
            if  cost_near < cost_min && world.obstacle_free(p_x)
                cost_min = cost_near;
                idx_min = near_idx(ii);
            end
        end
        
    end

    % Function for (possible) rewiring of the nodes in the neighborhood of
    % x_new described by the indices near_idx in nodes (column numbers) 
    % via the new state x_new, if a path with less cost could be found. 
    % The variable cost_min is the cost-to-come to x_new 
    % (computed in connect_min_cost)
    function rewire_neighborhood(x_new, near_idx, cost_min)
        for ii=1:numel(near_idx)
            x_near = nodes(:, near_idx(ii));
            
            % Create path from x_near to x_new
            p_x = [linspace(x_near(1), x_new(1), 10);...
                linspace(x_near(2), x_new(2), 10)];
            cost_near = cost_min + norm(x_new - nodes(:, near_idx(ii)));
            if  cost_near < cost(near_idx(ii)) && world.obstacle_free(p_x)
                parents(near_idx(ii)) = numel(parents); % x_new is the latest 
                                                        % node in the tree
                cost(near_idx(ii)) = cost_near;
            end
        end
    end

    % Start time measurement and define variables for nodes, parents, and 
    % associated cost
    tic;
    nodes = [start];
    parents = [1];
    % cost - 1 x N vector with the cost for reaching node j from the
    %           initial state (tree root) at element j in the vector 
    %           (node number counted as column in the matrix nodes)
    cost = [0];

    % YOUR CODE HERE
    

    Tplan = toc;
    [~, goal_idx] = min(sum((nodes-goal).^2, 1));
end
