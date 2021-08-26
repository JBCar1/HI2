function plot_value_and_policy(V, Pi, params)
    % Auxiliary function to plot the current iteration of the value 
    % function V and policy Pi

    n_rows = params.n_rows;
    n_cols = params.n_cols;
    occ_grid = params.occ_grid;
    R = params.R;
    goal = params.goal;
    sink = params.sink;

    actions = {'left', 'right', 'up', 'down'};

    figure(1)
    clf;
    hold on;
    for i = 1:n_rows
        for j = 1:n_cols
            if occ_grid(i,j) == 1
                text(j-0.25, n_rows-i+1, sprintf('%.3f', 0.0),...
                    'fontsize', 15, 'color', 'k')
            elseif i == sink(1,1) && j == sink(1,2)
                text(j-0.25, n_rows-i+1, sprintf('%.3f', R(i,j)),...
                    'fontsize', 15, 'color', 'r')
            elseif i == sink(2,1) && j == sink(2,2)
                text(j-0.25, n_rows-i+1, sprintf('%.3f', R(i,j)),...
                    'fontsize', 15, 'color', 'r')
            elseif i == sink(3,1) && j == sink(3,2)
                text(j-0.25, n_rows-i+1, sprintf('%.3f', R(i,j)),...
                    'fontsize', 15, 'color', 'r')
            elseif i == goal(1) && j == goal(2)
                text(j-0.25, n_rows-i+1, sprintf('%.3f', R(i,j)),...
                    'fontsize', 15, 'color', 'g')
            else
                text(j-0.25, n_rows-i+1, sprintf('%.3f', V(i,j)),...
                    'fontsize', 15, 'color', 'b')
            end
        end
    end
    axis([0 n_cols+1 0 n_rows+1])
    axis off

    figure(2)

    clf;
    hold on;
    for i = 1:n_rows
        for j = 1:n_cols
            if Pi(i,j) ~= -1
                text(j-0.25, n_rows-i+1, actions(Pi(i,j)), 'fontsize', 15)
            elseif i == goal(1) && j == goal(2)
                text(j-0.25, n_rows-i+1, sprintf('%.3f', R(i,j)),...
                    'fontsize', 15, 'color', 'g')
            elseif i == sink(1,1) && j == sink(1,2)
                text(j-0.25, n_rows-i+1, sprintf('%.3f', R(i,j)),...
                    'fontsize', 15, 'color', 'r')
            elseif i == sink(2,1) && j == sink(2,2)
                text(j-0.25, n_rows-i+1, sprintf('%.3f', R(i,j)),...
                    'fontsize', 15, 'color', 'r')
            elseif i == sink(3,1) && j == sink(3,2)
                text(j-0.25, n_rows-i+1, sprintf('%.3f', R(i,j)),...
                    'fontsize', 15, 'color', 'r')
            end
        end
    end
    axis([0 n_cols+1 0 n_rows+1])
    axis off
end