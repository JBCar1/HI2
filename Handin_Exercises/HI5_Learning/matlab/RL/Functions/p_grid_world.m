function [next_state, reward, probability] = p_grid_world(s, a, params)
    % Function for computing the possible new states and associated rewards 
    % and probabilities for a given state s and action a

    R = params.R;
    row = s(1);
    col = s(2);

    P_move_action = params.P_move_action;
    P_dist = params.P_dist;

    if a == 1 % Move left
        s_0 = [row, col - 1];
        s_1 = [row - 1, col];  % Accidental up
        s_2 = [row + 1, col];  % Accidental down
    elseif a == 2  % Move right
        s_0 = [row, col + 1];
        s_1 = [row - 1, col];  % Accidental up
        s_2 = [row + 1, col];  % Accidental down
    elseif a == 3  % Move up
        s_0 = [row - 1, col];
        s_1 = [row, col - 1];  % Accidental left
        s_2 = [row, col + 1];  % Accidental right
    elseif a == 4  % Move down
        s_0 = [row + 1, col];
        s_1 = [row, col - 1];  % Accidental left
        s_2 = [row, col + 1];  % Accidental right
    end

    % If the next state happens to be non-feasible, the agent stays in the
    % current state s
    if ~state_feasible(s_0, params)
        s_0 = s;
    end

    if ~state_feasible(s_1, params)
        s_1 = s;
    end

    if ~state_feasible(s_2, params)
        s_2 = s;
    end

    next_state = [s_0; s_1; s_2];
    reward = [R(s_0(1),s_0(2)), R(s_1(1),s_1(2)), R(s_2(1),s_2(2))];
    probability = [P_move_action, P_dist, P_dist];
end

function [is_feasible] = state_feasible(s, params)
    % Function for checking if a state is feasible (i.e., not outside of 
    % grid world or an obstacle)

    occ_grid = params.occ_grid;
    n_cols = params.n_cols;
    n_rows = params.n_rows;

    is_feasible = (0 < s(1)) && (s(1) <= n_rows) && (0 < s(2)) ...
        && (s(2) <= n_cols) && (occ_grid(s(1), s(2)) == 0);
end
