function [s_prim, r] = next_state(s, a, params)
    % Function for computing the next state s_prim and its 
    % associated reward
    [next_state, reward, p] = p_grid_world(s, a, params);
    n = numel(p);
    idx = datasample(1:n, 1, 'Weights', p);
    s_prim = next_state(idx, :);
    r = reward(idx);
end
