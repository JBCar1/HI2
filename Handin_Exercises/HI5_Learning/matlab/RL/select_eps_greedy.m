function action = select_eps_greedy(s_curr, itr_nbr, Q, params)
    % Select the action to take at a particular state s_curr using an 
    % epsilon-greedy strategy

    % Constant epsilon over episodes
    eps = params.eps;
    
    % Sample a number between 0 and 1, select action based on the outcome
    rnd = rand(1);

    % Find the best action according to current Q at state s_curr
    [~, max_a] = max(Q{s_curr(1), s_curr(2)});

    a_list = [];

    % Create vector with remaining actions
    for i = 1:4
        if i ~= max_a
            a_list(end+1) = i;
        end
    end
    
    % Select action according to sampled random value
    if rnd < 1-eps+eps/4
        action = max_a;
    elseif rnd < 1-eps+eps/2
        action = a_list(1);
    elseif rnd < 1-eps+3*eps/4
        action = a_list(2);
    else
        action = a_list(3);
    end
end
