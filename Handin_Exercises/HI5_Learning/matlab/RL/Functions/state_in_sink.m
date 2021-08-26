function [s_in_sink] = state_in_sink(s, params)
    % Check if a particular state s is a sink state
    row = s(1);
    col = s(2);

    sink = params.sink;

    s_in_sink = (row == sink(1,1) && col == sink(1,2)) || ...
                (row == sink(2,1) && col == sink(2,2)) || ...
                (row == sink(3,1) && col == sink(3,2));

end
