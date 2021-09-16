function [xi, u, d] = next_state(x, world, mp, rev, tol)

    % Input arguments:
    % x - current state
    % world - description of the map of the world
    %         using the class BoxWorld
    % mp - object with motion primitives of the class MotionPrimitives
    % rev - Allow reversing (default: true)
    % tol - tolerance for comparison of closeness of states
    
    % Output arguments:
    % xi - 1 x N matrix containing the indices of possible next states 
    %      from current state x, considering the obstacles and size of 
    %      the world model. To get the state corresponding to the 
    %      first element in xi, 
    %         world.st_sp(:, xi(1))
    % u - N x 3 matrix with the two indices of the motion primitives used 
    %     for reaching each state (row in u corresponds to column in xi)
    %     and the driving direction (1 forward, -1 reverse)
    %
    %     If u_i = u(1,:) (first row), the corresponding motion primitive is
    %      mp.mprims{u_i(1)}{u_i(2)}
    %
    % d - 1 x N vector with the cost associated with each possible 
    %     transition in xi 
    
    if nargin < 4
        rev = true;
    end
    if nargin < 5
        tol = 1e-5;
    end
    xi = [];
    u = [];
    d = [];

    x_state = world.st_sp(:, x);
    
    % Extract motion primtives (forward and reverse driving) corresponding
    % to the current angle state. Base set of motion primitives (nominally
    % corresponding to forward driving) is reversed for obtaining reverse
    % driving of the corresponding motion primitive.
    
    mprims_x = {};
    mprims_x_ind = {};
    
    for i = 1:length(mp.mprims)
       for j = 1:length(mp.mprims{i})   
           % Check forward driving
           if abs(mp.mprims{i}{j}.th(1) - x_state(3)) < tol
               mprims_x{end+1} = mp.mprims{i}{j};
               mprims_x_ind{end+1} = [i j 1];
           end
           if rev
               % Check reverse driving
               if abs(mp.mprims{i}{j}.th(end) - x_state(3)) < tol
                   mprims_x{end+1} = mp.mprims{i}{j};
                   mprims_x_ind{end+1} = [i j -1];
               end
           end
       end
    end
    
    % Iterate through all available primitives compatible with the current 
    % angle state
    for j = 1:length(mprims_x)
        mpj = mprims_x{j};
        mpj_ind = mprims_x_ind{j};

        % Create path to next state
        
        if mpj_ind(3) == 1   % Forward driving
            p = x_state(1:2) + [mpj.x; mpj.y];
            state_next(1:2, 1) = p(1:2, end);
            state_next(3, 1) = mpj.th(end);
        else   % Reverse driving
            p = x_state(1:2) + [fliplr(mpj.x)-mpj.x(end); fliplr(mpj.y)-mpj.y(end)];
            state_next(1:2, 1) = p(1:2, end);
            state_next(3, 1) = mpj.th(1);
        end
        
        % Check if the path to next state is in the allowed area
        if ~world.in_bound(state_next) || ~world.obstacle_free(p)
            continue;
        else
            xi = [xi find(all(abs(world.st_sp - state_next) < tol, 1))];
            u = [u; mpj_ind];
            d = [d mprims_x{j}.ds];
        end
    end
end
