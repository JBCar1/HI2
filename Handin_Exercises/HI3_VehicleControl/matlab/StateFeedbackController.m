classdef StateFeedbackController < ControllerBase
    properties
        K;
        L;
        plan;
        goal_tol;
        state;
        s0;
    end
    methods
        function obj=StateFeedbackController(K, L, path, goal_tol)
            if nargin < 4
                goal_tol = 1;
            end
            obj = obj@ControllerBase();
            obj.K = K;  % Feedback gain
            obj.L = L;  % Vehicle wheel base
            obj.plan = path;  % Path to follow
            obj.goal_tol = goal_tol;  % Goal tolerance
            obj.s0 = 0.0;  % Path position state
        end

        function theta_e = heading_error(obj, theta, s)
            % theta, heading of vehicle
            % s - position (length) on path
            % Compute heading error. The SplinePath method heading is useful
            
            % YOUR CODE HERE
            theta_e = 0.0;
        end

        function c = u(obj, t, w)
            theta = w(3);
            p_car = w(1:2);
            
            % Compute d and theta_e errors. Use the SplinePath method project
            % and the obj.heading_error() function you've written above

            % YOUR CODE HERE
            d = 0;
            theta_e = 0;

            % Compute control signal delta
            acc = 0;  % Constant speed
            delta = 0; % Steering angle

            c = [delta, acc];
        end

        function r = run(obj, t, w)
            % Function that returns true until goal is reached
            p_car = w(1:2);
            p_goal = obj.plan.path(end, :);
            r = norm(p_car - p_goal) > obj.goal_tol;
        end
    end
end
