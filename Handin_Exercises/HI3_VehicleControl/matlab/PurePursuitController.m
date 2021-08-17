classdef PurePursuitController < ControllerBase
    properties
        l;
        L;
        plan;
        goal_tol;
        s;
    end
    methods
        function obj=PurePursuitController(l, L, path, goal_tol)
            obj = obj@ControllerBase();
            if nargin < 4
                goal_tol = 0.25;
            end
            obj.l = l;
            obj.L = L;
            obj.plan = path;
            obj.goal_tol = goal_tol;
            obj.s = 0;
        end

        function p_purepursuit = pursuit_point(obj, p_car)
            % Input: 
            %   p_car - position of vehicle in globval coordinates
            %
            % Output:
            %   p_purepursuit - position of pursuit point in global
            %                   coordinates.

            s = obj.s; % Last stored path parameter
            path_points = obj.plan.path;  % Points 
            l = obj.l;  % Pure-pursuit look-ahead
            % Your code here
            
            % Hint: It is typically not important to find a point at _exactly_ distance l, 
            %       for example search pure-pursuit point among the points in path_points
            %       but don't forget to take into account the approximate pursuit-horizon when
            %       computing the steering angle.

            p_purepursuit = [0, 0]; 
        end
     
        function delta = pure_pursuit_control(obj, dp, theta)
            % Compute pure-pursuit steer angle.
            %
            %  Input:
            %    dp - Vector from position of car to pursuit point
            %    theta - heading of vehicle
            %
            % Output:
            %   delta - steer angle

            % Your code here to compute new steering angle
            delta = 0;
        end

        function c = u(obj, t, w)
            % Compute control action
            %
            %  Input:
            %    t - current time
            %    w - current state w = (x, y, theta, v)
            %  
            %  Output:
            %    return [delta, acc] where delta is steer angle and acc acceleration

            p_car = w(1:2);
            theta = w(3);      

            % Your code here to compute steering angle, use the functions
            % obj.pursuit_point() and obj.pure_pursuit_control() you 
            % have written above.

            delta = 0;
            acc = 0;
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
