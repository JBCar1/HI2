classdef ModelPredictiveController < ControllerBase
    properties
        plan;
        gamma_d;
        gamma_theta;
        gamma_u;
        steer_limit;
        L;
        sample_rate;
        prediction_horizon;
        N;
        goal_tol;
        s0;
        s_max;
        optimizer;
    end
    methods
        function obj = ModelPredictiveController(controller_params, path, dt, goal_tol)
            if nargin < 4
                goal_tol = 1;
            end
            obj = obj@ControllerBase();
            
            obj.plan = path;
            obj.gamma_d = controller_params.gamma_d;
            obj.gamma_theta = controller_params.gamma_theta;
            obj.gamma_u = controller_params.gamma_u;
            obj.steer_limit = controller_params.steer_limit;
            obj.L = controller_params.L;
            
            obj.sample_rate = dt;
            obj.prediction_horizon = controller_params.h_p;
            obj.N = round(obj.prediction_horizon / obj.sample_rate);
            
            obj.goal_tol = goal_tol; 

            obj.s0 = 0.0; 
            obj.s_max = obj.plan.length;

            obj.optimizer = obj.construct_problem();
        end
        
        function theta_e = heading_error(obj, theta, s)
            % theta, heading of vehicle
            % s - position (length) on path
            % Compute heading error. The SplinePath method heading is useful
            
            % YOUR CODE HERE
            theta_e = 0.0;
        end

        function optimizer = construct_problem(obj)
            
            dt = obj.sample_rate;
            
            % Create casadi.Opti instance
            opti = casadi.Opti('conic');
            
            d0 = opti.parameter();
            th0 = opti.parameter();
            v = opti.parameter();
            curvature = opti.parameter(obj.N);
            
            X = opti.variable(2, obj.N + 1);
            proj_error = X(1, :);
            head_error = X(2, :);
            
            % Control variable (Steering angle)
            Delta = opti.variable(obj.N);
            
            % Cost function to minimize
            % YOUR CODE HERE
            % Use the CasADi function sumsqr to compute sum-of-squares
            J = 0;
            
            opti.minimize(J);
            
            % Simulate the system forwards using RK4 and the implemented
            % error model.
            for k=1:obj.N
                k1 = obj.error_model(X(:, k), v, Delta(k), curvature(k));
                k2 = obj.error_model(X(:, k) + dt / 2 * k1, v, Delta(k), curvature(k));
                k3 = obj.error_model(X(:, k) + dt / 2 * k2, v, Delta(k), curvature(k));
                k4 = obj.error_model(X(:, k) + dt * k3, v, Delta(k), curvature(k));
                x_next = X(:,k) + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4);
                opti.subject_to(X(:, k+1) == x_next);
            end
            
            % Problem constraints.
            opti.subject_to(proj_error(1) == d0);
            opti.subject_to(head_error(1) == th0);
            opti.subject_to(-obj.steer_limit <= Delta <= obj.steer_limit);
            
            % Declare solver.
            opts_struct.print_iter = 0;
            opts_struct.print_time = 0;
            opts_struct.constr_viol_tol = 1e-12;

            % opts_struct.max_iter = 100;
            
            opti.solver('qrqp', opts_struct);
            
            % Cast the optimization problem as a callable function
            optimizer = opti.to_function('f', {d0, th0, v, curvature}, {Delta});
        end
        
        function ret = error_model(obj, w, v, delta, curvature)
            % Error model describing how the distance and 
            % heading error evolve for a certain input.
            % 
            % Input
            %   w = (d, theta_e)
            %   v - velocity
            %   delta - input, steering angle
            %   curvature - current curvature
            %
            % Output
            %   Time derivatives of d and theta_e
            
            % YOUR CODE HERE
            d_dot = 0;
            theta_e_dot = 0;
            
            ret = [d_dot; theta_e_dot];
        end

        function c = u(obj, ~, w)
            p_car = w(1:2);
            theta = w(3);
            v = w(4);
            
            % Compute distance and heading errors
            % YOUR CODE HERE
            d = 0;
            theta_e = 0;
            si = 0;  % Position for start of prediction
            
            % Solve optimization problem over the prediction horizon
            s_end = si + obj.N * v * obj.sample_rate;
            s_horizon = (si:(s_end - si) / (obj.N - 1):s_end);
            
            % YOUR CODE HERE
            % call obj.optimizer() with proper arguments
            % Call syntax:
            %   Delta = full(evalf(obj.optimizer( arguments )));
            Delta = 0; % 
            
            % Collect the controller output
            delta = 0;  % YOUR CODE HERE
            acc = 0;
            
            c = [delta, acc];
        end

        function r = run(obj, ~, w)
            if ~isempty(obj.plan)
                % Function that returns true until goal is reached
                p_car = w(1:2);
                p_goal = obj.plan.path(end, 1:2);
                r = norm(p_car - p_goal) > obj.goal_tol;
            else
                r = false;
            end
        end
    end
end
