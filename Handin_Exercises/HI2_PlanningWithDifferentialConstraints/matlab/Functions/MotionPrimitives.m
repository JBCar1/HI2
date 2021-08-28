classdef MotionPrimitives < handle
    % Class for motion primitives, computed given a certain pre-defined 
    % state lattice, or loaded from file if already computed.
    properties
        mprims;
        lattice;
        theta_init;
        L;
        v;
        u_max;
    end

    methods
        function obj = MotionPrimitives(filename)
            if nargin > 0
                obj.load(filename);
            end
        end
                
        function save(obj, filename)
            % Save the motion primitives in a file to avoid re-computing
            % them next time they are used
            mp.mprims = obj.mprims;
            mp.lattice = obj.lattice;
            mp.theta_init = obj.theta_init;
            mp.L = obj.L;
            mp.v = obj.v;
            mp.u_max = obj.u_max;
            
            save(filename, 'mp');
        end
        
        function load(obj, filename)
            % Load the motion primitives from file if they have been
            % computed already
            mp = load(filename);
            obj.mprims = mp.mp.mprims;
            obj.lattice = mp.mp.lattice;
            obj.theta_init = mp.mp.theta_init;
            obj.L = mp.mp.L;
            obj.v = mp.mp.v;
            obj.u_max = mp.mp.u_max;            
        end
        
        function generate_primitives(obj, theta_init, lattice, L, v, u_max)
           % Compute the specified motion primitives using optimization
           if nargin < 6
                u_max = pi/4; % Maximum steering angle
           end
           if nargin < 5
                L = 1.5; % Wheel base
           end
           if nargin < 4
                v = 15; % Constant velocity
           end
           obj.mprims = compute_mprims(theta_init, lattice, L, v, u_max);
           obj.lattice = lattice;
           obj.theta_init = theta_init;
           obj.L = L;
           obj.v = v;
           obj.u_max = u_max;
        end
        
        function plot(obj)
            % Plot the motion primitives for visualization
            colors = {'b', 'k', 'g', 'm'};
            ih = ishold;
            hold on
            for th_i=1:numel(obj.mprims)
                for mp_i=1:numel(obj.mprims{th_i})
                    plot(obj.mprims{th_i}{mp_i}.x, obj.mprims{th_i}{mp_i}.y, ...
                         colors{mod(th_i-1, 4)+1}, 'LineWidth', 2);
                end
            end
            if ~ih
                hold off
            end
        end
        
        function [p, sp] = plan_to_path(obj, start, plan)
            % Compute the path p from state start obtained for the motion
            % primitives specified by the plan. The output sp describes which parts of
            % the path that are driving forward and which parts that are driving
            % backwards. The matrix has the form
            %  [u_1, start_1, end_1;
            %   u_2, start_2, end_2;
            %   ...]
            % where u_i is 1 for forward and -1 for reverse. Start_i and
            % end_i are indices into the path p. 
            %
            % For example, to plot the k:th segment in blue when driving forward 
            % and red when reversing, you can write
            %
            %  segment_color = "b";
            %  if sp(k, 1) == -1
            %      segment_color = "r";
            %  end
            %  segment_range = sp(k, 2):sp(k, 3);
            %  plot(p(1, segment_range), p(2, segment_range), segment_color);    
            
            [p, sp] = obj.control_to_path(start, plan.control);
        end
        
        function [p, sp] = control_to_path(obj, start, control)
            % Compute the path p from state start obtained for the motion
            % primitives specified by their two indices in the variable
            % control. The output sp describes which parts of
            % the path that are driving forward and which parts that are driving
            % backwards. The matrix has the form
            %  [u_1, start_1, end_1;
            %   u_2, start_2, end_2;
            %   ...]
            % where u_i is 1 for forwards and -1 for reverse. Start_i and
            % end_i are indices into the path p. 
            %
            % For example, to plot the k:th segment in blue when driving forward 
            % and red when reversing, you can write
            %
            %  segment_color = "b";
            %  if sp(k, 1) == -1
            %      segment_color = "r";
            %  end
            %  segment_range = sp(k, 2):sp(k, 3);
            %  plot(p(1, segment_range), p(2, segment_range), segment_color);    
            
            p0 = start;
            p = [];

            u_current = control(1, 3);
            sp = [];
            curr_segment = [u_current, 1];
                       
            for ii=1:size(control, 1)
                mp_ii = obj.mprims{control(ii, 1)}{control(ii, 2)};
                if control(ii, 3) == 1   % Forward driving
                    p1 = [p0(1:2); 0] + [mp_ii.x; mp_ii.y; mp_ii.th];
                else   % Reverse driving
                    p1 = [p0(1:2); 0] + [fliplr(mp_ii.x)-mp_ii.x(end);
                        fliplr(mp_ii.y)-mp_ii.y(end); 
                        fliplr(mp_ii.th)];
                end
                
                if control(ii, 3) ~= u_current
                    curr_segment = [curr_segment size(p, 2)];
                    sp = [sp; curr_segment];
                    u_current = control(ii, 3);
                    curr_segment = [u_current, size(p, 2) + 1];
                end
                
                p0 = p1(:, end);
                p = [p p1];
            end

            curr_segment = [curr_segment size(p, 2)];
            sp = [sp; curr_segment];
        end
    end
end

function mprims = compute_mprims(theta_init, lattice, L, v, u_max)
    % Compute motion primitives with minimum path length for the 
    % kinematic car model using CasADi 

    dth = 2*pi/length(theta_init);
    
    % Generate the motion primitives
    mprims{1} = compute_mprim([0; 0; theta_init(1)], lattice, L, v, u_max);

    xy_vec_th_pi_4 = sqrt(2)*[cos(dth) -sin(dth); sin(dth) cos(dth)]*lattice(1:2, :);
    lattice_pi_4 = [xy_vec_th_pi_4; lattice(3, :) + pi/4];
    
    mprims{2} = compute_mprim([0; 0; theta_init(2)], lattice_pi_4, L, v, u_max);

    % The remaining primitive sets are computed based on the two previous sets

    for i = 3:length(theta_init)
        if mod(i, 2) == 0
            % Not multiple of pi/2
            mprims{i} = mprims{2};
            th = dth*(i-1)-pi/4;
        else
            % Multiple of pi/2
            mprims{i} = mprims{1};
            th = dth*(i-1);
        end
        % Rotate the previous set of primitives
        for j = 1:size(lattice, 2)
            xy_vec_rot = [cos(th) -sin(th); sin(th) cos(th)]* ...
                [mprims{i}{j}.x; mprims{i}{j}.y];
            mprims{i}{j}.x = xy_vec_rot(1, :);
            mprims{i}{j}.y = xy_vec_rot(2, :);
            mprims{i}{j}.th = mprims{i}{j}.th + th;
            % Fix orientation to interval [-pi,pi]
            for k = 1:length(mprims{i}{j}.th)
                if mprims{i}{j}.th(k) > pi
                    mprims{i}{j}.th(k) = mprims{i}{j}.th(k)-2*pi;
                elseif mprims{i}{j}.th(k) < -pi
                    mprims{i}{j}.th(k) = mprims{i}{j}.th(k)+2*pi;
                end
            end
        end
    end

end

function mprim = compute_mprim(state_i, lattice, L, v, u_max)
    % Compute the motion primitive using optimization with the tool CasADi
    % using direct collocation for discretization of the continuous-time
    % motion equations.

    % Parameters for collocation
    N = 75; % Number of elements
    nx = 3; % Degree of state vector
    Nc = 3; % Degree of interpolation polynomials

    x_vec = lattice(1, :);
    y_vec = lattice(2, :);
    th_vec = lattice(3, :);
    
    % Formulate the optimization problem for minimum path length using CasADi

    import casadi.*

    for i = 1:length(x_vec)
        
        % Use the opti interface in CasADi
        opti = casadi.Opti();

        state_f = [x_vec(i) y_vec(i) th_vec(i)]';

        % Define optimization variables and motion equations
        x = MX.sym('x',nx);
        u = MX.sym('u');

        f = Function('f',{x, u}, {v*cos(x(3)), v*sin(x(3)), v*tan(u)/L});
        X = opti.variable(nx,N+1);
        pos_x = X(1, :);
        pos_y = X(2, :);
        ang_th = X(3, :);

        U = opti.variable(N, 1);
        T = opti.variable(1);

        % Set the element length (with final time T unknown, and thus an 
        % optimization variable)
        dt = T/N;

        % Set initial guess values of variables
        opti.set_initial(T, 0.1);
        opti.set_initial(U, 0.0*ones(N, 1));
        opti.set_initial(pos_x, linspace(state_i(1), state_f(1), N+1));
        opti.set_initial(pos_y, linspace(state_i(2), state_f(2), N+1));

        % Define collocation parameters
        tau = collocation_points(Nc, 'radau');
        [C,~] = collocation_interpolators(tau);

        % Formulate collocation constraints

        for k = 1:N  % Loop over elements
            Xc = opti.variable(nx, Nc);
            X_kc = [X(:, k) Xc];
            for j = 1:Nc
                % Make sure that the motion equations are satisfied at
                % all collocation points
                [f_1, f_2, f_3] = f(Xc(:, j), U(k));
                opti.subject_to(X_kc*C{j+1}' == dt*[f_1; f_2; f_3]);
            end
            % Continuity constraints for states between elements
            opti.subject_to(X_kc(:, Nc+1) == X(:, k+1));
        end

        % Input constraints
        for k = 1:N
            opti.subject_to(-u_max <= U(k) <= u_max);
        end

        % Initial and terminal constraints
        opti.subject_to(T >= 0.001);
        opti.subject_to(X(:, 1) == state_i);
        opti.subject_to(X(:, end) == state_f);

        % Formulate the cost function
        alpha = 1e-2;
        opti.minimize(T + alpha*sumsqr(U));

        % Choose solver ipopt and solve the problem
        opti.solver('ipopt', struct('expand', true), struct('tol', 1e-8));
        sol = opti.solve();

        % Extract solution trajectories and store them in the mprim variable
        pos_x_opt = sol.value(pos_x);
        pos_y_opt = sol.value(pos_y);
        ang_th_opt = sol.value(ang_th);
        u_opt = sol.value(U);
        T_opt = sol.value(T);

        mprim{i}.x = pos_x_opt;
        mprim{i}.y = pos_y_opt;
        mprim{i}.th = ang_th_opt;
        mprim{i}.u = u_opt;
        mprim{i}.T = T_opt;
        mprim{i}.ds = T_opt*v;
    end
end