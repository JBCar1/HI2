classdef (Abstract) VehicleModelBase < handle
    properties
        integrator;
        u;
        controller;    
    end
    methods
        function obj=VehicleModelBase(Ts)
    %      obj.integrator = OdeIntegrator(@(t,w) obj.dx(w, obj.u));
            obj.integrator = EulerForwardIntegrator(@(t,w) obj.dx(w, obj.u), Ts);
        end
    
        function set_state(obj, w0, t0)
            if nargin < 2
                t0 = 0.0;
            end
            obj.integrator.set_initial_value(w0, t0)
        end
    
        function simulate_step(obj, Tend)
            obj.integrator.integrate(Tend);
        end

        function r = t(obj)
            r = obj.integrator.t;
        end

        function r = w(obj)
            r = obj.integrator.y;
        end   

        function [t, sim, u] = simulate(obj, w0, T, dt, t0)
            if nargin < 5
                t0 = 0;
            end
            if ~isa(obj.controller, 'ControllerBase')
                error('A controller need to be assigned to the vehicle before simulating');
            end

            obj.controller.u_time = 0.0;
            obj.set_state(w0, t0);

            sim = [];
            t = [];
            u = [];

            while obj.t < T && obj.controller.run(obj.t, obj.w)
                obj.u = obj.controller.u_timed(obj.t, obj.w);
                u(end+1, :) = obj.u;
                sim(end+1, :) = obj.w;
                t(end+1) = obj.t;
                obj.simulate_step(obj.t + dt);
            end
            u(end+1, :) = obj.controller.u(obj.t, obj.w);
            sim(end+1, :) = obj.w;
            t(end+1) = obj.t;      
        end
        
        function [t, sim, u] = viz_simulate(obj, fig, w0, T, dt, t0, sleep_time)
            if ~isa(obj.controller, 'ControllerBase')
                error('A controller need to be assigned to the vehicle before simulating');
            end

            if nargin < 7
                sleep_time = -1;
            end
            
            obj.controller.u_time = 0.0;
            obj.set_state(w0, t0);

            sim = [];
            t = [];
            u = [];
            
            hold_state = ishold;
            hold on
            car_pos = [w0(1), w0(2)];
            
            car_pos_h = plot(car_pos(1), car_pos(2), 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 6);

            if ~hold_state
                hold off
            end

            while obj.t < T && obj.controller.run(obj.t, obj.w)
                obj.u = obj.controller.u_timed(obj.t, obj.w);
                u(end+1, :) = obj.u;
                sim(end+1, :) = obj.w;
                t(end+1) = obj.t;
                obj.simulate_step(obj.t + dt);
                w_car = obj.w();
                car_pos_h.XData = w_car(1);
                car_pos_h.YData = w_car(2);
                
                if sleep_time > 0
                    pause(sleep_time);
                end
            end
            u(end+1, :) = obj.controller.u(obj.t, obj.w);
            sim(end+1, :) = obj.w;
            t(end+1) = obj.t;      
        end
    end
    
    methods (Abstract)
        dw = dx(obj, w, u)
    end
end
