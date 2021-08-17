classdef SingleTrackModel < VehicleModelBase
    properties
        L = 2;
        amax = inf;
        amin = -inf;
        steer_limit = pi/4;
    end
  
    methods
        function obj=SingleTrackModel(varargin)
        % Create Single track kinematic car
        %
        %   obj = SingleTrackModel(Ts)
        %
        %   Ts - controller sample time
            if nargin < 1
                Ts = 0.1;
            else
                Ts = varargin{1};
            end
            obj = obj@VehicleModelBase(Ts);
        end

        function dw = dx(obj, w, u)
            % Dynamic equation for single-track model
            %
            % dw = dx(w, u)
            %
            % Input: 
            %  w  - state (x, y, theta, v)
            %  u - control input (delta, acceleration)
            %
            theta = w(3);
            v = w(4);
            delta = max(min(u(1), obj.steer_limit), -obj.steer_limit);
            a = max(min(u(2), obj.amax), obj.amin);
            dw = [v * cos(theta), ...
                  v * sin(theta), ...
                  v / obj.L * tan(delta),...
                  a];
        end
    end
end
