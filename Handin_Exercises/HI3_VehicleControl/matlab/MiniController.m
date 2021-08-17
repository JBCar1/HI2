classdef MiniController < ControllerBase
    methods
        function obj = MiniController()
            obj = obj@ControllerBase();
        end

        function r = u(obj, t, w)
            a = 0.0;
            if t < 10
                r = [pi/180*10, a];
            elseif t >= 10 && t < 20
                r = [-pi/180*11, a];
            elseif t >= 20 && t < 23
                r = [-pi/180*0, a];
            elseif t >= 23 && t < 40
                r = [-pi/180*15, a];
            else
                r = [0, a];
            end
        end

        function r = run(obj, t, w)
            r = true;
        end
    end
end

