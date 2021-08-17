classdef EulerForwardIntegrator < handle
  properties
    f;
    t;
    y;
    Ts;
  end
  methods
    function obj = EulerForwardIntegrator(f, Ts)
      obj.f = f;
      obj.t = 0.0;
      obj.y = 0.0;
      obj.Ts = Ts;
    end
    
    function set_initial_value(obj, y, t)
      if nargin < 3
        t = 0.0;
      end
      obj.t = t;
      obj.y = y;
    end

    function integrate(obj, Tend)
      while obj.t < Tend
        if obj.t + obj.Ts < Tend
            dt = obj.Ts;
        else
            dt = Tend - obj.t;
        end
        obj.y = obj.y + obj.f(obj.t, obj.y)*dt;
        obj.t = obj.t + dt;        
      end
    end
  end
end
