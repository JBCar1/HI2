classdef OdeIntegrator < handle
  properties
    f;
    t;
    y;
  end
  methods
    function obj = OdeIntegrator(f)
      obj.f = f;
      obj.t = 0.0;
      obj.y = 0.0;
    end
    
    function set_initial_value(obj, y, t)
      if nargin < 3
        t = 0.0;
      end
      obj.t = t;
      obj.y = y;
    end

    function integrate(obj, Tend)
      [t, y] = ode45(obj.f, [obj.t, Tend], obj.y);
      obj.t = t(end);
      obj.y = y(end, :);
    end
  end
end
