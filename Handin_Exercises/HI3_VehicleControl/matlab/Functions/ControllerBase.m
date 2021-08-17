classdef (Abstract) ControllerBase < handle
  properties
    u_time = 0.0;    
  end
  
  methods
    function obj = ControllerBase()
      obj.u_time = 0.0;
    end
    
    function u = u_timed(obj, t, w)
      tic();
      u = obj.u(t, w);
      obj.u_time = obj.u_time + toc();
    end
    
    function r = run(obj, t, w)
      r = true;
    end
  end
  
  methods (Abstract)
    r = u(obj, t, w);
  end
end
