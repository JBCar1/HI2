function states = sim_car(xk, u, step, L, v)

% Simulate car forward in time from state xk with time-step length step

% dot x = v*cos(th)
% dot y = v*sin(th)
% dot th = v*tan(delta)/L
% u = delta
% x = [x y th]'

if nargin < 5
    v = 15;
end
if nargin < 4
    L = 1.5;
end

% Simulation with discretization using forward Euler

t = 0;
h = 0.01;
states = xk;

while t < step
    h = min(h,step-t);
    if abs(h) < 1e-8
        break;
    end
    xdot = [v*cos(xk(3)); v*sin(xk(3)); v*tan(u)/L];
    xk = xk + h*xdot;
    states = [states xk];
    t = t + h;
end


end