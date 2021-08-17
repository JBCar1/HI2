clear
addpath Functions

import casadi.*

%% Simulate a simple path to follow
car = SingleTrackModel();
car.controller = MiniController();
w0 = [0, 0, 0, 2];
[~, w, ~] = car.simulate(w0, 40, 0.1);
fprintf('Total time in controller: %.2f msek\n', car.controller.u_time*1000);

M = 10;  % Path downsample factor
p = w(1:M:end, 1:2);
ref_path = SplinePath(p);

s = linspace(0, ref_path.length, 200);

% Plot resulting path
figure(10)
plot(ref_path.x(s), ref_path.y(s), 'b')
hold on
plot(p(:, 1), p(:, 2), 'rx');
hold off
title('Path from simple controller');
xlabel('x [m]');
ylabel('y [m]');
box off

%% Run the MPC path following controller
% Implement an MPC controller in the ModelPredictiveController skeleton class.
% Parameters for the controller are
%
% gamma_d - Weight in the loss-function for _distance errors_
% gamma_theta - Weight in the loss-function for _heading errors_
% gamma_u - Weight in the loss-function for _control signal_ (steer angle)
% L - wheel base
% steer_limit - Steer limits (in radians) for the control signal

opts.h_p = 10;
opts.gamma_d = 1;
opts.gamma_theta = 1;
opts.gamma_u = 1;
opts.L = car.L;
opts.steer_limit = pi / 4;

car = SingleTrackModel();
car.steer_limit = opts.steer_limit;

mpc = ModelPredictiveController(opts, ref_path, 0.1);
car.controller = mpc;

w0 = [0, 6, pi / 2 * 0.9, 2];
[t, w, u] =  car.simulate(w0, 80, mpc.sample_rate, 0.0);
z_mpc = {t, w, u};
fprintf('Total time in controller: %.2f sek\n', mpc.u_time);
