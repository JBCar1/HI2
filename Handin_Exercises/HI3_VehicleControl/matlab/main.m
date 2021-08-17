clear
addpath Functions

%% Define a simple vehicle controller, simulate, and create a reference path
car = SingleTrackModel();  % Create car object
car.controller = MiniController();  % Create controller object and assign to car
w0 = [0, 0, 0, 2];  % Initial state (x, y, theta, v)
[t, w, u] = car.simulate(w0, 40, 0.1);  % Simulate closed-loop system

z_simple = {t, w, u};  % Save results
fprintf('Total time in controller: %.2f msek\n', car.controller.u_time*1000);

M = 10;  % Path downsample factor
p = w(1:M:end, 1:2);
ref_path = SplinePath(p); %% Create path from MiniController output

s = linspace(0, ref_path.length, 200);

% Plot resulting paths and control signals
figure(10)
plot(ref_path.x(s), ref_path.y(s), 'b')
hold on
plot(p(:, 1), p(:,2), 'rx');
hold off
title('Path from simple controller');
xlabel('x [m]');
ylabel('y [m]');

figure(11)
subplot(1, 2, 1)
plot(t, u(:,1)*180/pi)
xlabel('t [s]')
ylabel('steer [deg]');
title('Steer');
box off

subplot(1, 2, 2)
plot(t, u(:,2))
xlabel('t [s]')
ylabel('Acceleration [m/s^2]')
title('Acceleration');
box off

%% Pure pursuit controller
% Create vehicle and controller objects
car = SingleTrackModel();
pp_controller = PurePursuitController(4, car.L, ref_path);
car.controller = pp_controller;

w0 = [0, 1, pi/2*0.9, 2];  % Sample starting state

% YOUR CODE HERE


%% LQR Path tracker with linear feedback

% YOUR CODE HERE


%% LQR Path tracker with non-linear feedback

% YOUR CODE HERE
