#!/usr/bin/env python
# coding: utf-8

# # TSFS12 Hand-in exercise 3: Path following for autonomous vehicles

import numpy as np
import matplotlib.pyplot as plt
from vehiclecontrol import ControllerBase, SingleTrackModel
from splinepath import SplinePath
from scipy.linalg import solve_discrete_are


# %matplotlib  # Run if you want plots in external windows


# Run the ipython magic below to activate automated import of modules. Useful if you write code in external .py files.
# %load_ext autoreload
# %autoreload 2


# # Make a simple controller and simulate vehicle

class MiniController(ControllerBase):
    def __init__(self):
        super().__init__()
    
    def u(self, t, w):
        a = 0.0
        if t < 10:
            u = [np.pi / 180 * 10, a]
        elif 10 <= t < 20:
            u = [-np.pi / 180 * 11, a]
        elif 20 <= t < 23:
            u = [-np.pi / 180 * 0, a]
        elif 23 <= t < 40:
            u = [-np.pi / 180 * 15, a]
        else:
            u = [-np.pi / 180 * 0, a]
        return u

opts = {'L': 2, 
        'amax': np.inf,
        'amin': -np.inf,
        'steer_limit': np.pi / 3}

car = SingleTrackModel().set_attributes(opts)
car.Ts = 0.1
car.controller = MiniController()
w0 = [0, 0, 0, 2]
z0 = car.simulate(w0, T=40, dt=0.1, t0=0.0)
t, w, u = z0
M = 10
p = w[::M, 0:2]
pl = SplinePath(p)


s = np.linspace(0, pl.length, 100)

plt.figure(10, clear=True)
plt.plot(pl.x(s), pl.y(s))
plt.plot(p[:, 0], p[:, 1], 'rx')
plt.xlabel('x [m]')
plt.ylabel('y [m]')
plt.title('Path from simple controller')
plt.axis('square')

plt.figure(11, clear=True)
plt.subplot(1, 2, 1)
plt.plot(t, u[:, 0] * 180 / np.pi)
plt.xlabel('t [s]')
plt.ylabel('steer [deg]')
plt.title('Steer')

plt.subplot(1, 2, 2)
plt.plot(t, u[:, 1])
plt.xlabel('t [s]')
plt.ylabel('acceleration [m/s^2]')
plt.title('Acceleration')
plt.tight_layout()


# # Pure pursuit controller

class PurePursuitController(ControllerBase):
    def __init__(self, l, L, path=None, goal_tol=0.25):
        super().__init__()
        self.plan = path
        self.l = l
        self.L = L
        self.s = 0
        self.goal_tol = goal_tol
        
    def pursuit_point(self, p_car):
        """ Return pure-pursuit given the position of the car.
        
          Input:
            p_car - Car position in global coordinates
            
          Return:
            pursuit point in global coordinates
        """
        # p_car - position of vehicle

        path_points = self.plan.path  # Points on the path
        l = self.l  # Pure-pursuit look-ahead

        # Your code here

        # Hint: It is typically not important to find a point at _exactly_ distance l, 
        #       for example search pure-pursuit point among the points in path_points
        #       but don't forget to take into account the approximate pursuit-horizon when
        #       computing the steering angle.
        p_purepursuit = [0, 0]
        return p_purepursuit
    
    def pure_pursuit_control(self, dp, theta):
        """ Compute pure-pursuit steer angle.
        
          Input:
            dp - Vector from position of car to pursuit point
            theta - heading of vehicle
          
          Output:
            return steer angle
        """

        # Your code here to compute new steering angle
        delta = 0
        return delta

    def u(self, t, w):
        """ Compute control action
        
          Input:
            t - current time
            w - current state w = (x, y, theta, v)
          
          Output:
            return (delta, acc) where delta is steer angle and acc acceleration
        """
        x, y, theta, v = w
        p_car = np.array([x, y])

        # Your code here to compute steering angle, use the functions
        # obj.pursuit_point() and obj.pure_pursuit_control() you 
        # have written above.

        delta = 0
        acc = 0

        return np.array([delta, acc])
    
    def run(self, t, w):
        # Function that returns true until goal is reached
        p_goal = self.plan.path[-1, :]
        p_car = w[0:2]
        dp = p_car - p_goal
        dist = dp.dot(dp)        
        
        return dist > self.goal_tol


car = SingleTrackModel()
pp_controller = PurePursuitController(l=4, L=car.L, path=SplinePath(p))
car.controller = pp_controller
w0 = [0, 1, np.pi / 2 * 0.9, 2]  # Sample starting state

# YOUR CODE HERE


# # State feedback controller based on the linearized path

# Implement linear and non-linear state feedback control.

class StateFeedbackController(ControllerBase):
    def __init__(self, K, L, path=None, goal_tol=1.):
        super().__init__()
        self.plan = path
        self.K = K
        self.goal_tol = goal_tol
        self.d = []
        self.L = L
        self.s0 = 0

    def heading_error(self, theta, s):
        """Compute theta error
        Inputs
            theta - current heading angle
            s - projection point on path
            
        Outputs
            theta_e - heading error angle
        """
            
        # YOUR CODE HERE
        theta_e = 0.0
        return theta_e

    def u(self, t, w):
        x, y, theta, v = w
        p_car = w[0:2]

        # Compute d and theta_e errors. Use the SplinePath method project
        # and the obj.heading_error() function you've written above

        # YOUR CODE HERE
        d = 0
        theta_e = 0

        # Compute control signal delta
        acc = 0  # Constant speed
        delta = 0  # Steering angle

        return np.array([delta, acc])
    
    def run(self, t, w):
        p_goal = self.plan.path[-1, :]
        p_car = w[0:2]
        dp = p_car - p_goal
        dist = np.sqrt(dp.dot(dp))
        if dist < self.goal_tol:
            return False
        else:
            return True

plt.show()
