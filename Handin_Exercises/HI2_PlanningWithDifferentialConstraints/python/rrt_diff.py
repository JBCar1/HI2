#!/usr/bin/env python
# coding: utf-8

# # TSFS12 Hand-in Exercise 2: Planning for Vehicles with Differential Motion Constraints --- RRT with Motion Model for a Simple Car

import numpy as np
import matplotlib.pyplot as plt
from misc import Timer
from world import BoxWorld
import math


# Run instead if you want plots in external windows
# %matplotlib


# Run the ipython magic below to activate automated import of modules. Useful if you write code in external .py files.
# %load_ext autoreload
# %autoreload 2


# # Define the Planning World

# Define world with obstacles

world = BoxWorld([[0, 10], [0, 10]])
world.add_box(0, 1, 2, 4)
world.add_box(0, 6, 6, 4)
world.add_box(4, 1, 6, 4)
world.add_box(7, 7, 3, 3)

plt.figure(10, clear=True)
world.draw()
plt.xlabel('x')
plt.ylabel('y')
plt.axis([world.xmin, world.xmax, world.ymin, world.ymax])


# # Car Simulation Function

# Define function needed to simulate motion of single-track model

def sim_car(xk, u, step, h=0.01, L=1.5, v=15):
    """Car simulator
    
    Simulate car forward in time from state xk with time-step length step. 
    Returns next sequence of states.
    
    x' = v*cos(th)
    y' = v*sin(th)
    th' = v*tan(delta)/L
    u = delta
    x = [x y th]
    """

    # Simulation with discretization using forward Euler

    t = 0
    N = int(step / h) + 1
    states = np.zeros((3, N))
    states[:, 0] = xk

    k = 0
    while k < N - 1:
        hk = min(h, step - t)
        states[0, k + 1] = states[0, k] + hk * v * math.cos(states[2, k])
        states[1, k + 1] = states[1, k] + hk * v * math.sin(states[2, k])
        states[2, k + 1] = states[2, k] + hk * v * math.tan(u) / L
        t = t + h
        k = k + 1

    return states   


# # Implementation of RRT for Kinematic Car Model
# 
# Car model has two translational and one orientational degrees-of-freedom

def rrt_diff(start, goal, u_c, sim, world, opts):
    """RRT planner for kinematic car model
    
    Input arguments:
     start - initial state
     goal - desired goal state
     u_c - vector with possible control actions (steering angles)
     sim - function reference to the simulation model of the car motion
     world - description of the map of the world
             using an object from the class BoxWorld
     opts - structure with options for the RRT
    
    Output arguments:
     goal_idx - index of the node closest to the desired goal state
     nodes - 2 x N matrix with each column representing a state j
             in the tree
     parents - 1 x N vector with the node number for the parent of node j 
               at element j in the vector (node number counted as column
               in the matrix nodes)
     state_trajectories - a struct with the trajectory segment for 
                     reaching node j at element j (node number counted 
                     as column in the matrix nodes)
     Tplan - the time taken for computing the plan   
    """

    rg = np.random.default_rng()
    
    def sample_free():
        """Returns a state x in the free state space"""
        if rg.uniform(0, 1) < opts['beta']:
            return np.array(goal)
        else:
            found_random = False
            th = rg.uniform(0, 1) * 2 * np.pi - np.pi
            while not found_random:
                p = (rg.uniform(0, 1, 2) * [world.xmax - world.xmin, world.ymax - world.ymin] + 
                     [world.xmin, world.ymin])
                if world.obstacle_free(p[:, None]):
                    found_random = True
        return np.array([p[0], p[1], th])

    def nearest(x):
        """Find index of state nearest to x in nodes"""
        return np.argmin(distance_fcn(nodes, x[:, None]))
    
    def steer_candidates(x_nearest, x_rand):
        """Compute all possible paths for different steering control signals u_c
            to move from x_nearest towards x_rand, without collision
            If no valid paths are found, the returned variables are empty"""
        new_paths = [sim(x_nearest, ui, opts['lambda']) for ui in u_c]
        new_free = np.where([world.obstacle_free(traj_i) for traj_i in new_paths])[0]
        valid_new_paths = [new_paths[i] for i in new_free]
        
        if np.any(new_free):
            dist_to_x_rand = [distance_fcn(xi[:, -1], x_rand) for xi in valid_new_paths]
        else:
            dist_to_x_rand = -1

        return valid_new_paths, dist_to_x_rand

    def distance_fcn(x1, x2):
        """Function for computing the distance between states x1 and x2, where 
            x1 and x2 can be matrices with several state vectors, treating  
            all states equally"""
        d2 = x1 - x2
        return np.sqrt(d2[0]**2 + d2[1]**2)

    # Start time measurement and define variables for nodes, parents, and 
    # associated trajectories
    T = Timer()
    T.tic()
    nodes = start.reshape((-1, 1))   # Make numpy column vector
    parents = [0]    # Initial state has no parent
    state_trajectories = [start]   # No trajectory segment needed to reach start state

    # YOUR CODE HERE
    
    Tplan = T.toc()
    goal_idx = np.argmin(distance_fcn(nodes, goal[:, None]), axis=0)
    return goal_idx, nodes, parents, state_trajectories, Tplan


# Run the planner

start = np.array([1, 0, np.pi/4]) # Start state (x,y,th)
goal = np.array([6.5, 9, np.pi/2]) # Goal state (x,y,th)

# Define the possible control inputs
u_c = np.linspace(-np.pi / 4, np.pi / 4, 11)

# Define parameters and data structures

opts = {'beta': 0.05,  # Probability of selecting goal state as target state
        'lambda': 0.1,  # Step size (in time)
        'eps': -0.01,  # Threshold for stopping the search (negative for full search)
        'K': 4000}    # Maximum number of iterations

goal_idx, nodes, parents, state_trajectories, Tplan = rrt_diff(start, goal, u_c, sim_car, world, opts)
print(f'Finished in {Tplan:.2f} s')


# # Plots and Analysis

# Hint on plotting: To plot the path corresponding to the found solution,
# the following code could be useful (utilizing backtracking from the goal 
# node:
# drawlines = []
# idx = goal_idx
# while idx != 0:
#     traj_i = state_trajectories[idx]
#     drawlines.append(traj_i[0])
#     drawlines.append(traj_i[1])
#     idx = parents[idx]
# plt.plot(*drawlines, color='b', lw=4)




plt.show()
