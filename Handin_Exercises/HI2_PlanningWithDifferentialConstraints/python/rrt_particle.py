#!/usr/bin/env python
# coding: utf-8

# # TSFS12 Hand-in Exercise 2: Planning for Vehicles with Differential Motion Constraints --- RRT with Particle Model

import numpy as np
import matplotlib.pyplot as plt
from misc import Timer
from world import BoxWorld


# %matplotlib  # Run instead if you want plots in external windows


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


# # Implementation of RRT

# Implementation of the RRT planning algorithm for a particle moving in a plane (2D world)

def rrt_particle(start, goal, world, opts):
    """RRT planner for particle moving in a 2D world
    
    Input arguments:
     start - initial state
     goal - desired goal state
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
     Tplan - the time taken for computing the plan        
    """
    rg = np.random.default_rng()  # Get the default random number generator

    def sample_free():
        """Sample a state x in the free state space"""
        if rg.uniform(0, 1, 1) < opts['beta']:
            return np.array(goal)
        else:
            found_random = False
            while not found_random:
                x = (rg.uniform(0, 1, 2) * [world.xmax - world.xmin, world.ymax - world.ymin] + 
                     [world.xmin, world.ymin])
                if world.obstacle_free(x[:, None]):
                    found_random = True
            return x

    def nearest(x):
        """Find index of state nearest to x in the matrix nodes"""        
        idx = np.argmin(np.sum((nodes - x[:, None])**2, axis=0))
        return idx
    
    def steer(x1, x2):
        """Steer from x1 towards x2 with step size opts['lambda']
        
        If the distance to x2 is less than opts['lambda'], return
        state x2.
        """
        dx = np.linalg.norm(x2 - x1)
        if dx < opts['lambda']:
            x_new = x2
        else:
            x_new = x1 + opts['lambda'] * (x2 - x1) / dx
        return x_new
    
    # Start time measurement and define variables for nodes and parents
    T = Timer()
    T.tic()
    nodes = start.reshape((-1, 1))  # Make numpy column vector
    parents = [0]   # Initial state has no parent

    # YOUR CODE HERE
    
    Tplan = T.toc()
    goal_idx = np.argmin(np.sum((nodes - goal.reshape((-1, 1)))**2, axis=0))
    
    return goal_idx, nodes, parents, Tplan


# Run the planner

start = np.array([1, 0]) # Start state
goal = np.array([6.5, 9]) # Goal state

opts = {'beta': 0.05,  # Probability of selecting goal state as target state in the sample
        'lambda': 0.1,  # Step size
        'eps': -0.01,  # Threshold for stopping the search (negative for full search)
        'K': 5000}     # Maximum number of iterations, if eps < 0

print('Planning ...')
idx_goal, nodes, parents, Tplan = rrt_particle(start, goal, world, opts)
print('Finished in {:.2f} s'.format(Tplan))


# # Plots and Analysis

# Hint on plotting: To plot the path corresponding to the found solution,
# the following code could be useful (utilizing backtracking from the goal 
# node:
#
# drawlines = []
# idx = idx_goal
# while idx != 0:
#     ll = np.column_stack((nodes[:, parents[idx]], nodes[:, idx]))
#     drawlines.append(ll[0])
#     drawlines.append(ll[1])
#     idx = parents[idx]
# plt.plot(*drawlines, color='b', lw=2)




plt.show()
