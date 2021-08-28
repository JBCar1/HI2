#!/usr/bin/env python
# coding: utf-8

# # TSFS12 Hand-in Exercise 2, Extra Assignment: Planning for Vehicles with Differential Motion Constraints --- RRT* with Particle Model

import numpy as np
import matplotlib.pyplot as plt
from misc import Timer
from world import BoxWorld


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
plt.axis([world.xmin, world.xmax, world.ymin, world.ymax])


# # Implementation of RRT* Planning Algorithm

# RRT* for a particle moving in a plane (2D world)

def rrt_star_particle(start, goal, world, opts):
    """ Input arguments:
            start - initial state
            goal - desired goal state
            world - description of the map of the world
                 using an object from the class BoxWorld
            opts - structure with options for the RRT*
    
        Output arguments:
            goal_idx - index of the node closest to the desired goal state
            nodes - 2 x N matrix with each column representing a state j
                 in the tree
            parents - 1 x N vector with the node number for the parent of node j 
               at element j in the vector (node number counted as column
               in the matrix nodes)
            Tplan - the time taken for computing the plan"""
    
    def sample_free():
        """Sample a state x in the free state space"""
        if np.random.uniform(0, 1, 1) < opts['beta']:
            return goal
        else:
            found_random = False
            while not found_random:
                x = np.random.uniform(0, 1, 2) * [world.xmax - world.xmin, world.ymax - world.ymin] + [world.xmin, world.ymin]
                if world.obstacle_free(x[:, None]):
                    found_random = True
        return x

    def nearest(x):
        """Find index of state nearest to x in nodes"""
        idx = np.argmin(np.sum((nodes - x[:, None])**2, axis=0))
        return idx
    
    def near(x, r):
        """Find the indices of the states in nodes within a neighborhood with
            radius r from state x"""
        idx = np.where(np.sum((nodes - x[:, None])**2, axis=0) < r**2)
        return idx[0]

    def steer(x1, x2):
        """Steering function for moving from x1 towards x2 with step size
            opts['lambda']. If the distance to x2 is less than opts['lambda'], x_new
            becomes state x2."""
        dx = sum((x2 - x1)**2)
        if dx < opts['lambda']:
            x_new = x2
        else:
            x_new = x1 + opts['lambda'] * (x2 - x1) / dx
        return x_new
    
    def connect_min_cost(x_new, near_idx, idx_nearest, cost_via_nearest):
        """Function for connecting along a path from tree root to x_new with
            minimum cost among the states in a neighborhood of x_new
            described by the (column) indices near_idx in nodes. The variable
            idx_nearest is the index (column in matrix nodes) for the node 
            closest to x_new and cost_via_nearest is the cost to reach x_new 
            via the nearest node."""
        
        idx_min = idx_nearest
        cost_min = cost_via_nearest

        for idx_n in near_idx:
            x_near = nodes[:, idx_n]

            if (x_new[0] == x_near[0]) and (x_new[1] == x_near[1]):
                p = x_new[:, None]
            else:
                p = np.row_stack((np.arange(x_near[0], x_new[0], (x_new[0] - x_near[0]) / 10),
                                  np.arange(x_near[1], x_new[1], (x_new[1] - x_near[1]) / 10)))
            cost_near = cost[idx_n] + np.linalg.norm(x_near - x_new)

            if cost_near < cost_min and world.obstacle_free(p):
                cost_min = cost_near
                idx_min = idx_n
        return idx_min, cost_min

    def rewire_neighborhood(x_new, near_idx, cost_min):
        """Function for (possible) rewiring of the nodes in the neighborhood of
            x_new described by the indices near_idx in nodes (column numbers) 
            via the new state x_new, if a path with less cost could be found. 
            The variable cost_min is the cost-to-come to x_new 
            (computed in connect_min_cost)"""
        for idx_n in near_idx:
            x_near = nodes[:, idx_n]
            
            if (x_new[0] == x_near[0]) and (x_new[1] == x_near[1]):
                p = x_new[:, None]
            else:
                p = np.row_stack((np.arange(x_near[0], x_new[0], (x_new[0] - x_near[0]) / 10),
                                  np.arange(x_near[1], x_new[1], (x_new[1] - x_near[1]) / 10)))
            cost_near = cost_min + np.linalg.norm(x_near - x_new)
            if cost_near < cost[idx_n] and world.obstacle_free(p):
                parents[idx_n] = len(parents) - 1
                cost[idx_n] = cost_near
    
    # Start time measurement and define variables for nodes, parents, and 
    # associated cost
    T = Timer()
    T.tic()
    nodes = start.reshape((-1, 1))
    parents = np.array([0], dtype=np.int)
    
    # cost - 1 x N vector with the cost for reaching node j from the
    #        initial state (tree root) at element j in the vector 
    #        (node number counted as column in the matrix nodes)
    cost = np.array([0])
    
    # YOUR CODE HERE
    
    Tplan = T.toc()
    goal_idx = np.argmin(np.sum((nodes-goal[:, None])**2, axis=0))
    return goal_idx, nodes, parents, Tplan


# Run the planner

start = np.array([1, 0])
goal = np.array([6.5, 9])

opts = {'beta': 0.01,  # Probability of selecting goal state as target state
        'lambda': 0.1,  # Step size
        'eps': -0.01,  # Threshold for stopping the search (negative for full search)
        'r_neighbor': 0.5, # Radius of circle for definition of neighborhood
        'K': 10000}  # Maximum number of iterations

goal_idx, nodes, parents, Tplan = rrt_star_particle(start, goal, world, opts)
print('Finished in {:.3f} s'.format(Tplan))




plt.show()
