#!/usr/bin/env python
# coding: utf-8

# # TSFS12 Hand-in Exercise 5: Learning for autonomous vehicles --- Reinforcement learning and value iteration

# Value iteration for solving an extension of Example 6.6 in
# Sutton, R. S., & A. G. Barto: Reinforcement learning: An introduction.
# MIT Press, 2018.

# Initial imports

import matplotlib.pyplot as plt
import numpy as np
from itertools import product
from grid_world import plot_value_and_policy, p_grid_world


# Run if you want plots in external windows


# # Define the parameters for the Markov Decision Process

gamma = 0.99   # Discount factor
R_goal = 0.0   # Reward for reaching goal state
R_sink = -10.0   # Reward for reaching 'cliff' states
R_grid = -0.1   # Reward for remaining states

P_move_action = 0.99 # Probability of moving in the direction specified by action
P_dist = (1 - P_move_action) / 2  # Probability of moving sideways compared to 
                                  # intended because of disturbance

# Define size of grid world, goal, and cliff states

n_rows = 4
n_cols = 5

goal = (3, 4)   # Element index goal state
sink = [(3, 1), (3, 2), (3, 3)]   # Element indices for cliff states

# Setup reward matrix R

R = np.full((n_rows, n_cols), fill_value=R_grid)
R[goal[0], goal[1]] = R_goal
for p in sink:
    R[p[0], p[1]] = R_sink

# Occupancy grid defines states where there are obstacles (0 - no
# obstacles, 1 - obstacles)

occ_grid = np.zeros((n_rows, n_cols), dtype=int)
occ_grid[1, 1] = 1

# Save all parameters in a dictionary
params = {'gamma': gamma, 'R_goal': R_goal, 'R_sink': R_sink,
          'R_grid': R_grid, 'P_move_action': P_move_action,
          'P_dist': P_dist, 'n_rows': n_rows, 'n_cols': n_cols,
          'goal': goal, 'sink': sink, 'R': R, 'occ_grid': occ_grid}


# # Main loop for value iterations

# Press return to proceed to next iteration. Note that plots must be made in an external window, not inline, since value function and policy are updated in the `plot_iter` function call.

# Initialize the main learning loop

# Initialize variables for value function V and policy Pi for each state s
V = np.zeros((n_rows, n_cols))
# Actions - ['left', 'right', 'up', 'down'] counted as 0-3
Pi = np.full((n_rows, n_cols), fill_value=-1)

converged = False
num_iterations = 0


# Algorithm according to Section 4.4 in Sutton, R. S., & A. G. Barto: 
# Reinforcement learning: An introduction. MIT Press, 2018.

# Continue the iterations until convergence criterion is fulfilled
while not converged:
    Delta = 0
    # Go through all states
    for s in product(range(n_rows), range(n_cols)):
        row, col = s
        # Check if obstacle or sink states, then continue to next state
        if ((occ_grid[row, col] == 1) or s == goal or s in sink):
            continue
        v = V[row, col]
        V_a = np.zeros(4)
        for a in range(4):  # Go through all possible actions
            for s_prim, r, p_prim in p_grid_world(s, a, params):  
                # Three state transitions are possible in 
                # each state, sum the terms for each of them
                V_prim = V[s_prim[0], s_prim[1]]
                V_a[a] += p_prim * (r + params['gamma'] * V_prim)
                
        # Compute new value function V for current state and update
        # policy Pi
        max_a = np.argmax(V_a)
        Pi[row, col] = max_a
        V[row, col] = V_a[max_a]

        # Compute improvement criterion for V
        Delta = np.max((Delta, np.abs(v - V[row, col])))

    # Visualize the current value function V and policy Pi in the grid
    # world
    plot_value_and_policy(V, Pi, params)

    num_iterations += 1

    # Display current value function V and policy Pi
    print(V)
    print(Pi)
    print('Press enter')
    _ = input()

    # If improvement criterion below threshold, stop the value iteration
    # since convergence has been achieved
    if Delta < 1e-6:
        converged = True
print(f'Convergence after {num_iterations} iterations')







plt.show()
