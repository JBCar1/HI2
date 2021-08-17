#!/usr/bin/env python
# coding: utf-8

# # TSFS12 Hand-in exercise 1, extra assignment: Discrete planning in structured road networks

# Do initial imports of packages needed

import matplotlib.pyplot as plt
from misc import Timer, latlong_distance
from osm import load_osm_map
import numpy as np


# %matplotlib  # Run if you want plots in external windows


# Run the ipython magic below to activate automated import of modules. Useful if you write code in external .py files.
# %load_ext autoreload
# %autoreload 2


# # Load map, define state transition function, and heuristic for Astar

osm_file = 'linkoping.osm'
fig_file = 'linkoping.png'
osm_map = load_osm_map(osm_file, fig_file)  # Defaults loads files from ../Maps

num_nodes = len(osm_map.nodes)  # Number of nodes in the map


# Define function to compute possible next nodes from node x and the corresponding distances distances.

def f_next(x):
    """Compute, neighbours for a node"""
    cx = osm_map.distancematrix[x, :].tocoo()
    return cx.col, np.full(cx.col.shape, np.nan), cx.data


def heuristic(x, xg):
    # YOUR CODE HERE
    p_x = osm_map.nodeposition[x]
    p_g = osm_map.nodeposition[xg]
    return 0.0


# # Define planning missions with same goal node

# Predefined planning missions with the same goal node that you can use. You are welcome to experiment with other missions.

missions = [
    {'start': {'id': 10906}, 'goal': {'id': 1024}},
    {'start': {'id': 3987}, 'goal': {'id': 1024}},
    {'start': {'id': 423}, 'goal': {'id': 1024}}]


# # Exercises



plt.show()
