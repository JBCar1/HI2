#!/usr/bin/env python
# coding: utf-8

# # TSFS12 Hand-in exercise 1: Discrete planning in structured road networks

# Do initial imports of packages needed

import numpy as np
import matplotlib.pyplot as plt
from misc import Timer, latlong_distance
from queues import FIFO, LIFO, PriorityQueue
from osm import load_osm_map


# Activate plots in external windows (needed for mission definition)



# Run the ipython magic below to activate automated import of modules. Useful if you write code in external .py files.
# %load_ext autoreload
# %autoreload 2


# # Load map

osm_file = 'linkoping.osm'
fig_file = 'linkoping.png'
osm_map = load_osm_map(osm_file, fig_file)  # Defaults loads files from ../Maps

num_nodes = len(osm_map.nodes)  # Number of nodes in the map


# Define function to compute possible next nodes from node x and the corresponding distances distances.

def f_next(x):
    """Compute, neighbours for a node"""
    cx = osm_map.distancematrix[x, :].tocoo()
    return cx.col, np.full(cx.col.shape, np.nan), cx.data


# # Display basic information about map

# Print some basic map information

osm_map.info()


# Plot the map

plt.figure(10)
osm_map.plotmap()
plt.xlabel('Longitude')
plt.ylabel('Latitude')
_ = plt.title('Linköping')


# Which nodes are neighbors to node with index 110?

n, _, d = f_next(110)
print(f'Neighbours: {n}')
print(f'Distances: {d}')


# Look up the distance (in meters) between the nodes 110 and 3400 in the distance matrix?

print(osm_map.distancematrix[110, 3400])


# Latitude and longitude of node 110

p = osm_map.nodeposition[110]
print(f'Longitude = {p[0]:.3f}, Latitude = {p[1]:.3f}')


# Plot the distance matrix and illustrate sparseness

plt.figure(20)
plt.spy(osm_map.distancematrix > 0, markersize=0.5)
plt.xlabel('Node index')
plt.ylabel('Node index')
density = np.sum(osm_map.distancematrix > 0) / num_nodes**2
_ = plt.title(f'Density {density * 100:.2f}%')


# # Define planning mission

# Some pre-defined missions to experiment with. To use the first pre-defined mission, call the planner with
# ```planner(num_nodes, pre_mission[0], f_next, cost_to_go)```.

pre_mission = [
    {'start': {'id': 10906}, 'goal': {'id': 1024}},
    {'start': {'id': 3987}, 'goal': {'id': 4724}},
    {'start': {'id': 423}, 'goal': {'id': 364}}]
mission = pre_mission[0]  # Use this line if you want to use the predefined missions


# In the map, click on start and goal positions to define a mission. Try different missions, ranging from easy to more complex. 
# An easy mission is a mission in the city centre; while a more difficult could be from Vallastaden to Tannefors. Use this to find interesting plans when experimenting.

plt.figure(30, clear=True)
osm_map.plotmap()
plt.title('Linköping - click in map to define mission')
mission = {}

mission['start'] = osm_map.getmapposition()
plt.plot(mission['start']['pos'][0], mission['start']['pos'][1], 'bx')
mission['goal'] = osm_map.getmapposition()
plt.plot(mission['goal']['pos'][0], mission['goal']['pos'][1], 'bx')

plt.xlabel('Longitude')
plt.ylabel('Latitude')

print('Mission: Go from node %d ' % (mission['start']['id']), end='')
if mission['start']['name'] != '':
    print('(' + mission['start']['name'] + ')', end='')
print(' to node %d ' % (mission['goal']['id']), end='')
if mission['goal']['name'] != '':
    print('(' + mission['goal']['name'] + ')', end='')
print('')


mission


# # Implement planners

def depth_first(num_nodes, mission, f_next, heuristic=None, num_controls=0):
    """Depth first planner."""
    t = Timer()
    t.tic()
    
    unvis_node = -1
    previous = np.full(num_nodes, dtype=int, fill_value=unvis_node)
    cost_to_come = np.zeros(num_nodes)
    control_to_come = np.zeros((num_nodes, num_controls), dtype=int)
    expanded_nodes = []
    
    startNode = mission['start']['id']
    goalNode = mission['goal']['id']

    q = LIFO()
    q.insert(startNode)
    foundPlan = False

    while not q.IsEmpty():
        x = q.pop()
        expanded_nodes.append(x)
        if x == goalNode:
            foundPlan = True
            break
        neighbours, u, d = f_next(x)
        for xi, ui, di in zip(neighbours, u, d):
            if previous[xi] == unvis_node:
                previous[xi] = x
                q.insert(xi)
                cost_to_come[xi] = cost_to_come[x] + di
                if num_controls > 0:
                    control_to_come[xi] = ui

    # Recreate the plan by traversing previous from goal node
    if not foundPlan:
        return []
    else:
        plan = [goalNode]
        length = cost_to_come[goalNode]
        control = []
        while plan[0] != startNode:
            if num_controls > 0:
                control.insert(0, control_to_come[plan[0]])
            plan.insert(0, previous[plan[0]])

        return {'plan': plan,
                'length': length,
                'num_expanded_nodes': len(expanded_nodes),
                'name': 'DepthFirst',
                'time': t.toc(),
                'control': control,
                'expanded_nodes': expanded_nodes}


# ## Planning example using the DepthFirst planner

# Make a plan using the ```DepthFirst``` planner

df_plan = depth_first(num_nodes, mission, f_next)
print(f"{df_plan['length']:.1f} m, {df_plan['num_expanded_nodes']} expanded nodes, planning time {df_plan['time'] * 1e3:.1f} msek")


# Plot the resulting plan

plt.figure(40, clear=True)
osm_map.plotmap()
osm_map.plotplan(df_plan['plan'], 'b',
                 label=f"Depth first ({df_plan['length']:.1f} m)")
plt.title('Linköping')
_ = plt.legend()


# Plot nodes visited during search

plt.figure(41, clear=True)
osm_map.plotmap()
osm_map.plotplan(df_plan['expanded_nodes'], 'b.')
plt.ylabel('Latitude')
plt.xlabel('Longitude')
_ = plt.title('Nodes visited during DepthFirst search')


# Names of roads along the plan ...

plan_way_names = osm_map.getplanwaynames(df_plan['plan'])
print('Start: ', end='')
for w in plan_way_names[:-1]:
    print(w + ' -- ', end='')
print('Goal: ' + plan_way_names[-1])


# # Define planners

# Here, write your code for your planners. Start with the template code for the depth first search and extend.

def breadth_first(num_nodes, mission, f_next, heuristic=None, num_controls=0):
    pass

def dijkstra(num_nodes, mission, f_next, heuristic=None, num_controls=0):
    pass

def astar(num_nodes, mission, f_next, heuristic=None, num_controls=0):
    pass

def best_first(num_nodes, mission, f_next, heuristic=None, num_controls=0):
    pass


# # Define heuristic for Astar and BestFirst planners

# Define the heuristic for Astar and BestFirst. The ```latlong_distance``` function will be useful.

def cost_to_go(x, xg):
    p_x = osm_map.nodeposition[x]
    p_g = osm_map.nodeposition[xg]
    return 0.0


# # Investigations using all planners



plt.show()
