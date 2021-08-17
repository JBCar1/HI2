% TSFS12 Hand-in exercise 1, extra assignment: Discrete planning in 
% structured road networks

clear
close all

addpath Functions

%% Read map information
mapfile = 'linkoping.osm'; 
figurefile = 'linkoping.png';
osm_map = load_osm_map(mapfile, figurefile);

num_nodes = osm_map.nodes.size(1); % Number of nodes in the map

%% Some pre-defined missions to experiment with. 
% Example, the first mission can be used in a planner with
%   planner(num_nodes, missions{1}, f_next, cost_to_go)

missions = {...
    struct('start', struct('id', 10907), 'goal', struct('id', 1025)), ...
    struct('start', struct('id', 3988), 'goal', struct('id', 1025)), ...
    struct('start', struct('id', 424), 'goal', struct('id', 1025))};

%% Exercises
