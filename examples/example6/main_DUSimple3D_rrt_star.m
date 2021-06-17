
clc;clear;
map = struct('name', 'my_map.mat', 'start_point', [30 0 0.9], 'goal_point', [0 70 0.9]);
max_iter = 3e3;
is_benchmark = false;
rand_seed = 40;
variant = 'DUSimple3D';
result = rrt_star(map, max_iter, is_benchmark, rand_seed, variant);