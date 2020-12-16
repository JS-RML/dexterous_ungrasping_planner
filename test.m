clc;
clear;
map = struct('name', 'my_map.mat', 'start_point', [45 0 60], 'goal_point', [0 90 30]);
max_iter = 3e3;
is_benchmark = false;
rand_seed = 40;
variant = 'DUSimple3D';
MAX_NODES   = max_iter;
MAX_ITER    = max_iter;
RAND_SEED   = rand_seed;
MAP = map;
run([pwd '/configure_' variant '.m']);
CONF = conf;
ALGORITHM = 'RRTS';

problem = eval([variant '(RAND_SEED, MAX_NODES, MAP, CONF);']);
for i=1:10
    new_node = problem.sample();
    nearest_node_ind = problem.nearest(new_node);
    new_node = problem.steer(nearest_node_ind, new_node);
    problem.obstacle_collision(new_node, nearest_node_ind);
    neighbors = problem.neighbors(new_node, nearest_node_ind);
    min_node_ind = problem.chooseParent(neighbors, nearest_node_ind, new_node);
    new_node_ind = problem.insert_node(min_node_ind, new_node);
    problem.rewire(new_node_ind, neighbors, min_node_ind);
end
%problem.plot();

src1  = [0; 
         0; 
         0];
dest1 = [5; 
         5; 
         5];

src3 = [0 0 0 0;
        0 0 0 0;
        0 0 0 0] ;
dest3 = [1 2 5 3;
         1 2 5 3;
         1 2 5 3];
ans1 = cost_function(src1, dest1);
ans2 = cost_function(src3, dest3);

position = steer([1;1;1], [10;5;3]);


% Weight coefficient: cost = w_1*x + w_2*y + w_3*z
function dist = cost_function(src_pos, dest_pos)
    weight = [1;1;1];
    dist = vecnorm([src_pos - dest_pos].*weight);
end

function candidates = feasbile_step(parent_node, sample_rate)
    candidates = zeros(3, sample_rate+1);
    candidates(:, 1) = parent_node + [-1;0;0];
    for i=1:sample_rate
        x = 90/(sample_rate-1);
        candidates(:,i+1) = parent_node + [0 ; 1*sind(x*(i-1)) ; -1*cosd(x*(i-1))];
    end

end    

function position = steer(nearest_node, new_node_position)
    candidates = feasbile_step(nearest_node, 5);
    repmat(new_node_position, 1, 6);
    costs = cost_function(repmat(new_node_position, 1, 6), candidates);
    [m,i] = min(costs);
    position = candidates(:,i);
end
        
        

