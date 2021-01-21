classdef DUSimple3D < handle
    properties (SetAccess = private)
        tree                % Array stores position information of states
        orientation         % Array stores orientation of a state of the robot
        parent              % Array stores relations of nodes
        children            % Number of children of each node
        free_nodes          % Indices of free nodes
        free_nodes_ind      % Last element in free_nodes
        cost                % Cost between 2 connected states
        cumcost             % Cost from the root of the tree to the given node
        XYZ_BOUNDARY        % [min_x max_x min_y max_y]
        start_point         % Start position
        goal_point          % Goal position
        delta_goal_point    % Radius of goal position region
        delta_near          % Radius of near neighbor nodes
        nodes_added         % Keeps count of added nodes
        max_step            % The length of the maximum step while adding the node
        obstacle            % Obstacle information
        best_path_node      % The index of last node of the best path
        goal_reached
        %%% temporary variables
        compare_table
        index
        list
        num_rewired
        mode_change_weight
    end
    methods
        % class constructor
        function this = DUSimple3D(rand_seed, max_nodes, map, conf)
            rng(rand_seed);
            this.tree = zeros(3, max_nodes);
            this.parent = zeros(1, max_nodes);
            this.children = zeros(1, max_nodes+1);
            this.free_nodes = zeros(1, max_nodes);
            this.free_nodes_ind = 1;
            this.cost = zeros(1, max_nodes);
            this.cumcost = zeros(1,max_nodes);
            this.XYZ_BOUNDARY = zeros(6,1);
            this.tree(:, 1) = map.start_point; % Start position
            this.start_point = map.start_point;
            this.goal_point = map.goal_point;
            this.delta_goal_point = conf.delta_goal_point;
            this.delta_near = conf.delta_near;
            this.nodes_added = uint32(1);
            this.max_step = conf.max_step;
            this.best_path_node = -1;
            this.goal_reached = false;
            this.load_map(map.name);
            %%% temp var-s initialization
            this.compare_table = zeros(1, max_nodes);
            this.index = zeros(1, max_nodes);
            this.list = 1:max_nodes;
            this.num_rewired = 0;
            this.mode_change_weight = conf.mode_change_weight;
        end
   
        function position = sample(this)
            % generates and return random point in area defined in
            position = [this.XYZ_BOUNDARY(2) - this.XYZ_BOUNDARY(1); this.XYZ_BOUNDARY(4) - this.XYZ_BOUNDARY(3); this.goal_point(3) - this.start_point(3)] .* rand(3,1) ...
                + [this.XYZ_BOUNDARY(1);this.XYZ_BOUNDARY(3); this.start_point(3)];
        end
        
        function node_index = nearest(this, new_node)
            % find the nearest node to the given node
            %this.compare_table(1:(this.nodes_added)) = sum((this.tree(:, 1:(this.nodes_added)) - repmat(new_node(1:3),1,this.nodes_added)).^2); %Eucledian distance
            this.compare_table(1:(this.nodes_added)) = this.cost_function(this.tree(:, 1:(this.nodes_added)), repmat(new_node(1:3),1,this.nodes_added)); % Cost Function
            [this.compare_table(1:(this.nodes_added)), this.index(1:(this.nodes_added))] = sort(this.compare_table(1:(this.nodes_added)));
            node_index = this.index(1);
            return;
        end
        
        function position = steer(this, nearest_node, new_node_position)
            sample_rate = 5;
            if nearest_node == 1
                parent_of_nearest_node = 1;
            else
                parent_of_nearest_node = this.parent(nearest_node);
            end
            mode_of_prev = this.contact_mode(this.tree(:, nearest_node)-this.tree(:, parent_of_nearest_node));
            candidates = this.feasbile_action(this.max_step, this.tree(:, nearest_node), sample_rate);
            mode_of_candidates = this.contact_mode(candidates-repmat(this.tree(:, nearest_node),1,size(candidates,2)));
            mode_change_cost =  abs(mode_of_candidates-repmat(mode_of_prev,1,size(mode_of_candidates,2)));
            costs = this.cost_function(repmat(new_node_position, 1, sample_rate+1), candidates) + this.mode_change_weight*mode_change_cost;
            [m,i] = min(costs);
            position = candidates(:,i);
            %{
            relative_position = new_node_position - this.tree(:, nearest_node); 
            dist = this.cost_function(relative_position, [0; 0; 0]); 
            if dist > this.max_step % if dist > max_step, steer
                scale = this.max_step/dist; % 
                position_step = relative_position.*scale; 
                position = this.tree(:, nearest_node) + position_step; 
            else 
                position = new_node_position; % else sampled = new_node
            end
            %}
        end
        
        function load_map(this, map_name)
            % function loads '.mat' file with obstacle information and the
            % size of the map
            map_path = 'maps/';
            this.obstacle = load([map_path map_name], 'num', 'output', 'x_constraints', 'y_constraints', 'z_constraints');
            this.XYZ_BOUNDARY = [this.obstacle.x_constraints this.obstacle.y_constraints this.obstacle.z_constraints];
        end
        
        function collision = obstacle_collision(this, new_node_position, nearest_node_ind)
            sample_rate = 2;
            nearest_node_position = this.tree(:,nearest_node_ind);
            increment = (new_node_position-nearest_node_position)/sample_rate;
            mode = this.contact_mode(new_node_position-nearest_node_position);
            if mode==1 
                A_slide = false;
                B_slide = false;
            elseif mode==2
                A_slide = false;
                B_slide = true;
            elseif mode==3
                A_slide = true;
                B_slide = true;
            end
            collision = false;
            for i=0:sample_rate
                config = nearest_node_position+increment*i;
                fc = is_forceclosure(config(1), config(2), config(3), A_slide, B_slide);
                c  = is_collision(config(1), config(2), config(3), 0.47);
                if fc==false || c==true
                    collision = true; 
                    break
                end
            end
            
        end
        
        function new_node_ind = insert_node(this, parent_node_ind, new_node_position)
            % method insert new node in the tree
            this.nodes_added = this.nodes_added + 1;
            this.tree(:, this.nodes_added) = new_node_position(1:3);         % adding new node position to the tree
            this.parent(this.nodes_added) = parent_node_ind;            % adding information about parent-children information
            this.children(parent_node_ind) = this.children(parent_node_ind) + 1;
            this.cost(this.nodes_added) = this.cost_function(this.tree(:, parent_node_ind), new_node_position);  % not that important
            this.cumcost(this.nodes_added) = this.cumcost(parent_node_ind) + this.cost(this.nodes_added);   % cummulative cost
            new_node_ind = this.nodes_added;
        end
        
        %%% RRT* specific functions
        
        function neighbor_nodes = neighbors(this, new_node_position, nearest_node_ind)
            % seeks for neighbors and returns indices of neighboring nodes
            radius = this.delta_near;
            %this.compare_table(1:(this.nodes_added)) = sum((this.tree(:, 1:(this.nodes_added)) - repmat(new_node_position(1:3),1,this.nodes_added)).^2); % Eucledian distance
            this.compare_table(1:(this.nodes_added)) = this.cost_function(this.tree(:, 1:(this.nodes_added)), repmat(new_node_position(1:3),1,this.nodes_added)); 
            [this.compare_table(1:(this.nodes_added)), this.index(1:(this.nodes_added))] = sort(this.compare_table(1:(this.nodes_added)));
            temp = this.index((this.compare_table(1:(this.nodes_added)) <= radius) & (this.compare_table(1:(this.nodes_added)) > 0 ));
            neighbor_nodes = temp;
        end
        
        function min_node_ind = chooseParent(this, neighbors, nearest_node, new_node_position)
            % finds the node with minimal cummulative cost node from the root of
            % the tree. i.e. find the cheapest path end node.
            min_node_ind = nearest_node;
            min_cumcost = this.cumcost(nearest_node) + this.cost_function(this.tree(:, nearest_node), new_node_position);
            for ind=1:numel(neighbors)
                temp_cumcost = this.cumcost(neighbors(ind)) + this.cost_function(this.tree(:, neighbors(ind)), new_node_position);
                if temp_cumcost < min_cumcost
                    min_cumcost = temp_cumcost;
                    min_node_ind = neighbors(ind);
                end
            end
        end
        
        function rewire(this, new_node_ind, neighbors, min_node_ind)
            % method looks thru all neighbors(except min_node_ind) and
            % seeks and reconnects neighbors to the new node if it is
            % cheaper
            for ind = 1:numel(neighbors)
                % omit
                if (min_node_ind == neighbors(ind))
                    continue;
                end
                temp_cost = this.cumcost(new_node_ind) + this.cost_function(this.tree(:, neighbors(ind)), this.tree(:, new_node_ind));
                if (temp_cost < this.cumcost(neighbors(ind)))
                    this.cumcost(neighbors(ind)) = temp_cost;
                    this.children(this.parent(neighbors(ind))) = this.children(this.parent(neighbors(ind))) - 1;
                    this.parent(neighbors(ind)) = new_node_ind;
                    this.children(new_node_ind) = this.children(new_node_ind) + 1;
                    this.num_rewired = this.num_rewired + 1;
                end
            end
        end
        
   
        function plot(this)
            %%% Find the optimal path to the goal
            % finding all the point which are in the desired region
            distances = zeros(this.nodes_added, 2);
            distances(:, 1) = sum((this.tree(:,1:(this.nodes_added)) - repmat(this.goal_point', 1, this.nodes_added)).^2);
            distances(:, 2) = 1:this.nodes_added;
            distances = sortrows(distances, 1);
            distances(:, 1) = distances(:, 1) <= this.delta_goal_point ^ 2;
            dist_index = numel(find(distances(:, 1) == 1));
            % find the cheapest path
            if(dist_index ~= 0)
                distances(:, 1) = this.cumcost(int32(distances(:, 2)));
                distances = distances(1:dist_index, :);
                distances = sortrows(distances, 1);
                nearest_node_index = distances(1,2);
            else
                disp('NOTICE! Robot cannot reach the goal');
                nearest_node_index = distances(1,2);
            end
            % backtracing the path
            current_index = nearest_node_index;
            path_iter = 1;
            backtrace_path = zeros(1,1);
                
            while(current_index ~= 1)
                backtrace_path(path_iter) = current_index;
                path_iter = path_iter + 1;
                current_index = this.parent(current_index);
            end
            backtrace_path(path_iter) = current_index;
            close all;
            figure;
            set(gcf(), 'Renderer', 'opengl');
            hold on;
            
            drawn_nodes = zeros(1, this.nodes_added);
            for ind = this.nodes_added:-1:1;
                if(sum(this.free_nodes(1:this.free_nodes_ind) == ind)>0)
                    continue;
                end
                current_index = ind;
                while(current_index ~= 1 && current_index ~= -1)
                    % avoid drawing same nodes twice or more times
                    if(drawn_nodes(current_index) == false || drawn_nodes(this.parent(current_index)) == false)
                        plot3([this.tree(1,current_index);this.tree(1, this.parent(current_index))], ...
                            [this.tree(2, current_index);this.tree(2, this.parent(current_index))], ...
                            [this.tree(3, current_index);this.tree(3, this.parent(current_index))], 'g-','LineWidth', 0.5);
                        drawn_nodes(current_index) = true;
                        
                    end
                    current_index = this.parent(current_index);
                end
            end
            
            plot3(this.tree(1,backtrace_path), this.tree(2,backtrace_path), this.tree(3,backtrace_path), '*b-','LineWidth', 2);
            plot3(this.tree(1, 1), this.tree(2, 1), this.tree(3, 1), '-o','Color','r','MarkerSize',10,'MarkerFaceColor','r')
            plot3(this.goal_point(1), this.goal_point(2), this.goal_point(3),'-o','Color','m','MarkerSize',10,'MarkerFaceColor','m')
            
            %Save output path as csv file
            output_path = 0;
            if output_path == 1
                output_path = zeros(numel(backtrace_path),3);
                for ind = numel(backtrace_path):-1:1;
                    output_path(numel(backtrace_path)-ind+1,:) = this.tree(:,backtrace_path(ind)).';
                end
                file_name = ['output_path/' datestr(now, 'yyyy-mm-dd HH:MM:SS') '.csv'];
                writematrix(output_path, file_name);
            end

            %START: plot grey region 
            load("regions/grey_region(dft0.47).mat", "P")
            P(:,3) = P(:,3);
            set(findall(gca, 'Type', 'Line'),'LineWidth',1);
            grid on
            k = boundary(P,1);
            trisurf(k,P(:,2),P(:,1),P(:,3)./100, 'FaceColor', [0.5, 0.5, 0.5], 'FaceAlpha',0.2, 'EdgeColor', 'none', 'LineWidth', 0.1)
            P(:,3) = P(:,3)./100;
            for i=0.1:0.1:0.9
                this.plot_boundary(P, i)
            end
            %END: plot grey region
            
            %plot obs region
            load("regions/red_region(dft0.47).mat", "OBS")
            OBS(:,3) = OBS(:,3);
            set(findall(gca, 'Type', 'Line'),'LineWidth',1);
            grid on
            OBS_bound = boundary(OBS,1);
            trisurf(OBS_bound,OBS(:,2),OBS(:,1),OBS(:,3)./100, 'FaceColor', 'r', 'FaceAlpha',0.2, 'EdgeColor', 'none', 'LineWidth', 0.1)
            
            axis(this.XYZ_BOUNDARY);
            grid on;
            axis square;
            xlabel('Theta'); 
            ylabel('Psi');
            zlabel('Gamma');
            disp(num2str(this.cumcost(backtrace_path(1))));
        end
        
        function newObj = copyobj(thisObj)
            % Construct a new object based on a deep copy of the current
            % object of this class by copying properties over.
            props = properties(thisObj);
            for i = 1:length(props)
                % Use Dynamic Expressions to copy the required property.
                % For more info on usage of Dynamic Expressions, refer to
                % the section "Creating Field Names Dynamically" in:
                % web([docroot '/techdoc/matlab_prog/br04bw6-38.html#br1v5a9-1'])
                newObj.(props{i}) = thisObj.(props{i});
            end
        end
            
    end
    methods(Static)
        % COST FUNCTION REQUIREMENT
        % Input: 3xn (row x col)
        % Output: 1xn (row x col)
        
        % Euclidian distance
        function dist = euclidian_distance(src_pos, dest_pos)
            dist = vecnorm(src_pos - dest_pos);
        end

        % Weighted Sum Cost Function: cost = w_1*x + w_2*y + w_3*z
        function dist = cost_function(src_pos, dest_pos)
            w = 1;
            weight = [w; w; 100];
            weight_rep = repmat(weight, 1, size(src_pos,2));
            difference = dest_pos - src_pos;
            temp = abs(difference).*weight_rep;
            dist = sum(temp, 1);
        end
        
        function candidates = feasbile_action(max_step, parent_node_pos, sample_rate)
            candidates = zeros(3, sample_rate+1);
            candidates(:, 1) = parent_node_pos + [-max_step;0;0];
            for i=1:sample_rate
                x = 90/(sample_rate-1);
                candidates(:,i+1) = parent_node_pos + [0 ; max_step*sind(x*(i-1)) ; -(max_step/100)*cosd(x*(i-1))];
            end

        end     
        
        function mode = contact_mode(diff)
            mode = 2*ones(1,size(diff,2));
            for i=1:size(diff,2)
                if diff(1,i)<0 && diff(2,i)==0 && diff(3,i)==0
                    mode(i) = 1;
                elseif diff(1,i)==0 && diff(2,i)>0 && diff(3,i)==0
                    mode(i) = 2;
                elseif diff(1,i)==0 && diff(2,i)>0 && diff(3,i)<0
                    mode(i) = 3;
                elseif diff(1,i)==0 && diff(2,i)==0 && diff(3,i)<0
                    mode(i) = 3;
                end
            end
            
        end
        
        function plot_boundary(P, i)
            temp=[];
            for j=1:size(P,1)
                if round(P(j,3),1)==round(i,1)
                   temp = [temp;P(j,:)];
                end
            end
            hold on;
            k = boundary(temp(:,1), temp(:,2), 1);
            plot3(temp(k,2),temp(k,1), (i)*ones(length(k), 1));
        end
        
        
    end
end
