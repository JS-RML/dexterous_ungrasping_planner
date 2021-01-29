clc;
clear;
x = [];
y = [];
z = [];
x_t_obs = [];
y_t_obs = [];
z_t_obs = [];
x_f_obs = [];
y_f_obs = [];
z_f_obs = [];
finger_collision_check = false;

d_FT = 0;
%corner position relative to G
corner_x = 0.3;
corner_y = 0.7;
for i = 1:1:9
    d_AB = i/10;
    for theta = 1:1:90
        for psi =1:1:90
            fc = is_forceclosure(theta, psi, d_AB, 0, 0);
            thumb_collision = is_thumb_collision(theta, psi, d_AB, d_FT);
            if finger_collision_check == true
                finger_collision = is_finger_collision(theta, psi, d_AB, corner_x, corner_y);
            else 
                finger_collision = false;
            end
            answer(psi, theta) = fc;
            t_obs(psi,theta) = thumb_collision;
            f_obs(psi,theta) = finger_collision;
            if thumb_collision == 1 || finger_collision == 1 
                answer(psi, theta) = 0;
            end
        end         
    end

    [rowInd,colInd]=ind2sub(size(answer),find(answer));
    x = [x; rowInd];
    y = [y; colInd];
    z = [z; (i/10)*ones(length(rowInd),1)];

    [t_obs_rowInd,t_obs_colInd] = ind2sub(size(t_obs),find(t_obs));
    x_t_obs = [x_t_obs; t_obs_rowInd];
    y_t_obs = [y_t_obs; t_obs_colInd];
    z_t_obs = [z_t_obs; (i/10)*ones(length(t_obs_rowInd),1)];

    if finger_collision_check == true
        [f_obs_rowInd,f_obs_colInd] = ind2sub(size(f_obs),find(f_obs));
        x_f_obs = [x_f_obs; f_obs_rowInd];
        y_f_obs = [y_f_obs; f_obs_colInd];
        z_f_obs = [z_f_obs; (i/10)*ones(length(f_obs_rowInd),1)];
    end

    if i < 2
        k = boundary(rowInd, colInd, 0.8);
    elseif i < 6
        k = boundary(rowInd, colInd, 0.7);
    elseif i == 6
        k = boundary(rowInd, colInd, 0.5);
    elseif i > 6
        k = boundary(rowInd, colInd, 0.8);
    end
    hold on;
    plot3(colInd(k),rowInd(k), (i/10)*ones(length(k), 1));
    axis([0 90 0 90 0.1 0.9])

end

P = [x,y,z.*100];
k = boundary(P,1);
trisurf(k,P(:,2),P(:,1),P(:,3)./100, 'FaceColor', [0.5, 0.5, 0.5], 'FaceAlpha',0.2, 'EdgeColor', 'none', 'LineWidth', 0.1)
            
T_OBS = [x_t_obs,y_t_obs,z_t_obs.*100];
T_OBS_bound = boundary(T_OBS,1);
trisurf(T_OBS_bound,T_OBS(:,2),T_OBS(:,1),T_OBS(:,3)./100, 'FaceColor', 'r', 'FaceAlpha',0.2, 'EdgeColor', 'none', 'LineWidth', 0.1)

if finger_collision_check == true
    F_OBS = [x_f_obs,y_f_obs,z_f_obs.*100];
    F_OBS_bound = boundary(F_OBS,1);
    trisurf(F_OBS_bound,F_OBS(:,2),F_OBS(:,1),F_OBS(:,3)./100, 'FaceColor', '#D95319', 'FaceAlpha',0.2, 'EdgeColor', 'none', 'LineWidth', 0.1)
end