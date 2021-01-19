clc;
clear;
x = [];
y = [];
z = [];
x_obs = [];
y_obs = [];
z_obs = [];

d_FT = 0.47;
for i = 1:1:10
    d_AB = i/10;
    for theta = 1:1:90
        for psi =1:1:90
            fc = is_forceclosure(theta, psi, d_AB, 0, 0);
            collision = is_collision(theta, psi, d_AB, d_FT);
            answer(psi, theta) = fc;
            obs(psi,theta) = collision;
            if collision==1
                answer(psi, theta) = 0;
            end
        end         
    end
    [rowInd,colInd]=ind2sub(size(answer),find(answer));
    x = [x; rowInd];
    y = [y; colInd];
    z = [z; (i/10)*ones(length(rowInd),1)];
    [obs_rowInd,obs_colInd] = ind2sub(size(obs),find(obs));
    x_obs = [x_obs; obs_rowInd];
    y_obs = [y_obs; obs_colInd];
    z_obs = [z_obs; (i/10)*ones(length(obs_rowInd),1)];
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
P = [x,y,z];
k = boundary(P,1);
trisurf(k,P(:,2),P(:,1),P(:,3), 'FaceColor', [0.5, 0.5, 0.5], 'FaceAlpha',0.2, 'EdgeColor', 'none', 'LineWidth', 0.1)
            
OBS = [x_obs,y_obs,z_obs.*100];
OBS_bound = boundary(OBS,1);
trisurf(OBS_bound,OBS(:,2),OBS(:,1),OBS(:,3)./100, 'FaceColor', 'r', 'FaceAlpha',0.2, 'EdgeColor', 'none', 'LineWidth', 0.1)     
