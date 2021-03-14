clc;
clear;
x = [];
y = [];
z = [];

init = [30 0 0.8];
goal = [0 44 0.6];
path = [
   30.0000         0    0.8000
   30.0000    4.5500    0.7800
   30.0000    9.1000    0.7600
   30.0000   13.6600    0.7400
   30.0000   18.2100    0.7200
   30.0000   22.7600    0.7000
   30.0000   27.3100    0.6800
   30.0000   31.8600    0.6600
   30.0000   36.4100    0.6300
   30.0000   40.9700    0.6100
   30.0000   44.0000    0.6000
   25.0000   44.0000    0.6000
   20.0000   44.0000    0.6000
   15.0000   44.0000    0.6000
   10.0000   44.0000    0.6000
    5.0000   44.0000    0.6000
         0   44.0000    0.6000];

plot3(init(1), init(2), init(3), '-o','Color','b','MarkerSize',10,'MarkerFaceColor','b')
plot3(goal(1), goal(2), goal(3),'-o','Color','g','MarkerSize',10,'MarkerFaceColor','g')
plot3(path(:,1), path(:,2), path(:,3), '-.k','LineWidth', 2.5);
           

d_FT = 0.4;
for i = 6
    d_AB = i/10;
    for theta = 1:1:90
        for psi =1:1:90
            fc = is_forceclosure(theta, psi, d_AB, 0, 0);
            collision = is_thumb_collision(theta, psi, d_AB, d_FT);
            answer(psi, theta) = fc;
            if collision==1
                answer(psi, theta) = 0;
                cobs(psi,theta) = 1;
            end
        end         
    end
    [rowInd,colInd]=ind2sub(size(answer),find(answer));
    [cobs_rowInd,cobs_colInd]=ind2sub(size(cobs),find(cobs));
    x = [x; rowInd];
    y = [y; colInd];
    z = [z; (i/10)*ones(length(rowInd),1)];
    if i < 2
        k = boundary(rowInd, colInd, 0.8);
        cobs_k = boundary(cobs_rowInd, cobs_colInd, 0.8);
    elseif i < 6
        k = boundary(rowInd, colInd, 0.7);
        cobs_k = boundary(cobs_rowInd, cobs_colInd, 0.7);
    elseif i == 6
        k = boundary(rowInd, colInd, 0.5);
        cobs_k = boundary(cobs_rowInd, cobs_colInd, 0.5);
    elseif i > 6
        k = boundary(rowInd, colInd, 0.8);
        cobs_k = boundary(cobs_rowInd, cobs_colInd, 0.8);
    end
    hold on;
    plot3(colInd(k),rowInd(k), (i/10)*ones(length(k), 1), 'Color', '#EDB120', 'LineWidth', 1.5);
    lh = plot3(cobs_colInd(cobs_k),cobs_rowInd(cobs_k), (i/10)*ones(length(cobs_k), 1), 'Color', 'r', 'LineWidth', 1.5);
    lh.Color=[1,0,0,0.2];
    axis([0 90 0 90 0.1 0.9])

end
P = [x,y,z];
k = boundary(P,1);
trisurf(k,P(:,2),P(:,1),P(:,3), 'FaceColor', [0.5, 0.5, 0.5], 'FaceAlpha',0.2, 'EdgeColor', 'none', 'LineWidth', 0.1)
view(0,90)       
