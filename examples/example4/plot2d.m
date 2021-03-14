clc;
clear;
x = [];
y = [];
z = [];

init = [20 0 0.5];
goal = [0 21 0.3];
path = [
  20.0000         0    0.5000
  20.0000    2.9000    0.4700
  20.0000    5.7900    0.4400
  20.0000    8.6900    0.4200
  20.0000   11.5900    0.3900
  20.0000   14.4800    0.3600
  20.0000   17.3800    0.3300
  20.0000   20.2800    0.3100
  20.0000   21.0000    0.3100
  16.0000   21.0000    0.3000
  12.0000   21.0000    0.3000
  8.0000   21.0000    0.3000
  0.0000   21.0000    0.3000];

plot3(init(1), init(2), init(3), '-o','Color','b','MarkerSize',10,'MarkerFaceColor','b')
plot3(goal(1), goal(2), goal(3),'-o','Color','g','MarkerSize',10,'MarkerFaceColor','g')
plot3(path(:,1), path(:,2), path(:,3), '-.k','LineWidth', 2.5);
           

d_FT = 0.6;
for i = 5
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
    plot3(colInd(k),rowInd(k), (i/10)*ones(length(k), 1), 'Color', '#77AC30', 'LineWidth', 1.5);
    lh = plot3(cobs_colInd(cobs_k),cobs_rowInd(cobs_k), (i/10)*ones(length(cobs_k), 1), 'Color', 'r', 'LineWidth', 1.5);
    lh.Color=[1,0,0,0.2];
    axis([0 90 0 90 0.1 0.9])

end
P = [x,y,z];
k = boundary(P,1);
trisurf(k,P(:,2),P(:,1),P(:,3), 'FaceColor', [0.5, 0.5, 0.5], 'FaceAlpha',0.2, 'EdgeColor', 'none', 'LineWidth', 0.1)
view(0,90)       
