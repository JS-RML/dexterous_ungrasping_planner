function collision = is_finger_collision(theta, psi, d_AB, corner_x, corner_y)
    %the finger collision is modeled with a corner as an obstacle above the object and finger
    %corner_x: x_axis position of corner relative to G
    %corner_y: y_axis position of corner relative to G

    %finger dimensions
    finger_tip_thickness = 0.05;
    finger_length = 0.75;
    finger_base_thickness = 0.2;
    finger_trapezoid_angle = atand((finger_base_thickness-finger_tip_thickness)/finger_length);

    %contact A position relative to contact G
    A_x = (1-d_AB)*cosd(theta); 
    A_y = (1-d_AB)*sind(theta);

    %finger tip position relative to contact G
    tip_x = A_x - finger_tip_thickness*sind(theta+psi);
    tip_y = A_y + finger_tip_thickness*cosd(theta+psi);

    %use two inequalities to check if the corner is colliding with the object and finger
    %linear inequality for object (line intersect G and fingertip)
    obj_coll = true;
    obj_m = tip_y/tip_x;
    obj_c = 0;
    %linear inequality for finger (line intersect fingertip and finger base)
    finger_coll = true;
    finger_m = tand(finger_trapezoid_angle + theta + psi);
    finger_c = tip_y - finger_m*tip_x; %c = y - mx
    
    if obj_m > 0 && atand(obj_m) < 90
        if corner_y > (obj_m * corner_x + obj_c)
            obj_coll = false;
        end
    end

    if finger_trapezoid_angle + theta + psi < 90
        if corner_y > (finger_m * corner_x + finger_c)
            finger_coll = false;
        end
    end

    if ~obj_coll && ~finger_coll
        collision = false;
    else 
        collision = true;
    end

end
    