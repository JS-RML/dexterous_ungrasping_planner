function collision = is_thumb_collision(theta, psi, d_AB, d_FT)
thumb_tip_thickness = 0.05;%Coin 0.01;%0.056;%poker0.01;%book0.05;%card0.06;
thumb_length = 0.75;%1.00;%coin 0.75;%poker1.3; %book0.75; %card2.3;
thumb_base_thickness = 0.2;%Coin 0.7;%0.18;%poker0.6;%book0.2; %card0.6;
contactA_height = (1-d_AB)*sind(theta); 
gripper_aperature = d_AB*sind(psi);

if (theta+psi) < 90
    height_diff_between_contactA_and_thumbtip = (gripper_aperature * sind(90-psi-theta)) - (d_FT * sind(theta+psi)) + (thumb_tip_thickness * cosd(theta+psi));
    height_diff_between_contactA_and_thumbbase = (gripper_aperature * sind(90-psi-theta)) - (thumb_length * sind(theta+psi)) + (thumb_base_thickness * cosd(theta+psi));
    if contactA_height < height_diff_between_contactA_and_thumbtip || contactA_height < height_diff_between_contactA_and_thumbbase
        collision = true;
    else
        collision = false;
    end
else
    collision = false;
end    
end

