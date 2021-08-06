function fc = is_forceclosure(theta, psi, d_AB, A_slide, B_slide)

%Parameters
phi_G = 11; %9 savemap %11deg = 0.2rad; cardboard on steel
phi_A = 17; %28deg = 0.5rad; cardboard to rubber
phi_B = 17; %38deg = 0.8coef

%Unit Contact Forces
f_G1 = 1;
f_G2 = 1;
f_A0 = 1;
f_A1 = 1;
f_A2 = 1;
f_B0 = 1;
f_B1 = 1;
f_B2 = 1;

%Wrenches
F_G1 = f_G1 * [0, -sind(phi_G), cosd(phi_G)]';
F_G2 = f_G2 * [0, sind(phi_G), cosd(phi_G)]'; 
F_A0 = f_A0 * [-(1-d_AB)*sind(90), sind(theta), -cosd(theta)]';
F_A1 = f_A1 * [-(1-d_AB)*sind(90-phi_A), sind(theta-phi_A), -cosd(theta-phi_A)]';
F_A2 = f_A2 * [-(1-d_AB)*sind(90+phi_A), sind(theta+phi_A), -cosd(theta+phi_A)]';
F_B0 = f_B0 * [cosd(psi), -sind(psi+theta), cosd(psi+theta)]'; % Get rid for sliding at B
F_B1 = f_B1 * [cosd(psi+phi_B), -sind(psi+theta+phi_B), cosd(psi+theta+phi_B)]'; % Get rid for sliding at B
F_B2 = f_B2 * [cosd(psi-phi_B), -sind(psi+theta-phi_B), cosd(psi+theta-phi_B)]';

if A_slide==false && B_slide==false
    F = [F_G1, F_G2, F_A1, F_A2, F_B1, F_B2];
elseif A_slide==false && B_slide==true
    F = [F_G1, F_G2, F_A1, F_A2, F_B0, F_B2];
elseif A_slide==true && B_slide==true
    F = [F_G1, F_G2, F_A0, F_A1, F_B0, F_B2];
end

%F = [F_G1, F_G2, F_A1, F_A2, F_B2, F_B0];

%Condition 1: Rank(F) = n where n=3 for planar
rank_is = rank(F);
if rank_is < 3;
    exitflag = 0;
    fc = false;
%Condition 2: There exists a solution to the linear programming problem
else
    f = [1, 1, 1, 1, 1, 1];
    A = [[-1,0,0,0,0,0];[0,-1,0,0,0,0];[0,0,-1,0,0,0];[0,0,0,-1,0,0];[0,0,0,0,-1,0];[0,0,0,0,0,-1]];
    b = [-1, -1, -1, -1, -1,-1];
    Aeq = F;
    beq = [0, 0, 0];
    options = optimoptions('linprog','Display','none');
    [k,fval,exitflag] = linprog(f, A, b, Aeq, beq,[],[], options);
    if exitflag == -2;
        exitflag = 0;
        fc = false;
    elseif exitflag == 1;
        fc = true;
    end
end