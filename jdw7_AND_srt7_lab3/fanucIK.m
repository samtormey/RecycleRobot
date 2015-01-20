function [is_solution,joint_angles] = fanucIK(T,prev_joint_angles,fanuc)

L_1 = fanuc.parameters.L_1;
L_2 = fanuc.parameters.L_2;
L_3 = fanuc.parameters.L_3;
L_4 = fanuc.parameters.L_4;

a1 = fanuc.parameters.a1;
% a2 = fanuc.parameters.a2;
% a3 = fanuc.parameters.a3;
% a4 = fanuc.parameters.a4;

px = T(1,4);
py = T(2,4);

Rgoal = T(1:3,1:3);

joint_angles_mat = zeros(6,2);

%% Solve for theta 1
the1 = atan2(py,px);

%% Solve for two theta 3's

T01 = dhtf(0,0,0,the1);
T12 = dhtf(a1,L_1,0,0 + pi/2);

T02 = T01*T12;

p2vec = T02 \ T(:,4);
p2vec = p2vec(1:3);

L_a = sqrt(L_3^2 + L_4^2);

x2 = -p2vec(2);
y2 = p2vec(1);

if (sqrt(x2^2 + y2^2) > (L_2 + L_a))
    is_solution = 0;
    joint_angles = zeros(6,1);
    return
else
    is_solution = 1;
end

phi = atan(L_3/L_4);
the3a = -acos((x2^2 + y2^2 - L_2^2 - L_a^2)/(2*L_2*L_a)) - phi + pi/2;
the3b = -the3a - 2*phi - pi;

%% Solve for two theta 2's (one for each theta 3)

B = atan2(y2,x2);
psi = acos((x2^2 + y2^2 + L_2^2 - L_a^2)/(2*L_2*sqrt(x2^2 + y2^2)));

the2a = B + psi - pi/2;
the2b = B - psi - pi/2;

%% Solve for theta 4-6

T01 = dhtf(0,0,0,the1);

% a
T12a = dhtf(a1,L_1,0,the2a + pi/2);
T23a = dhtf(0,L_2,0,the3a);
T34a = dhtf(pi/2,180,1600,0);

T04a = T01 * T12a * T23a * T34a;
R04a = T04a(1:3,1:3);

R = R04a\Rgoal; % R36 from frame 6 to frame 3

the5a = atan2(sqrt(R(3,1)^2 + R(3,2)^2),R(3,3));

if abs(sin(the5a)) > 10*eps
    the4a = atan2(R(2,3)/sin(the5a),R(1,3)/sin(the5a));
    the6a = atan2(R(3,2)/sin(the5a),-R(3,1)/sin(the5a));
else
    if abs(the5a) < 10*eps
        the4a = 0;
        the5a = 0;
        the6a = atan2(-R(1,2),R(1,1));
    else
        the4a = 0;
        the5a = pi;
        the6a = atan2(R(1,2),-R(1,1));
    end
    
end


% b

T12b = dhtf(a1,L_1,0,the2b + pi/2);
T23b = dhtf(0,L_2,0,the3b);
T34b = dhtf(pi/2,180,1600,0);

T04b = T01 * T12b * T23b * T34b;

R04b = T04b(1:3,1:3);

R = R04b\Rgoal; % R36 from frame 6 to frame 3

the5b = atan2(sqrt(R(3,1)^2 + R(3,2)^2),R(3,3));

if abs(sin(the5b)) > 10*eps 
    the4b = atan2(R(2,3)/sin(the5b),R(1,3)/sin(the5b));
    the6b = atan2(R(3,2)/sin(the5b),-R(3,1)/sin(the5b));
else
    if abs(the5b) < 10*eps
        the4b = 0;
        the5b = 0;
        the6b = atan2(-R(1,2),R(1,1));
    else
        the4b = 0;
        the5b = pi;
        the6b = atan2(R(1,2),-R(1,1));
    end
end



joint_angles_mat(1,:) = the1;

joint_angles_mat(2,1) = the2a;
joint_angles_mat(2,2) = the2b;
joint_angles_mat(3,1) = the3a;
joint_angles_mat(3,2) = the3b;

joint_angles_mat(4,1) = the4a;
joint_angles_mat(4,2) = the4b;
joint_angles_mat(5,1) = the5a;
joint_angles_mat(5,2) = the5b;
joint_angles_mat(6,1) = the6a;
joint_angles_mat(6,2) = the6b;

allsol = joint_angles_mat;
cnt = 1;

d = [];

for sol = 1:2
    issol = true;
    for ang = 1:6
        if (allsol(ang,sol) < fanuc.joint_limits{ang}(1)) || (allsol(ang,sol) > fanuc.joint_limits{ang}(2))
            issol = false;
            break;
        end
    end
    if issol == 1
        validsol(:,cnt) = allsol(:,sol); %#ok<AGROW>
        cnt = cnt + 1;
    end
end




if cnt > 1
    
    is_solution = 1;
    
    for i = 1:cnt-1
        d(i) = norm(prev_joint_angles - validsol(:,i)); %#ok<AGROW>
    end
    
    m = min(d)==d;
    
    joint_angles = validsol(:,m);
    joint_angles = joint_angles(:,1);
    
else
    
    is_solution = 0;
    joint_angles = zeros(6,1);
end
% d1 = norm(prev_joint_angles - joint_angles_mat(:,1));
% d2 = norm(prev_joint_angles - joint_angles_mat(:,2));
% 
% if (d1 < d2)
%     joint_angles = joint_angles_mat(:,1);
% else
%     joint_angles = joint_angles_mat(:,2);
% end









