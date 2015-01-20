function [is_solution,joint_angles] = pumaIK(T,prev_joint_angles,puma)

px = T(1,4);
py = T(2,4);
pz = T(3,4);
d3 = puma.parameters.L_3;
a2 = puma.parameters.L_2;
a3 = 0;
a4 = puma.parameters.L_4;  %d4

the = zeros(6,4);



p1a =  atan2(py,px) - atan2(d3,sqrt(px^2+py^2-d3^2));
p1b = atan2(py,px) - atan2(d3,-sqrt((px^2+py^2-d3^2)));

the(1,1:2) = p1a;
the(1,3:4) = p1b;


K = (px^2 + py^2 + pz^2 - a2^2 - a3^2 - d3^2 - a4^2)/(2*a2);

p2a = atan2(-pz,sqrt(px^2+py^2-d3^2)) - atan2(sqrt(a3^2+a4^2-K^2),K+a2);
p2b = atan2(-pz,sqrt(px^2+py^2-d3^2)) - atan2(-sqrt(a3^2+a4^2-K^2),K+a2);
p2c = atan2(-pz,-sqrt(px^2+py^2-d3^2)) - atan2(sqrt(a3^2+a4^2-K^2),K+a2);
p2d = atan2(-pz,-sqrt(px^2+py^2-d3^2)) - atan2(-sqrt(a3^2+a4^2-K^2),K+a2);

the(2,1) = p2a;
the(2,2) = p2b;
the(2,3) = p2c;
the(2,4) = p2d;

p3a = atan2(a3,a4) - atan2(K,sqrt(a3^2+a4^2-K^2));
p3b = atan2(a3,a4) - atan2(K,-sqrt(a3^2+a4^2-K^2));

the(3,[1,3]) = p3a;
the(3,[2,4]) = p3b;

s5 = zeros(1,4);
s6 = s5;
c5 = s5;
c6 = s5;

for i = 1:4
    
    the(4,i) = atan2(-T(1,3)*sin(the(1,i)) + T(2,3)*cos(the(1,i)), -T(1,3)*cos(the(1,i))*cos(the(2,i)+the(3,i)) - T(2,3)*sin(the(1,i))* ...
        cos(the(2,i)+the(3,i)) + T(3,3)*sin(the(2,i)+the(3,i)));
    
    
    s5(i) = -1*(T(1,3)*(cos(the(1,i))*cos(the(2,i)+the(3,i))*cos(the(4,i)) + sin(the(1,i))*sin(the(4,i))) + T(2,3)*(sin(the(1,i))* ...
        cos(the(2,i)+the(3,i))*cos(the(4,i)) - cos(the(1,i))*sin(the(4,i))) - T(3,3)*sin(the(2,i)+the(3,i))*cos(the(4,i)));
    
    c5(i) = T(1,3)*-1*cos(the(1,i))*sin(the(2,i)+the(3,i)) + T(2,3)*-1*sin(the(1,i))*sin(the(2,i)+the(3,i)) + ...
        T(3,3)*-1*cos(the(2,i)+the(3,i));
    
    if abs(s5(i)) > 10*eps
        
        the(5,i) = atan2(s5(i),c5(i));
        
    else
        
        the(4,i) = prev_joint_angles(4);
        s5(i) = -1*(T(1,3)*(cos(the(1,i))*cos(the(2,i)+the(3,i))*cos(the(4,i)) + sin(the(1,i))*sin(the(4,i))) + T(2,3)*(sin(the(1,i))* ...
            cos(the(2,i)+the(3,i))*cos(the(4,i)) - cos(the(1,i))*sin(the(4,i))) - T(3,3)*sin(the(2,i)+the(3,i))*cos(the(4,i)));
        c5(i) = T(1,3)*-1*cos(the(1,i))*sin(the(2,i)+the(3,i)) + T(2,3)*-1*sin(the(1,i))*sin(the(2,i)+the(3,i)) + ...
            T(3,3)*-1*cos(the(2,i)+the(3,i));
        
        the(5,i) = atan2(s5(i),c5(i));
        
    end
    
    s6(i) = -T(1,1)*(cos(the(1,i))*cos(the(2,i)+the(3,i))*sin(the(4,i)) - sin(the(1,i))*cos(the(4,i))) - T(2,1)*(sin(the(1,i))* ...
        cos(the(2,i)+the(3,i))*sin(the(4,i)) + cos(the(1,i))*cos(the(4,i))) + T(3,1)*sin(the(2,i)+the(3,i))*sin(the(4,i));
    
    
    c6(i) = T(1,1)*((cos(the(1,i))*cos(the(2,i)+the(3,i))*cos(the(4,i)) + sin(the(1,i))*sin(the(4,i)))*cos(the(5,i)) - cos(the(1,i))*sin(the(2,i)+the(3,i))*sin(the(5,i))) + ...
        T(2,1)*((sin(the(1,i))*cos(the(2,i)+the(3,i))*cos(the(4,i)) - cos(the(1,i))*sin(the(4,i)))*cos(the(5,i)) - sin(the(1,i))*sin(the(2,i)+the(3,i))*sin(the(5,i)))...
        - T(3,1)*(sin(the(2,i)+the(3,i))*cos(the(4,i))*cos(the(5,i)) + cos(the(2,i)+the(3,i))*sin(the(5,i)));
    
    the(6,i) = atan2(s6(i),c6(i));
    
end

theflipped = the;
theflipped(4,:) = theflipped(4,:) + pi;
theflipped(5,:) = -theflipped(5,:);
theflipped(6,:) = theflipped(6,:) + pi;


allsol = [the, theflipped];
cnt = 1;

d = [];

for sol = 1:8
    issol = true;
    for ang = 1:6
        if (allsol(ang,sol) < puma.joint_limits{ang}(1)) || (allsol(ang,sol) > puma.joint_limits{ang}(2))
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
        d(i) = norm(prev_joint_angles - validsol(:,i));
    end
    
    m = min(d)==d;
    
    joint_angles = validsol(:,m);
    joint_angles = joint_angles(:,1);
    
else
    
    is_solution = 0;
    joint_angles = zeros(6,1);
end
