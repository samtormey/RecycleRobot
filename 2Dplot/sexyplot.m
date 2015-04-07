function sexyplot

pit = load('Precompute/Controls_n=20_numThe=80_gps=5');
A = pit.A;
n = pit.n;

robot = ScaraInit;
len1 = robot.l_1;
len2 = robot.l_2;

belt = ConvBelt;
goal_y = belt.robo2goal;
belt_bottom = belt.robo2bottom;
belt_top = belt.robo2top;
% generate goal region points
gps = 5;
goal_width = 2*sqrt((len1+len2)^2 - goal_y^2);
goal_points_x = linspace(-goal_width/2,goal_width/2,gps);
goal_points_y = goal_y*ones(1,gps);
points = [goal_points_x; goal_points_y];

% Inverse Kinematics
[the1p, the2p, the1n, the2n] = inverseThe1_2(points,len1,len2);
goal_configs = [the1p the1n(2:end-1); the2p the2n(2:end-1)]'; % note this!

num_theta = 80;
dt = 2*pi/num_theta;
theta_vec = -pi+dt:dt:pi;

Direction = 1;

x_y_time =[];

for th1_i = 1:length(theta_vec)
    th1 = theta_vec(th1_i);
    for th2_i = 1:length(theta_vec) 
        th2 = theta_vec(th2_i);    
        [x,y] = FK(th1,th2,len1,len2);
%         [the1p, the2p, the1n, the2n] = inverseThe1_2([x y]',len1,len2);
%         
%         keyboard
        for k = 1: size(goal_configs,1)            
            min_time = Inf;
            time = A{th1_i,th2_i,k,1,1}; % time from belt to goal
            if time < min_time
                min_time = time;
            end  
        end
        x_y_time = [x_y_time;x,y,min_time];
    end
end

keyboard
surf(x_y_time(:,1),x_y_time(:,2),diag(x_y_time(:,3)))

end
    
function [x,y] = FK(the1,the2,len1,len2)

x1 = cos(the1)*len1;
y1 = sin(the1)*len1;
x = cos(the2+the1)*len2 + x1;
y = sin(the2+the1)*len2 + y1;

end


function [the1p, the2p, the1n, the2n] = inverseThe1_2(points,len1,len2)

    xx = points(1,:);
    yy = points(2,:);
    
    I = ones(size(xx));
    
    c2 = (xx.^2 + yy.^2 - len1^2 - len2^2) / ...
                (2*len1*len2);
     
    if abs(c2) > 1
        disp('Circle Position is out of reachable workspace')
        return
    end
            
    %c2 = cos(inside2);
    s2p = sqrt(1-c2.^2);
    s2n = -sqrt(1-c2.^2);
     
    the2p = atan2(s2p,c2);
    the2n = atan2(s2n,c2);
    
    
    k1 = len1 + len2*c2;
    k2p = len2*s2p;
    k2n = len2*s2n;
    
    the1p = atan2(yy,xx) - ...
              atan2(k2p,k1);
          
    the1n = atan2(yy,xx) - ...
              atan2(k2n,k1);    

end