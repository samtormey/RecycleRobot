function ControllersDriver

% A = (th1, th2, k, Direction, ToP)
%     th1 = theta1 * (num_theta/2pi)
%        where num_theta = number of thetas taken in discretization
%        and theta1 = first angle for initial belt configuration
%     k   =  index of the goal point
%     Direction = 1 for belt to goal, 2 for goal to belt
%     ToP = 1 for time, 2 for control

% notice there is structure to the NaNs in A, we could reduce storage if 
% we needed to

clc;
A = {};

addpath /Users/samtormey/matlab/RecycleRobot/2DPlot/

% Initialize robot and belt values
robot = ScaraInit;
len1 = robot.l_1;
len2 = robot.l_2;
err = .01;

belt = ConvBelt;
goal_y = belt.robo2goal;
belt_bottom = belt.robo2bottom;
belt_top = belt.robo2top;

% Create discretization of theta configurations
num_theta = 80;
dt = 2*pi/num_theta;
theta_vec = -pi+dt:dt:pi;

n = 20; % number of time steps
cnt = 0;
error = 0;

total_time = 0; % computational time

% generate goal region points
gps = 5;
goal_width = 2*sqrt((len1+len2)^2 - goal_y^2);
goal_points_x = linspace(-goal_width/2,goal_width/2,gps);
goal_points_y = goal_y*ones(1,gps);
points = [goal_points_x; goal_points_y];

% Inverse Kinematics
[the1p, the2p, the1n, the2n] = inverseThe1_2(points,len1,len2);
goal_configs = [the1p the1n(2:end-1); the2p the2n(2:end-1)]'; % note this!

A = cell(num_theta,num_theta,8,2,2);



for k = 1: size(goal_configs,1)
    % retrieve goal configuration
    goal_th1 = goal_configs(k,1);
    goal_th2 = goal_configs(k,2);
    
    for th1_i = 1:length(theta_vec)
        th1 = theta_vec(th1_i);
        
        for th2_i = 1:length(theta_vec)            
            
            th2 = theta_vec(th2_i);            
            % check if starting configuration is on the belt
            [xB,yB] = FK(th1,th2,len1,len2);
            
            if belt_bottom <= yB && yB <= belt_top
                cnt = cnt + 1;
                fprintf('\niter %d: ', cnt)
                start = [th1; th2; 0; 0];
                finish = [goal_th1; goal_th2; 0; 0];                
                tic
                [T_b2g, control_b2g] = controllers_Approx (  start,  finish, n, err, robot);     
                [T_g2b, control_g2b] = controllers_Approx (  finish, start,  n, err, robot);
                toc
%          
                A{th1_i,th2_i,k,1,1} = T_b2g;
                A{th1_i,th2_i,k,1,2} = control_b2g;
                A{th1_i,th2_i,k,2,1} = T_g2b;
                A{th1_i,th2_i,k,2,2} = control_g2b;
                                
            
            else % if starting config is not on the belt
                A{th1_i,th2_i,k,1,1} = NaN;
                A{th1_i,th2_i,k,1,2} = NaN;
                A{th1_i,th2_i,k,2,1} = NaN;
                A{th1_i,th2_i,k,2,2} = NaN;
                
            end % if
        end % th1
    end % th2
end % k (goal goal_configs)



save(['./Precompute/Controllers_3_Controls_n=',num2str(n),'_numThe=',num2str(num_theta)],'A',...
        'goal_configs','belt','n','robot','num_theta')

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


