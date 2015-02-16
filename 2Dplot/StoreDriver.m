function StoreDriver

% A = (th1_B, th2_B2, th1_G, th2_G, ToP)
%     th1_B = theta1 * (num_theta/2pi)
%        where num_theta = number of thetas taken in discretization
%        and theta1 = first angle for initial belt configuration
%     th1_G = theta1(Goal) * (num_theta_G/2pi)
%        where num_x = number of discretizated elements of x
%     ToP = 1 for time, 2 for path

A = {};

addpath /Users/samtormey/matlab/RecycleRobot/2DPlot/

% Initialize robot and belt values
robot = ScaraInit;
len1 = robot.l_1;
len2 = robot.l_2;

belt = ConvBelt;
goal_y = belt.robo2goal;
belt_bottom = belt.robo2bottom;
belt_top = belt.robo2top;

% Create discretization of theta configurations
num_theta = 2;
dt = 2*pi/num_theta
theta_vec = -pi+dt:dt:pi;

n = 20; % number of time steps
options.init = 1;
X0 = zeros(9*n+1,1);

% generate goal region points
gps = 10;
goal_width = 2*sqrt((len1+len2)^2 - goal_y^2);
goal_points_x = linspace(-goal_width/2,goal_width/2,gps);
goal_points_y = goal_y*ones(1,gps);
points = [goal_points_x; goal_points_y];

% Inverse Kinematics
[the1p, the2p, the1n, the2n] = inverseThe1_2(points);
goal_configs = [the1p the2p; the1n the2n]'; 

for k = 1: length(goal_configs)
    % retrieve goal configuration
    goal_th1 = goal_configs(k,1);
    goal_th2 = goal_configs(k,2);
    
    for th1_i = 1:length(theta_vec)
        th1 = theta_vec(th1_i);
        
        for th2_i = 1:length(theta_vec)
            th2 = theta_vec(th2_i);            
            % check if starting configuration is on the belt
            [xB,yB] = FK(th1,th2,len1,len2);
            
            if belt_bottom <= yB <= belt_top
                start = [th1; th2; zeros(4,1)];
                finish = [goal_th1; goal_th2; zeros(4,1)];
                [X0 statePath stateVelocity d_delta T] = ...
                    RealOptimalPathFind(start,finish,options,X0,n);
                
                % store solutions
                A{th1_i,th2_i,k,1} = T;
                A{th1_i,th2_i,k,2} = statePath;
            
            else % if starting config is not on the belt
                A{th1_i,th2_i,k,1} = NaN;
                A{th1_i,th2_i,k,2} = NaN;
                
            end % if
        end % th1
    end % th2
end % k (goal goal_configs)


rmpath /Users/samtormey/matlab/RecycleRobot/2DPlot/

save(['./Precompute/Paths_n=',num2str(n),'_numThe=',num2str(num_theta)],'A','goal_config')

end


function [x,y] = FK(the1,the2,len1,len2)

    x1 = cos(the1)*len1;
    y1 = sin(the1)*len1;
    x = cos(the2+the1)*len2 + x1;
    y = sin(the2+the1)*len2 + y1;
    
end


function [the1p, the2p, the1n, the2n] = inverseThe1_2(points)

    global len1 len2
    
    xx = points(1,:);
    yy = points(2,:);
    
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


