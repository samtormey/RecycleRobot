function ModStoreDriver

% Solve real optimal path find with the pos and negative shifts of the x
% domain.
% A = (th1, th2, k, Direction, ToP)
%     th1 = theta1 * (num_theta/2pi)
%        where num_theta = number of thetas taken in discretization
%        and theta1 = first angle for initial belt configuration
%     k   =  index of the goal point
%     Direction = 1 for belt to goal, 2 for goal to belt
%     ToP = 1 for time, 2 for control
clc;
SingleMod(0,1);

end

function SingleMod (pos_shift1, pos_shift2)

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
num_theta = 80;
% num_theta = 4;
dt = 2*pi/num_theta;
theta_vec = -pi+dt:dt:pi;
theta_vec1 = theta_vec;
theta_vec2 = theta_vec;

n = 20; % number of time steps
options.init = 1;
X0 = zeros(9*n+1,1);
cnt = 0;
error = 0;
error_list = [];

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
goal_configs_old = goal_configs;

% create 0 to 2pi thetas and goals
theta_vec2pi = theta_vec;
theta_vec2pi(1:num_theta/2-1) = theta_vec2pi(1:num_theta/2-1) + 2*pi; % num_theta needs to be even
goal_configs2pi = goal_configs;
goal_configs2pi(6:8,2) = goal_configs2pi(6:8,2) + 2*pi; % this is positively shifted for 5 gps

if pos_shift1 == 1
    theta_vec1 = theta_vec2pi;
    
end 
if pos_shift2 == 1
    theta_vec2 = theta_vec2pi;
    goal_configs = goal_configs2pi;
end


for k = 1: size(goal_configs,1)
    % retrieve goal configuration
    goal_th1 = goal_configs(k,1);
    goal_th2 = goal_configs(k,2);
    
    for th1_i = 1:length(theta_vec1)
        th1 = theta_vec1(th1_i);
        
        for th2_i = 1:length(theta_vec2)            
            
            th2 = theta_vec2(th2_i);            
            
            if (((isequal(goal_configs_old(k,:),goal_configs(k,:))) && ...
                    (th1 == theta_vec(th1_i))) && (th2 == theta_vec(th2_i)))
                fprintf('No Compute!')
                A{th1_i,th2_i,k,1,1} = NaN;
                A{th1_i,th2_i,k,1,2} = NaN;
                A{th1_i,th2_i,k,2,1} = NaN;
                A{th1_i,th2_i,k,2,2} = NaN;
                
                
            else
                 
                % check if starting configuration is on the belt
                [xB,yB] = FK(th1,th2,len1,len2);
                
                if belt_bottom <= yB && yB <= belt_top
                    cnt = cnt + 1;
                    %                 fprintf('\niter %d: ', cnt)
                    start = [th1; th2; 0; 0];
                    finish = [goal_th1; goal_th2; 0; 0];
                    [X0 control_b2g T_b2g exitflag1 comp_time] = ...
                        RealOptimalPathFind(start,finish,options,X0,n);
                    [X0 control_g2b T_g2b exitflag2 comp_time] = ...
                        RealOptimalPathFind(finish,start,options,X0,n);
                    
                    if exitflag1 ~= 1 || exitflag2 ~= 1
                        error = 1;
                        fprintf('\nWARNING: fmincon solution not found for ')
                        fprintf('k = %d, th1_i = %d, th2_i = %d\n', k, th1_i, th2_i)
                        error_list =[error_list; k, th1_i, th2_i, th1, th2];
                    end
                    
                    % store solutions
                    A{th1_i,th2_i,k,1,1} = T_b2g;
                    A{th1_i,th2_i,k,1,2} = control_b2g;
                    A{th1_i,th2_i,k,2,1} = T_g2b;
                    A{th1_i,th2_i,k,2,2} = control_g2b;
                    
                    options.init = 1; % reuse initial guess
                    
                    total_time = total_time + comp_time;
                    
                else % if starting config is not on the belt
                    A{th1_i,th2_i,k,1,1} = NaN;
                    A{th1_i,th2_i,k,1,2} = NaN;
                    A{th1_i,th2_i,k,2,1} = NaN;
                    A{th1_i,th2_i,k,2,2} = NaN;
                    
                end % if
            end % if repeating computation
        end % th1
    end % th2
end % k (goal goal_configs)

if error == 1
   fprintf('There was an error in one of the computations') 
end

save(['./Precompute/Controls_n=',num2str(n),'_numThe=',num2str(num_theta),'_gps=',num2str(gps),'_posshift1=',num2str(pos_shift1),'_posshift2=',num2str(pos_shift2)],'A',...
        'goal_configs','belt','n','robot','num_theta','total_time','error_list')
rmpath /Users/samtormey/matlab/RecycleRobot/2DPlot/

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


