function Fourway_minimizer
% Take in the four different A matrices and find the minimum path across
% all of them.

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

theta_vec2pi = theta_vec;
theta_vec2pi(1:num_theta/2-1) = theta_vec2pi(1:num_theta/2-1) + 2*pi; % num_theta needs to be even
goal_configs2pi = goal_configs;
goal_configs2pi(6:8,2) = goal_configs2pi(6:8,2) + 2*pi; % this is positively shifted for 5 gps


pit = load('Precompute/Controls_n=20_numThe=80_gps=5');
A = pit.A;
n = pit.n;
robot = pit.robot;
num_theta = pit.num_theta;
goal_configs = pit.goal_configs;
belt = pit.belt;

pit = load('Precompute/Controls_n=20_numThe=80_gps=5_posshift1=0_posshift2=1.mat');
B = pit.A;
B_error = pit.error_list(:,1:3);
B_theta = pit.error_list(:,4:5);

pit = load('Precompute/Controls_n=20_numThe=80_gps=5_posshift1=1_posshift2=1.mat');
C = pit.A;
C_error = pit.error_list(:,1:3);
C_theta = pit.error_list(:,4:5);

pit = load('Precompute/Controls_n=20_numThe=80_gps=5_posshift1=1_posshift2=0.mat');
D = pit.A;
D_error = pit.error_list(:,1:3);
C_theta = pit.error_list(:,4:5);

B_test = 0;
C_test = 0;
D_test = 0;

second_error_list = [];
for k = 1:8
    for th1_i = 1:80
        for th2_i = 1:80
            B_check = sum(ismember(B_error,[k,th1_i,th2_i],'rows'));
            C_check = sum(ismember(C_error,[k,th1_i,th2_i],'rows'));
            D_check = sum(ismember(D_error,[k,th1_i,th2_i],'rows'));
            if B_check == 1
               B_test = B_test + 1; 
            end
            if C_check == 1
               C_test = C_test + 1; 
            end
            if D_check == 1
               D_test = D_test + 1; 
            end
            for direction = 1:2
                time(1) = A{th1_i,th2_i,k,direction,1};
                
                if B_check == 0 % There is no error in B
                    time(2) = B{th1_i,th2_i,k,direction,1};
                    if time(2) < 0
                        time(2) = Inf;
                    end
                end
                if C_check == 0 
                    time(3) = C{th1_i,th2_i,k,direction,1};
                    if time(3) < 0
                        time(3) = Inf;
                    end
                end
                if D_check == 0
                    time(4) = D{th1_i,th2_i,k,direction,1};
                    if time(4) < 0
                        time(4) = Inf;
                    end
                end
                if B_check + C_check + D_check > 0                             
                    th1 = theta_vec(th1_i);
                    th2 =theta_vec(th2_i);
                    start = [th1; th2; 0; 0];
                    goal_th1 = goal_configs(k,1); goal_th2 = goal_configs(k,2);
                    finish = [goal_th1; goal_th2; 0; 0]; 
                    [X control T exitflag comp_time statePath] = ModRealOptimalPathFind(start,finish,options,X0,n);
                    % Check if there is an error in this direction
                    % if there is no error, take the time
                    % if there is an error, set it to Inf
                    if exitflag == 1
                        time(2) = T;
                        time(3) = Inf;
                        time(4) = Inf;
                        mod_control = control;
                    else
                        time(2) = Inf;
                        second_error_list = [second_error_list; k, th1_i, th2_i, direction];
                        disp('Woops!')
                    end
                end
                                    
                
                [min_time,min_time_index] = min(time);
                
%                 if min_time < 0
%                     
%                     start = [th1; th2; 0; 0];
%                     if min_time_index == 2 
%                         goal_th1 = goal_configs2pi(k,1);
%                         goal_th2 = goal_configs2pi(k,2);
%                     elseif min_time_index == 3
%                         goal_th1 = goal_configs2pi(k,1);
%                         goal_th2 = goal_configs2pi(k,2);
%                     else
%                         goal_th1 = goal_configs(k,1);
%                         goal_th2 = goal_configs(k,2);
%                     end
%                     finish = [goal_th1; goal_th2; 0; 0];
%                     [X0 control_b2g T_b2g exitflag1 comp_time] = ...
%                         RealOptimalPathFind(start,finish,options,X0,n);
%                     keyboard
%                 end
                                
                if min_time_index == 2                              
                    if B_check + C_check + D_check == 0 
                        A{th1_i,th2_i,k,direction,1} = B{th1_i,th2_i,k,direction,1};
                        A{th1_i,th2_i,k,direction,2} = B{th1_i,th2_i,k,direction,2};
                    else
                        A{th1_i,th2_i,k,direction,1} = T;
                        A{th1_i,th2_i,k,direction,2} = mod_control;
                    end
%                   
                end
                if min_time_index == 3
                    A{th1_i,th2_i,k,direction,1} = C{th1_i,th2_i,k,direction,1};
                    A{th1_i,th2_i,k,direction,2} = C{th1_i,th2_i,k,direction,2};
%                     disp('got C')                    
                end
                if min_time_index == 4
                    A{th1_i,th2_i,k,direction,1} = D{th1_i,th2_i,k,direction,1};
                    A{th1_i,th2_i,k,direction,2} = D{th1_i,th2_i,k,direction,2};
%                     disp('got A')
                end
%                 if B_check + C_check + D_check > 0
%                     fprintf('\n min T I: %d \n diff in control: %f \n diff in time: %f\n', ...
%                     min_time_index, norm(A{th1_i,th2_i,k,direction,2} - mod_control), ...
%                     A{th1_i,th2_i,k,direction,1} - T);
%                 end
%                 if A{th1_i,th2_i,k,direction,1} < 0
%                     keyboard
%                 end
            end
        end
    end
end
size(B_error)
B_test
size(C_error)
C_test
size(D_error)
D_test
keyboard

UnitedA = A;

save('./Precompute/ModUnitedFriendMatrix','UnitedA',...
        'goal_configs','belt','n','robot','num_theta')
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


          