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
robot = ScaraInit;
len1 = robot.l_1;
len2 = robot.l_2;

belt = ConvBelt;
small_goal_y = belt.small_goal_y;
big_goal_y = belt.big_goal_y;
small_belt_y = belt.small_belt_y;
big_belt_y = belt.big_belt_y;

% just to start

num_theta = 3;
dt = 2*pi/num_theta
theta_vec = -pi+dt:dt:pi;

% Not the real way to get the end vector or theta values
% num_theta_end = 2;
% Goal_vec = 0:2*pi/(num_theta_end-1):2*pi;

time = 1;
path = ones(5,3);

goal_iter = 1; % iteration counter for goal storage
goal_configs = [];

n = 20; % number of time steps
options.init = 1;
X0 = zeros(9*n+1,1);


save(['./Precompute/Paths_n=',num2str(n),'_numThe=',num2str(num_theta)],'A')


for g1 = theta_vec
    for g2 = theta_vec
        [xg,yg] = FK(g1,g2,len1,len2);
        if small_goal_y <= yg <= big_goal_y           
            % update goal configuration storage
            goal_config(goal_iter,1:2) = [g1,g2];
            goal_iter = goal_iter + 1;
            for th1 = theta_vec % cycle through goal theta 1
                for th2 = theta_vec                                      
                    [xB,yB] = FK(th1,th2,len1,len2);
                    if small_belt_y <= yB <= big_belt_y                        
                        start = [th1; th2; zeros(4,1)];
                        finish = [g1; g2; zeros(4,1)];                    
                        [X0 statePath stateVelocity d_delta T] = ...
                            RealOptimalPathFind(start,finish,options,X0,n);
                        the1_ind = round((th1 + pi)/dt);
                        the2_ind = round((th2 + pi)/dt);
                        A{the1_ind,the2_ind,goal_iter,1} = T;
                        A{the1_ind,the2_ind,goal_iter,2} = statePath;          
                    end
                end    
            end
        end        
    end
end

save(['./Precompute/Paths_n=',num2str(n),'_numThe=',num2str(num_theta)],'A','goal_config')

rmpath /Users/samtormey/matlab/RecycleRobot/2DPlot/

end


function [x,y] = FK(the1,the2,len1,len2)

    x1 = cos(the1)*len1;
    y1 = sin(the1)*len1;
    x = cos(the2+the1)*len2 + x1;
    y = sin(the2+the1)*len2 + y1;
    
end