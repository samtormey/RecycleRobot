function testPathStorage

% A = (th1_B, th2_B2, th1_G, th2_G, ToP)
%     th1_B = theta1 * (num_theta/2pi)
%        where num_theta = number of thetas taken in discretization
%        and theta1 = first angle for initial belt configuration
%     th1_G = theta1(Goal) * (num_theta_G/2pi)
%        where num_x = number of discretizated elements of x
%     ToP = 1 for time, 2 for path


A = {};

% just to start

num_theta = 40;
dt = 2*pi/num_theta
theta_vec = dt:dt:2*pi;

% Not the real way to get the end vector or theta values
% num_theta_end = 2;
% Goal_vec = 0:2*pi/(num_theta_end-1):2*pi;

time = 1;
path = ones(5,3);

goal_iter = 1; % iteration counter for goal storage
goal_configs = [];

n = 20; % number of time steps
options.init = 1;
X0 = zeros(9*n+1);

for th1_i = 1: num_theta % cycle through theta 1
    th1 = theta_vec(th1_i);
    for th2_i = 1: num_theta 
        th2 = theta_vec(th2_i);
        for g1_i = 1: num_theta  % cycle through goal theta 1
            g1 = theta_vec(g1_i);
            for g2_i = 1: num_theta 
                g2 = theta_vec(g2_i);                
                if g2 > -1   %g1 and g2 are in goal region
                    start = [th1; th2; zeros(4,1)];
                    finish = [g1; g2; zeros(4,1)];
                    
                    [X0 statePath stateVelocity d_delta T] = ...
                        RealOptimalPathFind(start,finish,options,X0,n);
                    
                    A{th1_i,th2_i,goal_iter,1} = T;
                    A{th1_i,th2_i,goal_iter,2} = statePath;
                    if g2_i > 3
                        keyboard
                        options.init = 0;
                    end
                    
                    % update goal configuration storage
                    goal_config(goal_iter,1:2) = [g1,g2];
                    goal_iter = goal_iter + 1;
                    
                    % update initial guess
                end
                if goal_iter > 10
                    keyboard
                end
            end
        end
    end
end

end