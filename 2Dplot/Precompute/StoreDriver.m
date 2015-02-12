function testPathStorage

% A = (th1, th2, x, y, ToP)
%     th1 = theta1 * (num_theta/2pi)
%        where num_theta = number of thetas taken in discretization
%     th2 = " "
%     x = x*num_x/width
%        where num_x = number of discretizated elements of x
%     ToP = 1 for time, 2 for path


A = {};

% just to start

num_theta_space = 2;
theta_vec = 0:2*pi/(num_theta_space-1):2*pi;

% Not the real way to get the end vector or theta values
num_theta_end = 1;
E_vec = 0:2*pi/(num_theta_end-1):2*pi;

time = 1;
path = ones(5,3);

n = 20; % number of time steps
options.init = 0;
X0 = zeros(9*n,1);

for th1_i = 1: num_theta_space % cycle through theta 1
    th1 = theta_vec(th1_i);
    for th2_i = 1: num_theta_space % cycle through theta 2
        th2 = theta_vec(th2_i);
        for e1_i = 1: num_theta_end  % cycle through x
            e1 = E_vec(e1_i);
            for e2_i = 1: num_theta_end % cycle through y
                e2 = E_vec(e2_i);
                
                start = [th1; th2; zeros(4,1)];
                finish = [e1_i; e2_i; zeros(4,1)];
                
                % get initial guess X0
                [testPath testVelocity testTorque dummy3 X0] = ...
                    approx_traj(n,16,start,finish);
         
                X0(6*n+1:9*n) = X0(6*n+1:9*n)/10;
                  
                [statePath stateVelocity d_delta T] = ...
                    RealOptimalPathFind(start,finish,options,X0,n);
                
                A{th1_i,th2_i,e1_i,e2_i,1} = T;
                A{th1_i,th2_i,e1_i,e2_i,2} = statePath;
                keyboard
                
                %update X0
%                 for i = 1:3
%       statePath(:,i) = X(i:6:6*n);
%       stateVelocity(:,i) = X((i+3):6:6*n);
%       control(:,i) = X(6*n+i:3:end-1);
%       T = X(end);
%     end
               
            end
        end
    end
end
keyboard

end