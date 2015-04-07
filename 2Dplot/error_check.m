function error_check

robot = ScaraInit;
len1 = robot.l_1;
len2 = robot.l_2;

belt = ConvBelt;
goal_y = belt.robo2goal;
belt_bottom = belt.robo2bottom;
belt_top = belt.robo2top;

pit = load('Precompute/Controls_n=20_numThe=80_gps=5');
A = pit.A;
n = pit.n;

% Create discretization of theta configurations
num_theta = 80;
dt = 2*pi/num_theta;
theta_vec = -pi+dt:dt:pi;

% generate goal region points
gps = 5;
goal_width = 2*sqrt((len1+len2)^2 - goal_y^2);
goal_points_x = linspace(-goal_width/2,goal_width/2,gps);
goal_points_y = goal_y*ones(1,gps);
points = [goal_points_x; goal_points_y];

% Inverse Kinematics
[the1p, the2p, the1n, the2n] = inverseThe1_2(points,len1,len2);
goal_configs = [the1p the1n(2:end-1); the2p the2n(2:end-1)]'; % note this!

errors = [0 0 0];
diff = zeros(80*80*8,1);
time_vec = 0;
i = 0;

% belt2goal
% direction = 1;
% for th1 = 1:80
%     for th2 = 1:80
%         start = [theta_vec(th1) theta_vec(th2) 0 0]';
%         
%         for k = 1:8
%             goal = goal_configs(k,:);
%            
%             time = A{th1, th2, k, direction, 1};
%             if isnan(time) == 0
%                 control = A{th1, th2, k, direction, 2};
%                 path = control_to_position(control, n, start, time);
%                 end_path = path(n,:);
%                 
% %                 if norm(end_path - goal) > 1e-3
% %                     errors = [errors; th1, th2, k];
% %                 end
%                 i = i+1;
%                 diff(i) = norm(end_path - goal);
%                 time_vec(i) = time;
%                 
%             end
%                 
%         end
%         
%     end
% end

direction = 2;

for th1 = 1:80
    for th2 = 1:80
        for k = 1:8
            goalstart = [goal_configs(k,:) 0 0];           
            time = A{th1, th2, k, direction, 1};
            if isnan(time) == 0
                control = A{th1, th2, k, direction, 2};
                path = control_to_position(control, n, goalstart, time);
                end_path = path(n,:);
                
%                 if norm(end_path - goal) > 1e-3
%                     errors = [errors; th1, th2, k];
%                 end
                i = i+1;
                endpos = [theta_vec(th1) theta_vec(th2)];
                diff(i) = norm(end_path - endpos);
                time_vec(i) = time;
                
            end
                
        end
        
    end
end
    
    
    
keyboard

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

