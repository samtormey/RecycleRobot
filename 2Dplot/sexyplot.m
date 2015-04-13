function sexyplot

% pit = load('Precompute/ModUnitedFriendMatrix.mat');
% pit = load('Precompute/UnitedFriendMatrix.mat');
pit = load('Precompute/Controls_n=20_numThe=80_gps=5.mat') 
A = pit.A;
n = pit.n;

B = zeros(80,80,8,2);
robot = ScaraInit();
robot = ScaraInit;
len1 = robot.l_1;
len2 = robot.l_2;

plot_eh = 0;
err = .05;
n = 20;
Kp = -50;
Kv = Kp/2;
M = 10;

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
x_y_time_controllers = [];
tally = 0;
tally2 = 0;

for th1_i = 1:length(theta_vec)

    th1 = theta_vec(th1_i);
    for th2_i = 1:length(theta_vec)
        th2 = theta_vec(th2_i);    
        [x,y] = FK(th1,th2,len1,len2);
        
        % x_y_test
        counter = 0;
        for i = 1:size(x_y_time,1)
            if abs(x_y_time(i,1) - x) + abs(x_y_time(i,2) - y) < 1e-8 
                counter = 1;
                tally2 = tally2 + 1;
            end
        end
           
        if counter == 0
        
            [the1p, the2p, the1n, the2n] = inverseThe1_2([x y]',len1,len2);
            [th1_i_pos,th2_i_pos] = getBestStoredIndices(the1p, the2p, theta_vec);
            [th1_i_neg,th2_i_neg] = getBestStoredIndices(the1n, the2n, theta_vec);
            
            if th1_i ~= th1_i_pos && th1_i ~= th1_i_neg
                tally = tally + 1;
            end
            
                min_time = Inf;
                min_time_controllers = Inf;
            for k = 1: size(goal_configs,1)
                time_pos = A{th1_i_pos,th2_i_pos,k,1,1};

                time_neg = A{th1_i_neg,th2_i_neg,k,1,1}; 
                keyboard
                time_pos_controllers = controllers_Approx ( [the1p; the2p; 0; 0], [goal_configs(k,:)'; 0; 0], n, plot_eh, err, Kp, Kv, M, robot);     
                time_neg_controllers = controllers_Approx ( [the1n; the2n; 0; 0], [goal_configs(k,:)'; 0; 0], n, plot_eh, err, Kp, Kv, M, robot);
                B(th1_i,th2_i,k,1) = time_pos_controllers;
                B(th1_i,th2_i,k,2) = time_neg_controllers;
                
                if time_pos < min_time 

                    min_time = time_pos;
                end
%                 if time_pos_controllers < min_time_controllers
%                     min_time_controllers = time_pos_controllers;
%                 end  
%                 if time_neg_controllers < min_time_controllers
%                     min_time_controllers = time_neg_controllers;
%                 end  
                if time_neg < min_time
                    min_time = time_neg;
                end
%                 if min_time < 0
%                     keyboard
%                 end
            end
            if min_time < Inf
                x_y_time = [x_y_time;x,y,min_time];

                x_y_time_controllers = [x_y_time_controllers;x,y,min_time_controllers];
                min_time_controllers
            end
        end
        
    end
end

save('./Precompute/ControllerApprox','B','Kp','Kv','err','M');

X = x_y_time(:,1);
Y = x_y_time(:,2);
Z = x_y_time(:,3);
[XI YI ZI] = griddata(X,Y,Z,linspace(-2,2),linspace(0,2)');

figure(3)
trisurf(delaunay(X,Y),X,Y,Z)
% surf(XI,YI,ZI)
% plot_belt
% plot3D_SCARA(0,0,0);
view([0 0 90])

hold on

% [XIC YIC ZIC] = griddata(x_y_time_controllers(:,1),x_y_time_controllers(:,2),...
%     x_y_time_controllers(:,3),linspace(-2,2),linspace(0,2)');
% trisurf(delaunay(x_y_time_controllers(:,1),x_y_time_controllers(:,2)),...
%     x_y_time_controllers(:,1),x_y_time_controllers(:,2),x_y_time_controllers(:,3))



title('Optimal Path Time','Fontsize',19)
xlabel('x','Fontsize',20)
ylabel('y','Fontsize',20)
h = colorbar;
ylabel(h, 'Time (seconds) ','Fontsize',20)

% figure(2)
% surf(XI,YI,ZI)
% % plot_belt
% % plot3D_SCARA2(0,0,0);
% view([0 0 90])

title('Optimal Path Time','Fontsize',19)
xlabel('x','Fontsize',20)
ylabel('y','Fontsize',20)
h = colorbar;
ylabel(h, 'Time (seconds) ','Fontsize',20)
% 
% figure(2)
% surf(XI,YI,ZI)
% % view(2)
% colorbar

keyboard

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

function plot_belt
belt = ConvBelt;
goal_y = belt.robo2goal;
belt_bottom = belt.robo2bottom;
belt_top = belt.robo2top;
blr = belt.left_right;
height = belt.height;

top_corners = [-blr belt_bottom 0;
               -blr belt_top 0;
                blr belt_top 0;
                blr belt_bottom 0];
            
bottom_corners =  [-blr belt_bottom -height;
               -blr belt_top -height;
                blr belt_top -height;
                blr belt_bottom -height];
            
verts = [top_corners; bottom_corners];

faces = [1 2 3 4 4 4 4 4; 5 6 7 8 8 8 8 8; 2 1 5 6 6 6 6 6; ...
    2 3 7 6 6 6 6 6; 3 4 8 7 7 7 7 7; 4 1 5 8 8 8 8 8];

patch('Vertices',verts,'Faces',faces,'facecolor',[.5 .5 .5]);
axis([-2 2 -2 2 0 3])
end

