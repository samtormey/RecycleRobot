function The_Simulation 

close all
rng(4);
belt = ConvBelt;
goal_y = belt.robo2goal;
belt_bottom = belt.robo2bottom;
belt_top = belt.robo2top;
blr = belt.left_right;
height = belt.height;
v = belt.velocity;
robot = ScaraInit;
%robot.path = zeros(20,2);
starting_angles = [2.1256 0.0];
robot.path = [starting_angles(1)*ones(20,1), starting_angles(2)*ones(20,1)];
robot.pathCounter = 1;
robot.state = 'waiting';
robot.time = 1;
robot.curr_goal_index = 1;

len1 = robot.l_1;
len2 = robot.l_2;
num_rec = belt.num_rec;
rec_width = belt.rec_width;
h = zeros(num_rec,1);
d_fart = .1;

% generate goal region points
gps = 5;
goal_width = 2*sqrt((len1+len2)^2 - goal_y^2);
goal_points_x = linspace(-goal_width/2,goal_width/2,gps);
goal_points_y = goal_y*ones(1,gps);
points = [goal_points_x; goal_points_y];

% Inverse Kinematics
[the1p, the2p, the1n, the2n] = inverseThe(points,len1,len2);
goal_configs = [the1p the1n(2:end-1); the2p the2n(2:end-1)]'; % note this!

algo = 'Right';

real_time = 0;

start_rec = linspace(-blr,blr,num_rec);

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

figure(1); clf;
plot3D_SCARA(0,0,0);

% axis([-blr blr -blr blr 0 blr])
% grid on
% rectangle('Position',[-blr,belt_bottom,2*blr,belt_top],'FaceColor',[.5 .5 .5])

for i = 1:num_rec
    rec_vert(:,:,i) = [start_rec(i),belt_bottom,.1;
                        start_rec(i), belt_top,.1;
                        start_rec(i)+rec_width, belt_top,.1;
                        start_rec(i)+rec_width, belt_bottom,.1];
end


% initial octoprism

octo.state = 1; % 0 = inv_belt, 1 = vis_belt, 2 = robot, 3 = goal
octo.x = -blr;
octo.y = (belt_top - belt_bottom - 0.5)*rand + belt_bottom;
octo.z = 0;
octo.theta = 0;
octo.id = 1;
octos = octo;


max_time = 0.8/v; % max time difference between octoprisms appearing on the belt
min_time = 0.3/v;

new_octo = min_time; % time check for adding octoprisms

% num_octos = 500;
% for i = 2:num_octos
%     octo.x = octos(i-1).x - rand*max_space;
%     octo.y = (belt_top - belt_bottom - 0.5)*rand + belt_bottom;
%     octos = [octos octo];
%     octos(end).id = i;
% end



% 
%  pit = load('Precompute/UnitedFriendMatrix.mat');
%  A = pit.UnitedA;
pit = load('Precompute/Controls_n=20_numThe=80_gps=5.mat');

% pit = load('Precompute/Controllers_3_Controls_n=20_numThe=80.mat');

A = pit.A;
n = pit.n;
[num_goal_pts,~] = size(pit.goal_configs);
rng(1);
% generate goal region points
gps = 5;
goal_width = 2*sqrt((len1+len2)^2 - goal_y^2);
goal_points_x = linspace(-goal_width/2,goal_width/2,gps);
goal_points_y = goal_y*ones(1,gps);
points = [goal_points_x; goal_points_y];
goal_octos = zeros(5,1);

% Inverse Kinematics
[the1p, the2p, the1n, the2n] = inverseThe1_2(points,len1,len2);
goal_configs = [the1p the1n(2:end-1); the2p the2n(2:end-1)]'; % note this!
    
    
num_theta = 80;
dtheta = 2*pi/num_theta;
theta_vec = -pi+dtheta:dtheta:pi;


control_b2g = A{1,18,1,1,2};
time_b2g =  A{1,18,1,1,1};

%

test = 0;
test_octo = 0;

sim_counter = 1;

while real_time < 250
        real_time;
        tic
        
    % This if statement updates the robot state and what step it is on for
    % the current path. If the path is complete it finds a new path.
    if strcmp(robot.state,'goalToBelt')
    % Robot is moving from the goal to the belt
       if robot.pathCounter == n

            start = [robot.path(n,1) robot.path(n,2) 0 0]';
            [control,closest_goal_ind,time] = belt2goal_picker(A,start,num_goal_pts);   
            robot.path = control_to_position(control, size(control,1), start, time);  
            if size(robot.path,1) > n
                 temp = interp1(linspace(0,time,size(control,1)),robot.path,linspace(0,time,n)');               
                 robot.path = [temp(1:end-1,:); robot.path(end,:)];                   
            end
           
           robot.time = time;
           robot.pathCounter = 1;
           robot.state = 'beltToGoal';           
           robot.curr_goal_index = closest_goal_ind;
           octos(id).state = 2;
       else
           robot.pathCounter = robot.pathCounter + 1;

       end
    elseif strcmp(robot.state, 'beltToGoal')
       % Robot is moving from the belt to the goal
       if robot.pathCounter == n
           robot.state = 'waiting'; 
           octos(id).state = 3;
           
           % store the goal to be plotted
            [xx,yy,zz] = fkSCARA(robot.path(robot.pathCounter,1),robot.path(robot.pathCounter,2),len1,len2);
           
            [val, ind] = min(abs(points(1,:) - xx));
            if val < 0.1;
               goal_octos(ind) = 1; 
            else
               fprintf('Something aint right!')
               keyboard
            end
       else
           robot.pathCounter = robot.pathCounter + 1;
  
       end
    end
    
    
    if strcmp(robot.state, 'waiting') 
       
           [id, control, time] = decisionAlgo (octos,robot,A,algo);    

           if id ~= 0 % there is a reachable octoprism
               start = [pit.goal_configs(robot.curr_goal_index,:) 0 0]';
               robot.path = control_to_position(control, size(control,1), start, time);
               if size(robot.path,1) > n
                   temp = interp1(linspace(0,time,size(control,1)),robot.path,linspace(0,time,n)');               
                   robot.path = [temp(1:end-1,:); robot.path(end,:)];                   
               end
               robot.pathCounter = 1;
               robot.time = time;

               robot.state = 'goalToBelt';          
           end
    end
    
    dt = robot.time/(n-1);
    
    % octos on the belt move right
    
    plot3D_SCARA(robot.path(robot.pathCounter,1),robot.path(robot.pathCounter,2),-1)
    grid on
    patch('Vertices',verts,'Faces',faces,'facecolor',[.5 .5 .5]);
   
    octos_plotted = 0;
    for k = 1:numel(octos)
        if octos(k).state == 0 || octos(k).state == 1
            octos(k).x = octos(k).x + v*dt;
            if octos(k).x > blr + 1;
                octos(k).state = 4;
            end
        end
        if octos(k).state == 0 && octos(k).x > -blr
            octos(k).state = 1;
        end
        if octos(k).state == 2
            [xx,yy,zz] = fkSCARA(robot.path(robot.pathCounter,1),robot.path(robot.pathCounter,2),len1,len2);
            octos(k).x = xx;
            octos(k).y = yy;            
        end
        if octos(k).state == 3
%             [xx,yy,zz] = fkSCARA(robot.path(robot.pathCounter,1),robot.path(robot.pathCounter,2),len1,len2);
%            
%             [val, ind] = min(points(1,:) - xx);
%             if val < 0.1;
%                goal_octos(ind) = 1; 
%                keyboard
%             else
%                fprintf('Something aint right!')
%                keyboard
%             end
           
        end
        % if octo has not fallen off (fall off == state 4)
        if octos(k).state < 3
            plot3D_OCTO(octos(k).x,octos(k).y,octos(k).z,octos(k).theta);
            octos_plotted = octos_plotted + 1;
        end

    end
    
    % plot goal region
    for i = 1:5
        if goal_octos(i) == 1
            plot3D_OCTO(points(1,i),points(2,i),0,0);
            octos_plotted = octos_plotted + 1;
        end
    end
    
    patch('Vertices',verts,'Faces',faces,'facecolor',[.5 .5 .5]);
    
    % Update octoprism struct
    if real_time > new_octo
        % add octo to struct
        octo.x = -blr;
        octo.y = (belt_top - belt_bottom - 0.5)*rand + belt_bottom + 0.25;
       
        % make sure the octos don't overlap
        counter = 0;
        while ((octo.x - octos(end).x)^2 + (octo.y - octos(end).y)^2) < 0.5 && counter < 10
            octo.x = -blr;
            octo.y = (belt_top - belt_bottom - 0.5)*rand + belt_bottom + 0.25;
            counter = counter + 1;
        end
        
        octo.state = 1;
        octos = [octos octo];
        octos(end).id = octos(end-1).id + 1;
        new_octo = new_octo + min_time + rand*(max_time - min_time);
        
    end
   
%     if real_time > 10
%         profile viewer
% %         fprintf('T_robot = %f \n T_octo = %f \n T_plot = %f \n T_extra = %f \n', ...
% %             T_robot, T_octo, T_plot, extra_time)
%        keyboard 
%     end
    
    % code below print-logs the states of the octos
    if mod(sim_counter,10) == 0
        octo_n = numel(octos);
        state_array = zeros(octo_n,1);
        for j = 1:octo_n
            state_array(j) = octos(j).state;
        end
        
        % hardwired now to states 0 through 4, held at indices
        % 1 through 5
        numPerState = zeros(5,1);
        for j = 1:5
            % just a cute way of extracting frequencies
            numPerState(j) =  ...
                octo_n - nnz(state_array - (j-1)*ones(octo_n,1));
        end
        
%         disp(['Real time is:  ' num2str(real_time)]);
%         disp('');
%         disp(['# Invisibles:        ' num2str(numPerState(1))]);
%         disp(['# Visible on belt:   ' num2str(numPerState(2))]);
%         disp(['# Attached to robot: ' num2str(numPerState(3))]);
%         disp(['# In goal region:    ' num2str(numPerState(4))]);
%         disp(['# Fallen off belt:   ' num2str(numPerState(5))]);
%         disp('***');
        
        
        
    end        
    
    sim_counter = sim_counter + 1;
    
    comptime = toc;     
    if (dt - comptime) < 0
        disp('pause is neg')
        pause(.01)
    else
        pause(dt/2 - comptime) 
    end
   
    real_time = real_time + dt;
    fprintf('\noctos plotted = %d\n', octos_plotted)

end
    

end 


% return id of next octo or 0 if no next octo is desirable
% accept the array of current octoprisms
function [best_id, control, shortest_time] = decisionAlgo(octos,robot,A,algo)

curr_num_octo = numel(octos);
maxiter = 30; % 120 earlier
% alpha is a toggle, bigger alpha means smaller gap in search path
alpha = 2; % 8 earlier
best_id = 0;
control = 0;
time = 0;
shortest_time = Inf;

if strcmp(algo,'SPT')
    for i = 1:curr_num_octo

        if octos(i).state == 1 && norm([octos(i).x octos(i).y]) < robot.l_1 + robot.l_2
            % alpha is a toggle, bigger alpha means smaller gap in search path
            
            [temp_control, time] = goal2belt_picker(robot.curr_goal_index, ...
                [octos(i).x; octos(i).y], A, maxiter, alpha);

            if time < shortest_time
                best_id = octos(i).id;
                control = temp_control;
                shortest_time = time;
            end
        end


    end
end


if strcmp(algo,'Right')
    
    X = -1000;

    for i = 1:curr_num_octo
      
        if octos(i).state == 1
            [temp_control, time] = goal2belt_picker(robot.curr_goal_index, ...
                [octos(i).x; octos(i).y], A, maxiter, alpha);     
           
            if time < Inf && X < octos(i).x
                best_id = octos(i).id;
                control = temp_control;
                shortest_time = time;
                X = octos(i).x;
            end
        end


    end
    
end


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



% sol = [octo.x; octo.y];
% 
% 
% sgp_index = 1;
% start = [pit.goal_configs(sgp_index,:) 0 0]';
% 
% 
% maxiter = 50;
% 
% [control, time] = goal2belt_picker(sgp_index, sol, A, maxiter);
% 
%  path = control_to_position(control, n, start, time);
%  dt = time/(n-1);
% 
% 
% for i = 1:2*n    
% 
%     if strcmp(octo.state,'belt')
%         octo.x = octo.x + v*dt;
%     end
%     if strcmp(octo.state,2)
%         [octo.x,octo.y,octo.z] = fkSCARA(path(i-n,1),path(i-n,2),len1,len2);      
%     end        
% 
%     if i < n+1
%         plot3D_SCARA(path(i,1),path(i,2),-1)
%         grid on
%     end
% 
% 
%     % the time it takes to compute this makes the simulation stop
%     % briefly        
%     if i == n+1 
%         current_config = [path(n,1) path(n,2) 0 0];
%         [control,closest_goal_ind,time] = belt2goal_picker(A,current_config,num_goal_pts);                 
%         path = control_to_position(control, n, current_config, time);
%         dt = time/(n-1);
% 
%     end
%     if i >= n+1
%         [octo.x,octo.y,octo.z] = fkSCARA(path(i-n,1),path(i-n,2),len1,len2);      
%         plot3D_SCARA(path(i-n,1),path(i-n,2),-1)               
%         g = plot3D_OCTO(octo.x,octo.y,octo.z,octo.theta);
%         patch('Vertices',verts,'Faces',faces,'facecolor',[.5 .5 .5]);            
%         grid on
%     end
% 
%     if i == n+1 % should be if distance between arm and octo is small
%         octo.state = 2;
%     end
% 
% 
%     g = plot3D_OCTO(octo.x,octo.y,octo.z,octo.theta);
% 
%     patch('Vertices',verts,'Faces',faces,'facecolor',[.5 .5 .5]);
% 
%     pause(.05)
% end
% 
% end
    





% grid on
% rectangle('Position',[-blr,belt_bottom,2*blr,belt_top],'FaceColor',[.5 .5 .5])
% 
%     for i = 1:100
%         for j = 1:num_rec
%             if i > 1
% %                 delete(h(j))
%             end
%             h(j) = rectangle('Position',[start_rec(j)+(2*blr)/i,belt_bottom,rec_width,belt_top], ...
%                 'FaceColor',[.3,.3,.3]);
%             pause(.001)
%         end
%         pause(.1)
%     end
% 
% end


