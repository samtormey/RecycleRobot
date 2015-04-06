function The_Simulation

close all
rng(1);
belt = ConvBelt;
goal_y = belt.robo2goal;
belt_bottom = belt.robo2bottom;
belt_top = belt.robo2top;
blr = belt.left_right;
height = belt.height;
v = belt.velocity;
robot = ScaraInit;
robot.path = zeros(20,2);
robot.pathCounter = 1;
robot.state = 'waiting';
robot.time = 1;
robot.curr_goal_index = 1;

len1 = robot.l_1;
len2 = robot.l_2;
num_rec = belt.num_rec;
rec_width = belt.rec_width;
h = zeros(num_rec,1);

% generate goal region points
gps = 5;
goal_width = 2*sqrt((len1+len2)^2 - goal_y^2);
goal_points_x = linspace(-goal_width/2,goal_width/2,gps);
goal_points_y = goal_y*ones(1,gps);
points = [goal_points_x; goal_points_y];

% Inverse Kinematics
[the1p, the2p, the1n, the2n] = inverseThe(points,len1,len2);
goal_configs = [the1p the1n(2:end-1); the2p the2n(2:end-1)]'; % note this!


checkk = 0;
cntt = 0;
loops = 1000;
M(loops) = struct('cdata',[],'colormap',[]);


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

plot3D_SCARA(0,0,0);
axis([-blr blr -blr blr 0 blr])
grid on
rectangle('Position',[-blr,belt_bottom,2*blr,belt_top],'FaceColor',[.5 .5 .5])

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



pit = load('Precompute/Controls_n=20_numThe=80_gps=5');
A = pit.A;
n = pit.n;
[num_goal_pts,~] = size(pit.goal_configs);

test = 0;
test_octo = 0;
while 1
    
    % This if statement updates the robot state and what step it is on for
    % the current path. If the path is complete it finds a new path.
    if strcmp(robot.state,'goalToBelt')
    % Robot is moving from the goal to the belt
       if robot.pathCounter == n
            start = [robot.path(n,1) robot.path(n,2) 0 0]';
            [control,closest_goal_ind,time,start] = belt2goal_picker(A,start,num_goal_pts); 
            robot.path = control_to_position(control, n, start, time);
           
            options.init = 2;
            X0 = 0;
%           [X control T exitflag comp_time] =  RealOptimalPathFind(start,[goal_configs(1,:) 0 0]',options,X0,n)
           
           
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
           
       else
           robot.pathCounter = robot.pathCounter + 1;
  
       end
    end
        
    if strcmp(robot.state, 'waiting') 
           
            algo = 'Right';
       
           [id, control, time] = decisionAlgo(octos,robot,A,algo);    
           
           
           if id ~= 0 % there is a reachable octoprism
               start = [pit.goal_configs(robot.curr_goal_index,:) 0 0]';
               robot.path = control_to_position(control, n, start, time);
               robot.pathCounter = 1;
               robot.time = time;
               robot.state = 'goalToBelt';               
           end
           
    end
   
    dt = robot.time/(n-1);
    % octos on the belt move right
    
    tic
    plot3D_SCARA(robot.path(robot.pathCounter,1),robot.path(robot.pathCounter,2),-1)
    grid on
    test = test + toc;
    
    for k = 1:numel(octos)
        if octos(k).state == 0 || octos(k).state == 1
            octos(k).x = octos(k).x + v*dt;
        end
        if octos(k).state == 0 && octos(k).x > -blr
            octos(k).state = 1;
        end
        if octos(k).state == 2
            [xx,yy,zz] = fkSCARA(robot.path(robot.pathCounter,1),robot.path(robot.pathCounter,2),len1,len2);
            octos(k).x = xx;
            octos(k).y = yy;
        end
        tic
        plot3D_OCTO(octos(k).x,octos(k).y,octos(k).z,octos(k).theta); 
        test_octo = test_octo + toc;
    end
   
        
    patch('Vertices',verts,'Faces',faces,'facecolor',[.5 .5 .5]);
    
    pause(dt/10)
    
    real_time = real_time + dt;
    
    
    % Update octoprism struct
    if real_time > new_octo
        % add octo to struct
        octo.x = -blr;
        octo.y = (belt_top - belt_bottom - 0.5)*rand + belt_bottom + 0.25;
       
%         % make sure the octos don't overlap
%         counter = 0;
%         while ((octo.x - octos(end).x)^2 + (octo.y - octos(end).y)^2) < 0.5 && counter < 10
%             octo.x = -blr;
%             octo.y = (belt_top - belt_bottom - 0.5)*rand + belt_bottom + 0.25;
%             counter = counter + 1;
%         end
        
        octo.state = 1;
        octos = [octos octo];
        octos(end).id = octos(end-1).id + 1;
        new_octo = new_octo + min_time + rand*(max_time - min_time);
        
        

    end
  
end
    

end 


% return id of next octo or 0 if no next octo is desirable
% accept the array of current octoprisms
function [best_id, control, shortest_time] = decisionAlgo(octos,robot,A,algo)

curr_num_octo = numel(octos);
maxiter = 50;
best_id = 0;
control = 0;
time = 0;
shortest_time = Inf;

if strcmp(algo,'SPT')
    for i = 1:curr_num_octo

        if octos(i).state == 1 && norm([octos(i).x octos(i).y]) < robot.l_1 + robot.l_2
            [temp_control, time] = goal2belt_picker(robot.curr_goal_index, ...
                [octos(i).x; octos(i).y], A, maxiter);
            
            if time < shortest_time
                best_id = octos(i).id;
                control = temp_control;
                shortest_time = time;
            end
        end


    end
if size(time_vec,1) > 0
    keyboard
end
end


if strcmp(algo,'Right')
    
    X = -1000;

    for i = 1:curr_num_octo

        if octos(i).state == 1 && norm([octos(i).x octos(i).y]) < robot.l_1 + robot.l_2
            [temp_control, time] = goal2belt_picker(robot.curr_goal_index, ...
                [octos(i).x; octos(i).y], A, maxiter);
            
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


