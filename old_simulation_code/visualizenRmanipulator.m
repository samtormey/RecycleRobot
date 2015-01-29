function visualizeOMPLRobot

    clear all
    close all


    global lengths
    global n


    lengths = [1]; %, 1, 1];
    goal = [2]; %[-1.13; -1.57; 1.57];
    n = 1;

    pit = load('SinglePathCalc.mat');
    X = pit.statePath'

%     for i = 1:n
%         X(i,:) = pit(:,2*i - 1);
%     end

    t = size(X,2);
    

    robot = OMPLRobotInit();
    robot2 = robot;
    drawOMPLRobot(goal,robot2,1);  
    robot.handles = drawOMPLRobot(X(:,1),robot,0);


    for i = 2:t
        angles = X(:,i);
        [~,robot_T] = OMPLFK(angles,robot);
        for i = 1:n
            set(robot.handles(i),'Matrix',robot_T{i});
        end
        drawnow;
        pause(1e-1); % adjustable pause in seconds
    end

end


function [ handles ] = drawOMPLRobot( joint_angles, robot, new )

global lengths n

[~,robot_T] = OMPLFK(joint_angles,robot);
% Plot scaling properties
origin_size = 20;
marker_size = 10;
vector_size = 0.05*max(abs(diff(reshape(robot.workspace,2,3))));

if new == 1
% Create figure window
figure('Color','w');
end 

% Create axes object
ax = axes('XLim',robot.workspace(1:2),'YLim',robot.workspace(3:4));

grid on;
axis equal;
xlabel('X (mm)','FontSize',16);
ylabel('Y (mm)','FontSize',16);

% Create frames and links
h = drawRobotFrame([0,0,0]);
hg = hggroup('Parent',ax);
set(h,'Parent',hg);
T(1) = hgtransform('Parent',ax,'Matrix',eye(4));
set(hg,'Parent',T(1));

% Create link 1 and frame 1

for i = 1:n

h = drawRobotFrame(robot.colors{i});
hg = hggroup('Parent',ax);
set(h,'Parent',hg);
L(i) = line([lengths(i),0,0],[0,0,0],[0,0,0],...
    'Color',robot.colors{i},'LineWidth',1.5);
set(L(i),'Parent',hg);
T(i+1) = hgtransform('Parent',T(i),'Matrix',robot_T{i});
set(hg,'Parent',T(i+1));

end



set(gcf,'Renderer','openGL');
drawnow;

% Return hgtransform handles
handles = T(2:end);


    function h = drawRobotFrame( color )
         
        % Plot reference frame
        X_b = [vector_size,0,0,1]';
        Y_b = [0,vector_size,0,1]';
        Z_b = [0,0,vector_size,1]';
        h(1) = line(0,0,0,'Marker','.','MarkerSize',origin_size,'Color',color);
        h(2) = line([0,X_b(1)],[0,X_b(2)],[0,X_b(3)],'LineWidth',1.5,'Color',color);
        h(3) = line([0,Y_b(1)],[0,Y_b(2)],[0,Y_b(3)],'LineWidth',1.5,'Color',color);
        h(4) = line([0,Z_b(1)],[0,Z_b(2)],[0,Z_b(3)],'LineWidth',1.5,'Color',color);
        h(5) = line(X_b(1),X_b(2),X_b(3),'LineWidth',1.5,'Marker','x','MarkerSize',marker_size,'Color',color);
        h(6) = line(Y_b(1),Y_b(2),Y_b(3),'LineWidth',1.5,'Marker','o','MarkerSize',marker_size,'Color',color);
        h(7) = line(Z_b(1),Z_b(2),Z_b(3),'LineWidth',1.5,'Marker','d','MarkerSize',marker_size,'Color',color);
    end


end

function [T, robot_T] = OMPLFK(joint_angles, ~)  


global lengths n

Trans{1} = dhtf(0,0,0,joint_angles(1));
T = Trans{1};

for i = 2:n
    
    Trans{i} = dhtf(0,lengths(i-1),0,joint_angles(i));
    T = T*Trans{i};
    
end

robot_T = Trans;

end

function [ robot ] = OMPLRobotInit(  )

global lengths n


robot.m_1 = 2; % [kg]
robot.m_2 = 1.2422; % [kg]
robot.m_3 = .7715;
robot.m_r1 = 0; % [kg]
robot.m_r2 = 0; % [kg]
robot.l_1 = 2; % [m]
robot.l_2 = 1.2422; % [m]
robot.l_3 = .7715;
robot.g = 9.81; % [m/s^2]
robot.tool = [];
robot.workspace = [-4, 4, -4, 4, -4, 4]; % only used to determine size of figure window
robot.colors = {[.25,.10,.111],[.23,.44,.10],[.200,.10,.40]};

end

