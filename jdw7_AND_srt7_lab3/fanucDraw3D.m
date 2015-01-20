function fanucDraw3D( path_file )
% MECH 498 - Intro to Robotics - Spring 2014
% Lab 3 - Inverse Kinematics
%
%    DESCRIPTION - Plot a graphical representation of the fanuc 560
%    Industrial robot with attached coordinate frames as it moves through a
%    series of poses defined by path_file.
%    
%    ADDITIONAL CODE NEEDED: lots

% Initialize the fanuc struct
fanuc = fanucInit();

% Get path position data
data = load(path_file);
s = data.s;

% Draw fanuc initially in zero position (do not change)
prev_angles = zeros(6,1);
fanuc.handles = drawFanuc(prev_angles,fanuc);
hold on;

% Select desired orientation for the tool



% Draw in 3D
count = 1;
n = size(s,2);
for t = 1:size(s,2)
    
    
    R = [1 0 0; 0 cos(s(4,t)) -sin(s(4,t)); 0 cos(s(4,t)) sin(s(4,t))];
    
    % Select desired position for the tool
    pos = s(1:3,t);
    T_goal = [R, pos; [0 0 0 1]] * inv(fanuc.tool); %#ok<MINV>
    
    % Solve inverse kinematics for nearest solution
    [is_sol, angles] = fanucIK(T_goal, prev_angles, fanuc);
    if (~is_sol)
        disp('Error- no solution')
        count = count + 1;
        continue;
    end
    
    % Move robot using setPuma() if solution exists
    setfanuc(angles,fanuc);
    
    % Plot a point at the tool tip
    T_actual = fanucFK(angles,fanuc) * fanuc.tool;
    point_actual = T_actual * [0,0,0,1]';
    point_actual(3) = point_actual(3) + 1000;  %fanuc.parameters.L_1;
    
    plot3(point_actual(1),point_actual(2),point_actual(3),'--rs');
    
    % Update previous joint angles
    prev_angles = angles;

end


end

