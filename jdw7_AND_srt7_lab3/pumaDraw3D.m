function pumaDraw3D( path_file )
% MECH 498 - Intro to Robotics - Spring 2014
% Lab 3 - Inverse Kinematics
%
%    DESCRIPTION - Plot a graphical representation of the PUMA 560
%    Industrial robot with attached coordinate frames as it moves through a
%    series of poses defined by path_file.
%    
%    ADDITIONAL CODE NEEDED: lots

% Initialize the puma struct
puma = pumaInit();

% Get path position data
data = load(path_file);
s = data.s;

% Draw PUMA initially in zero position (do not change)
prev_angles = zeros(6,1);
puma.handles = drawPuma(prev_angles,puma);
hold on;

% Select desired orientation for the tool
R = [1 0 0; 0 -1 0; 0 0 -1];

% Draw in 3D

for t = 1:size(s,2)
    
    % Select desired position for the tool
    pos = s(:,t);
    T_goal = [R, pos; [0 0 0 1]] * inv(puma.tool); %#ok<MINV>
    
    % Solve inverse kinematics for nearest solution
    [is_sol, angles] = pumaIK(T_goal, prev_angles, puma);

    if (~is_sol)
        disp('Error- no solution')
        continue;
    end
    
    % Move robot using setPuma() if solution exists
    setPuma(angles,puma);
    
    % Plot a point at the tool tip
    T_actual = pumaFK(angles,puma) * puma.tool;
    point_actual = T_actual * [0,0,0,1]';
    point_actual(3) = point_actual(3) + puma.parameters.L_1;
    
    plot3(point_actual(1),point_actual(2),point_actual(3),'--rs');

%     plot3(pos(1),pos(2),pos(3) + puma.parameters.L_1,'--gs');

    
    % Update previous joint angles
    prev_angles = angles;

end




end

