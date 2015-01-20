function [ fanuc_struct ] = fanucInit()
% MECH 498 - Intro to Robotics - Spring 2014
% Lab 3 - Inverse Kinematics
%
%    DESCRIPTION - Initialize a structure fanuc_struct to contain important
%    robot information that will be passed into various fanuc simulation
%    functions.
%
%    ADDITIONAL CODE NEEDED:
%
%    Add all dimensions that you will make use of to the nested structure,
%    fanuc_struct.parameters
%
%    Provide the transform to move from the tool frame to the end-effector
%    frame (frame {6}) in fanuc_struct.tool
%
%    Provide the limits of the workspace.
%

% fanuc dimensions in millimeters

L = 1000;
L_1 = 300; % [mm]
L_2 = 900; % [mm]
L_3 = 180;
L_4 = 1600;


% Fill in fanuc D-H parameters and other necessary parameters 
fanuc_struct.parameters.L = L;
fanuc_struct.parameters.L_1 = L_1;
fanuc_struct.parameters.L_2 = L_2;
fanuc_struct.parameters.L_3 = L_3;
fanuc_struct.parameters.L_4 = L_4;

fanuc_struct.parameters.a1 = pi/2;
fanuc_struct.parameters.a2 = pi/2;
fanuc_struct.parameters.a3 = -pi/2;
fanuc_struct.parameters.a4 = pi/2;

% Homogeneous transform expressing the tool frame relative to the
% end-effector frame
fanuc_struct.tool =  [1 0 0 0; 0 1 0 0; 0 0 1 25+55.8800; 0 0 0 1] * ...
                    [cos(-pi/4) -sin(-pi/4) 0 0; sin(-pi/4) cos(-pi/4) 0 0; ...
                       0 0 1 0; 0 0 0 1] * ...
                    [1 0 0 0; 0 1/sqrt(3) -sin(acos(1/sqrt(3))) 0; ...
                        0 sin(acos(1/sqrt(3))) 1/sqrt(3) 0; 0 0 0 1] * ...
                    [1 0 0 0; 0 1 0 0; 0 0 1 50; 0 0 0 1];

% fanuc base (zero) frame relative to the "station" frame
fanuc_struct.base = makehgtform('translate',[0,0,L_1]);

% fanuc joint limits (deg)
deg2rad = pi/180;
fanuc_struct.joint_limits{1} = [-150,150]*deg2rad;
fanuc_struct.joint_limits{2} = [-80,80]*deg2rad;
fanuc_struct.joint_limits{3} = [-80,80]*deg2rad;
fanuc_struct.joint_limits{4} = [-240,240]*deg2rad;
fanuc_struct.joint_limits{5} = [-220,220]*deg2rad;
fanuc_struct.joint_limits{6} = [-450,450]*deg2rad;

% Set bounds on the cartesian workspace of the fanuc for plotting in the
% form:  [ xmin, xmax, ymin, ymax, zmin, zmax]
fanuc_struct.workspace = [-4000, 4000, -4000, 4000, -10, 4000];


% Set colors to be drawn for each link and associated frame, including the
% tool
fanuc_struct.colors{1} = [0,0,0];
fanuc_struct.colors{2} = [0,0,0];
fanuc_struct.colors{3} = [0,0,0];
fanuc_struct.colors{4} = [0,0,0];
fanuc_struct.colors{5} = [0,0,0];
fanuc_struct.colors{6} = [0,0,0];
fanuc_struct.colors{7} = [0.7,0.1,0.5];



end

