function [ puma_struct ] = pumaInit()
% MECH 498 - Intro to Robotics - Spring 2014
% Lab 3 - Inverse Kinematics
%
%    DESCRIPTION - Initialize a structure puma_struct to contain important
%    robot information that will be passed into various PUMA simulation
%    functions.
%
%    ADDITIONAL CODE NEEDED:
%
%    Add all dimensions that you will make use of to the nested structure,
%    puma_struct.parameters
%
%    Provide the transform to move from the tool frame to the end-effector
%    frame (frame {6}) in puma_struct.tool
%
%    Provide the limits of the workspace.
%

% PUMA dimensions in millimeters
in2mm = 25.4;
L_1 = 26.45*in2mm; % [mm]
L_2 = 17.0*in2mm; % [mm]
L_3 = 5.5*in2mm;
L_4 = 17.05*in2mm;


% Fill in PUMA D-H parameters and other necessary parameters 
puma_struct.parameters.L_1 = L_1;
puma_struct.parameters.L_2 = L_2;
puma_struct.parameters.L_3 = L_3;
puma_struct.parameters.L_4 = L_4;

puma_struct.parameters.a1 = -pi/2;
puma_struct.parameters.a2 = -pi/2;
puma_struct.parameters.a3 = pi/2;
puma_struct.parameters.a4 = -pi/2;

% Homogeneous transform expressing the tool frame relative to the
% end-effector frame
puma_struct.tool =  [1 0 0 0; 0 1 0 0; 0 0 1 25+55.8800; 0 0 0 1] * ...
                    [cos(pi/4) -sin(pi/4) 0 0; sin(pi/4) cos(pi/4) 0 0; ...
                       0 0 1 0; 0 0 0 1] * ...
                    [1 0 0 0; 0 1/sqrt(3) -sin(-acos(1/sqrt(3))) 0; ...
                        0 sin(-acos(1/sqrt(3))) 1/sqrt(3) 0; 0 0 0 1] * ...
                    [1 0 0 0; 0 1 0 0; 0 0 1 50; 0 0 0 1];

% PUMA base (zero) frame relative to the "station" frame
puma_struct.base = makehgtform('translate',[0,0,L_1]);

% PUMA joint limits (deg)
deg2rad = pi/180;
puma_struct.joint_limits{1} = [-170,170]*deg2rad;
puma_struct.joint_limits{2} = [-225,45]*deg2rad;
puma_struct.joint_limits{3} = [-250,75]*deg2rad;
puma_struct.joint_limits{4} = [-135,135]*deg2rad;
puma_struct.joint_limits{5} = [-100,100]*deg2rad;
puma_struct.joint_limits{6} = [-180,180]*deg2rad;

% Set bounds on the cartesian workspace of the PUMA for plotting in the
% form:  [ xmin, xmax, ymin, ymax, zmin, zmax]
puma_struct.workspace = [-1200, 1200, -1200, 1200, -10, 1000];


% Set colors to be drawn for each link and associated frame, including the
% tool
puma_struct.colors{1} = [0,0,0];
puma_struct.colors{2} = [0,0,0];
puma_struct.colors{3} = [0,0,0];
puma_struct.colors{4} = [0,0,0];
puma_struct.colors{5} = [0,0,0];
puma_struct.colors{6} = [0,0,0];
puma_struct.colors{7} = [0.7,0.1,0.5];



end

