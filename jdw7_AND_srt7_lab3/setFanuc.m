function setFanuc( joint_angles, fanuc )
% MECH 498 - Intro to Robotics - Spring 2014
% Lab 3 - Inverse Kinematics
% 
%    DESCRIPTION - Update the position of the fanuc after calling drawFanuc()
% 
%    This function can be used as is one fanucFK() and drawFanuc() have been
%    completed.

[~,fanuc_T] = fanucFK(joint_angles,fanuc);
set(fanuc.handles(1),'Matrix',fanuc_T{1});
set(fanuc.handles(2),'Matrix',fanuc_T{2});
set(fanuc.handles(3),'Matrix',fanuc_T{3});
set(fanuc.handles(4),'Matrix',fanuc_T{4});
set(fanuc.handles(5),'Matrix',fanuc_T{5});
set(fanuc.handles(6),'Matrix',fanuc_T{6});
drawnow;

end
