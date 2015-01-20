function setPuma( joint_angles, puma )
% MECH 498 - Intro to Robotics - Spring 2014
% Lab 3 - Inverse Kinematics
% 
%    DESCRIPTION - Update the position of the PUMA after calling drawPuma()
% 
%    This function can be used as is one pumaFK() and drawPuma() have been
%    completed.

[~,puma_T] = pumaFK(joint_angles,puma);
set(puma.handles(1),'Matrix',puma_T{1});
set(puma.handles(2),'Matrix',puma_T{2});
set(puma.handles(3),'Matrix',puma_T{3});
set(puma.handles(4),'Matrix',puma_T{4});
set(puma.handles(5),'Matrix',puma_T{5});
set(puma.handles(6),'Matrix',puma_T{6});
drawnow;

end

