% plot coverage of nxn scara discretizations
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



n = 80;
l1 = 1;
l2 = 1;

counter1 = 0;
counter2 = 0;
figure(1)
clf
hold on
axis equal

for i = 1:n
    for j = 1:n
        angle1 = i*2*pi/n;
        angle2 = j*2*pi/n;
        tip = [l1*cos(angle1) + l2*cos(angle2), ...
               l1*sin(angle1) + l2*sin(angle2)];
        if tip(2) < belt_top + 1e-3 && tip(2) > belt_bottom - 1e-3
            if counter1 == 0
                h1 = plot(tip(1),tip(2),'b*');
                counter1 = 1;
            else
                plot(tip(1),tip(2),'b*');
            end
        else
            if counter2 == 0
                h2 = plot(tip(1),tip(2),'r*');
                counter2 = 1;
            else
                plot(tip(1),tip(2),'r*');
            end
        end
    end
end

patch('Vertices',verts,'Faces',faces,'facecolor',[.5 .5 .5]);
plot3D_SCARA2(0.5,2,0);
axis([-2 2 -0.5 2])
axis off
title('\textbf{Discretized SCARA Coverage}','interpreter','latex','Fontsize',20)

print('SCARA_coverage','-depsc2')

% title(['Scara coverage with n = ' num2str(n) ' discretization'], ...
%     'fontsize',16)
       

