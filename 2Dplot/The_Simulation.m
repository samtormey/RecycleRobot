function The_Simulation

close all
belt = ConvBelt;
goal_y = belt.robo2goal;
belt_bottom = belt.robo2bottom;
belt_top = belt.robo2top;
blr = belt.left_right;
v = belt.velocity;
robot = ScaraInit;
len1 = robot.l_1;
len2 = robot.l_2;
num_rec = belt.num_rec;
rec_width = belt.rec_width;
h = zeros(num_rec,1);

start_rec = linspace(-blr,blr,num_rec);

plot3D_SCARA(0,0,0);
axis([-blr blr -blr blr 0 blr])
grid on
rectangle('Position',[-blr,belt_bottom,2*blr,belt_top],'FaceColor',[.5 .5 .5])

    for i = 1:100
        for j = 1:num_rec
            if i > 1
%                 delete(h(j))
            end
            h(j) = rectangle('Position',[start_rec(j)+(2*blr)/i,belt_bottom,rec_width,belt_top], ...
                'FaceColor',[.3,.3,.3]);
        end
        pause(.1)
    end

end


