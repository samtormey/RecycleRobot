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

for i = 1:num_rec
    rec_vert(:,:,i) = [start_rec(i),belt_bottom,.1;
                        start_rec(i), belt_top,.1;
                        start_rec(i)+rec_width, belt_top,.1;
                        start_rec(i)+rec_width, belt_bottom,.1];
end

%octox = -blr;  
octox = 0;

start = [2.1256 0 0 0]';
sgp_index = 1;
sol = [0; 0.8];
pit = load('Precompute/Controls_n=20_numThe=20_gps=4.mat');
A = pit.A;
n = pit.n;
maxiter = 50;

[time,control] = goal2belt_picker(sgp_index, sol, A, maxiter);
dt = time/(n-1);
path = control_to_position(control, n, start, time);


disp(path)




    for i = 1:100       
%         for j = 1:num_rec
%             if rec_vert(1,1,j) > blr
%                 rec_vert(:,1,j) = -blr*ones(4,1) + [0 0 rec_width rec_width]';
%             end
%             rec_vert(:,:,j) = rec_vert(:,:,j) + [.05*ones(4,1) zeros(4,2)]; 
%             if i > 1
%                  delete(h(j))
%             end
%             h(j) = patch(rec_vert(:,1,j),rec_vert(:,2,j),rec_vert(:,3,j));
%             pause(.001)
%         end
        octox = octox + v*dt;   
      
        if i < n+1
            plot3D_SCARA(path(i,1),path(i,2),0)
            grid on
        end
        if i > 1
            delete(g);
        end
        g = plot3D_OCTO(octox,.8,0,0);
        rectangle('Position',[-blr,belt_bottom,2*blr,belt_top],'FaceColor',[.5 .5 .5])
        pause(.1)
    end

end


