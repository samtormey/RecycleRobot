% plot coverage of nxn scara discretizations

n = 80;
l1 = 1;
l2 = 1;

hold on
axis equal

for i = 1:n
    for j = 1:n
        angle1 = i*2*pi/n;
        angle2 = j*2*pi/n;
        tip = [l1*cos(angle1) + l2*cos(angle2), ...
               l1*sin(angle1) + l2*sin(angle2)];
        plot(tip(1),tip(2),'b*');
    end
end

% title(['Scara coverage with n = ' num2str(n) ' discretization'], ...
%     'fontsize',16)
        