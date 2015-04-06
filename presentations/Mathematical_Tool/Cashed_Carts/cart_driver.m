function cart_driver
% Call the precomputed solutions to the time minimization problem for
% different methods (forward and backward euler) and different
% discretizations. Plot the positions and controls.

% Load file
file = 'BkWDn=40';
pit = load(file);
position = pit.position; dt = pit.dt; control = pit.control;
time = pit.time; n = pit.n;

% plotting position or control
pos = 0; 
con = 1;

% plot the position over time
if pos == 1
    
    % plot
    speed = 5;
    
    figure;
    plot(position(1),0,'o')
    hold on
    axis([-5,105,-0.1,0.1])
    title(strcat('Movement of The Cart Over Time for ',file),'Fontsize',12)
    xlabel('Cart Position','Fontsize',12)
    
    pause(dt/5)
    
    for i = 2:n
        plot(position(i),0,'o')
        pause(dt/5)
    end

end

% plot the control over time
if con == 1
    
    figure;
    xx = [0:dt:time]';
    plot(xx,control)
    
    title(strcat('Control Over Time for   ',file),'Fontsize',12)
    xlabel('Time','Fontsize',12)
    ylabel('Control','Fontsize',12)

end