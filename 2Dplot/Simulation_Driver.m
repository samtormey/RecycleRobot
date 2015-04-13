function Simulation_Driver
% Call the simulation to simulate the movement of the robot arm for
% different 

belt = ConvBelt;
v = belt.velocity;

end_octo_time = 100;
display = 1; % display = 0 for no simulation, 1 for simulation
maxiter = 5;

max_time_vec = [1.5, 2.5];
min_time_vec = [0.5, 1.5];
max_time = 1.5/v;
min_time = 0.8/v;

x_bar = [];
y_bar = [];
for j = 1:length(max_time_vec) 
    total_state_SPT = [0, 0];
    total_state_FIFO = [0, 0];
    
    max_time = max_time_vec(j);
    min_time = min_time_vec(j);
    
    for average_iter = 1:maxiter
        % SPT
        [time, numPerState] =  The_Simulation2(end_octo_time, display, 'SPT', max_time, min_time);
%         disp('SPT')
%         [numPerState(4), numPerState(5)]
        total_state_SPT = total_state_SPT + [numPerState(4), numPerState(5)];
        
        % FIFO, or 'Right'
        [time, numPerState] =  The_Simulation2(end_octo_time, display, 'Right', max_time, min_time);
%         disp('FIFO')
%         [numPerState(4), numPerState(5)]
        total_state_FIFO = total_state_FIFO + [numPerState(4), numPerState(5)];
    end
    avg_SPT = total_state_SPT(1)/(total_state_SPT(1) + total_state_SPT(2));
    avg_FIFO = total_state_FIFO(1)/(total_state_FIFO(1) + total_state_FIFO(2));
    
    x_bar = [x_bar; max_time - min_time];
    y_bar = [y_bar; avg_SPT, avg_FIFO];
end

clf
bar(x_bar,y_bar)


% fprintf('\nFor time = %f run %d times,', end_octo_time, maxiter)
% fprintf('\nFIFO: Goal: %d, Ungrabbed: %d, Percentage: %f',total_state_FIFO(1), ...
%     total_state_FIFO(2), total_state_FIFO(3))
%    
% fprintf('\nSPT: Goal: %d, Ungrabbed: %d, Percentage: %f\n',total_state_SPT(1), ...
%     total_state_SPT(2), total_state_SPT(3))
% 

%     clc;
% fprintf('\nTime of Octo Output: %f', end_octo_time)
% fprintf('\nNumber in goal region: %d', numPerState(4))
% fprintf('\nNumber fallen off belt: %d\n', numPerState(5))
%         disp(['# Invisibles:        ' num2str(numPerState(1))]);
%         disp(['# Visible on belt:   ' num2str(numPerState(2))]);
%         disp(['# Attached to robot: ' num2str(numPerState(3))]);
%         disp(['# In goal region:    ' num2str(numPerState(4))]);
%         disp(['# Fallen off belt:   ' num2str(numPerState(5))]);

end