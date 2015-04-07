function Simulation_Driver
% Call the simulation to simulate the movement of the robot arm for
% different 
max_space_separation = 0.8; % maximum space allowed between octoprisms, in meters
min_space_separation = 0.3;
for iter = 22:22
    iter
    rng(iter)
    The_Simulation(max_space_separation, min_space_separation)
end

end