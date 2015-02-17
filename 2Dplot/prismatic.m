function prismatic

start = -.3;
finish = -.8;
startvelocity = 0;
finishvelocity = 0;
n = 20;
dt = 1/(n-1);
M = 10;
I = 1;

% bang bang!
pos = zeros(n,1);
vel = zeros(n,1);
acc = zeros(n,1);

pos_cur = start;

s = sign(finish - start);

pos(1) = start; vel(1) = startvelocity; 
for i = 2:n
    
    ...
    if start < finish 
        if pos(i) < (start+finish)/2
            torque = -s*M;
            acc = -torque/I;
            
            % acc > 0
        else
            torque = s*M;
            acc = -torque/I;
            % acc < 0
        end
    else
        if pos(i) > (start+finish)/2
            % acc < 0
        else
            % acc > 0
            
        end
    end
end

% numerically integrate until we are half way

% turn torque around!