function FIFO
% Carry out the FIFO algorithm given a formula for the gripping time of
% objects.
clc;

algorithm = 'SPT ';
% Check algorithm
if algorithm == 'FIFO'
    alg = 0;
elseif algorithm == 'SPT '
    alg = 1;    
else
    fprintf('Not a proper algorithm\n')
    return
end

vb = 12.2; % cm/s?
lb = 200;  % length belt
wb = 30; % width belt

t0 = 0; tf = 100;
N = 1000;
dt = (tf-t0)/N;

% initialize object matrix:
% O[entrance time, x coordinate, current y coordinates, status]
numObjects = 100;
O = zeros(numObjects, 4);

O(:,1) = rand(numObjects,1)*tf; % uniformly distributed random time entrance
O(:,2) = rand(numObjects,1)*wb; % uniformly distributed random x values
% O(:,4) = ones(numObjects,1)*(-1);

% order objects by entrance time
[~,order] = sort(O(:,1)); 
O = O(order,:);

pickList = [];

t = t0;
iter = 0;
while t < tf
    % update y   
    O(:,3) = (t - O(:,1)) * vb;
    % check if objects are available
    grabZone = find(sqrt((O(:,2)-wb).^2 + (O(:,3)-lb/2+10).^2) < 80);
    avail = find(O(grabZone,4) == 0);
    avail = grabZone(avail);
    
    if isempty(avail) == 0  % there are objects on board
        if alg == 0 % fifo
            Obj = avail(1);
            pickList = [pickList; Obj, t];
            O(Obj,4) = 1;
            %fprintf('Object %d: t = %5.3f at (%5.3f,%5.3f), Grip time = %5.3f \n',...
            %    Obj,t,O(Obj,2),O(Obj,3),gripTime(O(Obj,2),O(Obj,3)))
            t = t + gripTime(O(Obj,2),O(Obj,3));
        elseif alg == 1  % spt
            [~, temp] = min(gripTime(O(avail,2),O(avail,3)));
            Obj = avail(temp);
            pickList = [pickList; Obj, t];
            O(Obj,4) = 1;
            t = t + gripTime(O(Obj,2),O(Obj,3));
        end
    else % no available objects to pick
        t = t + dt;
    end
end
if (t > tf && isempty(pickList) == 0) %(If it is equal we are kosher because it reached there by time steps)
    pickList = [pickList(1:(end-1),:)];
    t = t - gripTime(O(Obj,2),O(Obj,3));
end

fprintf('\nAlgorithm = %s \ntotal time = %5.2f',algorithm,t)
fprintf('\nTotal objects = %d \nObjects picked = %d\n', numObjects,length(pickList))

end


function time = gripTime(x,y)
% Compute the time it takes to grip an object at position (x,y).
alpha = 0.05;
time = alpha*sqrt(x.^2 + (y-90).^2);

end
