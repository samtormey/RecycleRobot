function SCARA_sim_driver

close all

global len1 len2

    %rng(180)  % one that breaks
    rng(90)  % interesting, works

    robot = ScaraInit;

    len1 = robot.l_1;
    len2 = robot.l_2;
    len3 = robot.l_3;
    numofCircles = 5;
    
    goalregion = [0;0;-1;zeros(3,1)];    
    
    Octo_Center(1,:) = len1*rand(1,numofCircles) + len2;
    Octo_Center(2,:) = rand(1,numofCircles)*pi;
    Octo_Center(3,:) = zeros(numofCircles,1);

    
    [Octo_Center(1,:) Octo_Center(2,:)] = pol2cart(Octo_Center(2,:),Octo_Center(1,:));    
     
    [the1p, the2p, the1n, the2n] = inverseThe1_2([Octo_Center(1,:); Octo_Center(2,:)])
    d3p = -1 + Octo_Center(3,:);
    d3n = d3p;
    
    oct = Octo_Center;    
    axislength = 2*(len1 + len2 + .5);
    figure
    hold on  
    axis([-axislength/2 axislength/2 -axislength/2 axislength])
    axis square
   
    
    %%% The plots need to change velocity according to T!!!! 
    %%% it is not just a time step forward, they need to know the velocity
    %%% to know how much to pause!!!
    
    %%% Maybe each plot could be .1 seconds, then we divide the change in
    %%% position to 
    
    

    
    options.init = 1;
    
    %%% may have options.n for RealOptimalPathFind
    
%     loops = 20*numofCircles;
%     M(loops) = struct('cdata',[],'colormap',[]);
    cnt = 1;    
    n = 20;
    
    
    for i = 1:numofCircles

 
         options.objective = 1;
        [statePath d_delta T] = RealOptimalPathFind(goalregion,...
            [the1p(i) the2p(i) d3p(i) zeros(1,3)]',options,[],n);                
       
        keyboard
        
        
        for j = 1:n
            hold off
            plot3D_SCARA(statePath(j,1),statePath(j,2),statePath(j,3));     
            for k = 1:numofCircles
                plot3D_OCTO(oct(1,k),oct(2,k),oct(3,k),0);
            end
            grid on
            pause(d_delta)

        end
        for j = 1:n
            hold off
            plot3D_SCARA(statePath(n-j+1,1),statePath(n-j+1,2),statePath(n-j+1,3)); 
            oct(1,i) = len1*cos(statePath(n-j+1,1)) + len2*cos(statePath(n-j+1,1) + statePath(n-j+1,2));
            oct(2,i) = len1*sin(statePath(n-j+1,1)) + len2*sin(statePath(n-j+1,1) + statePath(n-j+1,2));
            oct(3,i) = 1 + statePath(n-j+1,3);
            for k = 1:numofCircles
                plot3D_OCTO(oct(1,k),oct(2,k),oct(3,k),0);  % is is possible to only plot changing oct positions, on the belt, they will always be changing
            end       
            grid on
            pause(d_delta)
            
        end
        options.init = 1;
    end
    
    
    keyboard
    

end

function [the1p, the2p, the1n, the2n] = inverseThe1_2(Octo_Center)

    global len1 len2
    
    xx = Octo_Center(1,:);
    yy = Octo_Center(2,:);
    
    c2 = (xx.^2 + yy.^2 - len1^2 - len2^2) / ...
                (2*len1*len2);
     
    if abs(c2) > 1
        disp('Circle Position is out of reachable workspace')
        return
    end
            
    %c2 = cos(inside2);
    s2p = sqrt(1-c2.^2);
    s2n = -sqrt(1-c2.^2);
     
    the2p = atan2(s2p,c2);
    the2n = atan2(s2n,c2);
    
    
    k1 = len1 + len2*c2;
    k2p = len2*s2p;
    k2n = len2*s2n;
    
    the1p = atan2(yy,xx) - ...
              atan2(k2p,k1);
          
    the1n = atan2(yy,xx) - ...
              atan2(k2n,k1);    

end