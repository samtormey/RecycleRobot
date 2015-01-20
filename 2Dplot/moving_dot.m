function moving_dot

close all

global circsize len1 len2 width


    circsize = 15;
    len1 = 2;
    len2 = 1;
    width = .1;
    numofCircles = 5;
    
    goalregion = [pi/2;0];
    
    
    %CirclePositions(1,:) = [.1:.1:.5, 0, .2];
    %CirclePositions(2,:) = [.1:.1:.5, 0, -.1];
    
    
%      CirclePositions(1,:) = len1*rand(1,numofCircles) + len2;
%      CirclePositions(2,:) = rand(1,numofCircles)*pi;

%     CirclePositions(1,:) = len1*.5 + len2;
%     CirclePositions(2,:) = -.5:numofCircles:.5;
    
%     [CirclePositions(1,:) CirclePositions(2,:)] = pol2cart(CirclePositions(2,:),CirclePositions(1,:));
    
    CirclePositions(1,:) = linspace(-.5,.5,numofCircles); 
    CirclePositions(2,:) = ones(1,numofCircles)*1.5;
  

    
    [the1p, the2p, the1n, the2n] = inverseThe1_2([CirclePositions(1,:); CirclePositions(2,:)])
    
    axislength = 2*(len1 + len2 + .5);

    figure
    hold on
    plot(0,0,'o','MarkerSize',20,'MarkerFaceColor','k')
        
    the1 = 0:.01:2*pi;
    the2 = -2*pi:.01:0;
    time = numel(the1);   
    
   
    
    for i = 1:numofCircles
        statePath = RealOptimalPathFind([goalregion' zeros(1,4)]',[the1p(i) the2p(i) zeros(1,4)]');
        n = size(statePath,1);
        for j = 1:n
            hold off
            plot2D_SCARA(statePath(j,1),statePath(j,2),j/n);
            plotcircle(CirclePositions);         
            axis([-axislength/2 axislength/2 -axislength/2 axislength])
            axis square
            pause(.2)
        end
        for j = 1:n
            hold off
            [x2 y2] = plot2D_SCARA(statePath(n-j+1,1),statePath(n-j+1,2),(1-j/n));           
            CirclePositions(:,i) = [x2; y2];
            plotcircle(CirclePositions);         
            axis([-axislength/2 axislength/2 -axislength/2 axislength])
            axis square
            pause(.2)
        end
        
    end

end

function [the1p, the2p, the1n, the2n] = inverseThe1_2(circlePositions)

    global len1 len2
    
    xx = circlePositions(1,:);
    yy = circlePositions(2,:);
    
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

function [x2 y2] = plot2D_SCARA(the1,the2,d3)

global len1 len2 width

    x1 = cos(the1)*len1;
    y1 = sin(the1)*len1;
    
    the1_perp = the1 - pi/2;
    the2_perp = the1 + the2 - pi/2;
    
    w1 = width*cos(the1_perp)/2;
    w2 = width*sin(the1_perp)/2;
    
    u1 = width*cos(the2_perp)/2;
    u2 = width*sin(the2_perp)/2;
    
    x2 = cos(the2+the1)*len2 + x1;
    y2 = sin(the2+the1)*len2 + y1;

    
    p1 = [w1, w2];
    p2 = [x1 + w1, y1 + w2];
    p3 = [x1 - w1, y1 - w2];
    p4 = -p1;
    
    q1 = [x1 y1] + [u1 u2];
    q2 = [x2 y2] + [u1 u2];
    q3 = [x2 y2] - [u1 u2];
    q4 = [x1 y1] - [u1 u2];
    
    fill([p1(1) p2(1) p3(1) p4(1)],[p1(2) p2(2) p3(2) p4(2)],'b')
    hold on
    fill([q1(1) q2(1) q3(1) q4(1)],[q1(2) q2(2) q3(2) q4(2)],'g')
    color = [1,0,0]*d3;
    plot(x2,y2,'o','Markersize',20,'MarkerFaceColor',color)


end



function plotcircle(positions)

% takes an array, 'positions' of circles where the first row are the x
% positions and the 2nd row are the y positions.

global circsize

    for i = 1:size(positions,2)
        plot(positions(1,i),positions(2,i),'o','Markersize',circsize,'MarkerFaceColor','b');
    end
    
end