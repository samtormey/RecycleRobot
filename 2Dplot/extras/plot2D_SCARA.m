
function plot2D_SCARA(the1,the2,d3)

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
