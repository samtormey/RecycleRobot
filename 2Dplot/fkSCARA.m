function [x,y,z] = fkSCARA(the1,the2, len1, len2)

% forward kinematix
x = len1*cos(the1) + len2*cos(the2+the1);
y = len1*sin(the1) + len2*sin(the2+the1);
z = 0;
end