function dist = distance(x1,y1,x2,y2,mesh_point)
%This function calculates the distance between any two cartesian 
%coordinates.
%   Copyright 2009-2010 The MathWorks, Inc.
dist = sum(mesh_point{1}(min(x1,x2):max(x1-1,x2-1)))+ sum(mesh_point{2}(min(y1,y2):max(y1-1,y2-1))); %Manhattan dist
% dist = sqrt((x1-x2)^2+(y1-y2)^2);
end

