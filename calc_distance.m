function dist = calc_distance(x1,x2,mesh_point)
%This function calculates the distance between any two cartesian 
%coordinates.
%   Copyright 2009-2010 The MathWorks, Inc.
[m,~]=size(x1);
dist=zeros([1,m]);
for i=1:m
    if isempty(min(x1(i,1),x2(1)):max(x1(i,1)-1,x2(1)-1))
        a=0; 
    else
        a=sum(mesh_point{1}(min(x1(i,1),x2(1)):max(x1(i,1)-1,x2(1)-1)));
    end
    if isempty(min(x1(i,2),x2(2)):max(x1(i,2)-1,x2(2)-1))
        b=0; 
    else
        b=sum(mesh_point{2}(min(x1(i,2),x2(2)):max(x1(i,2)-1,x2(2)-1)));
    end
    dist(i) =a+b;
            %dist(i) = sum(mesh_point{1}(min(x1(i,1),x2(1)):max(x1(i,1)-1,x2(1)-1)))+ sum(mesh_point{2}(min(x1(i,2),x2(2)):max(x1(i,2)-1,x2(2)-1))); %Manhattan dist

% dist = sqrt((x1-x2)^2+(y1-y2)^2);
end
end

