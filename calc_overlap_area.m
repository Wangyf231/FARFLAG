function area=calc_overlap_area(device1,device2)
coordinate1=device1.edge;
coordinate2=device2.edge;
coordinate1_botle=min(coordinate1);
size1=max(coordinate1)-min(coordinate1);
x1 = coordinate1_botle(1);
y1 = coordinate1_botle(2);
width1 = size1(1);
height1 = size1(2);
coordinate2_botle=min(coordinate2);
size2=max(coordinate2)-min(coordinate2);
x2 = coordinate2_botle(1);
y2 = coordinate2_botle(2);
width2 = size2(1);
height2 = size2(2);


endx = max(x1+width1,x2+width2);%xÖá×î´óÖµ
startx = min(x1,x2);%xÖá×îÐ¡Öµ
width = width1+width2-(endx-startx);%ÖØµþ¾ØÐÎ¿í


endy = max(y1+height1,y2+height2);%yÖá×î´óÖµ
starty = min(y1,y2);%yÖá×îÐ¡Öµ
height = height1+height2-(endy-starty);%ÖØµþ¾ØÐÎ¿í


if width<=0||height<=0
    area=-0.00001;
else
    area = width*height+0.00001;
    
end


% xmin = max(coordinate1(1,1), coordinate2(1,1));
% ymin = max(coordinate1(1,2), coordinate2(1,2));
% xmax = min(coordinate1(3,1), coordinate2(3,1));
% ymax = min(coordinate1(3,2), coordinate2(3,2));
% width = xmax - xmin;
% height = ymax - ymin;
% if width <= 0 || height <= 0
%     area=-0.1;
% else
%     area = width * height;
% end
end