function point_area=get_point_area(center_point,Opt_,oth)
if nargin<3
oth=1;
end

port_area=[center_point+oth+Opt_.wire_length/2;center_point-oth-Opt_.wire_length/2];
port_area_Astar=real2Astar2(Opt_.A_Star.mesh_point,port_area);
[X_grid, Y_grid] = meshgrid(min( port_area_Astar(:,1)):max( port_area_Astar(:,1)), min( port_area_Astar(:,2)):max( port_area_Astar(:,2)));
point_area_temp=[X_grid(:),Y_grid(:)];
if ismember([0,0],point_area_temp,"rows")
point_area=center_point;
else

point_area=point_area_temp;
end
end