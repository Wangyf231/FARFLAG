function route_length=calc_route_length(path,mesh_point)
real_piont=Astar2real2(mesh_point,path);
deltaX=real_piont(2:end,1)-real_piont(1:end-1,1);
deltaY=real_piont(2:end,2)-real_piont(1:end-1,2);
route_length=sum(sqrt(deltaX.^2+deltaY.^2));
end