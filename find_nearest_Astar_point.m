function point_out=find_nearest_Astar_point(point_Astar,directions,Opt_)
meshx=Opt_.A_Star.mesh_point{1};
meshy=Opt_.A_Star.mesh_point{2};
mesh_edge=[length(meshx)+1,length(meshy)+1];
%point_Astar=real2Astar2(Opt_.A_Star.mesh_point,point_real);
%directions=[0,1];
if directions(1)==0
    xlist1=(point_Astar(1):mesh_edge(1))';
    point_edge1=[xlist1,repmat(point_Astar(2),[length(xlist1),1])];
    xout1=find(calc_distance(point_edge1,point_Astar,Opt_.A_Star.mesh_point)>5.5);
    point_out_edge1=point_edge1(xout1(1),:);
    xlist2=(point_Astar(1):-1:1)';
    point_edge2=[xlist2,repmat(point_Astar(2),[length(xlist2),1])];
    xout2=find(calc_distance(point_edge2,point_Astar,Opt_.A_Star.mesh_point)>5.5);
    point_out_edge2=point_edge2(xout2(1),:);



else
    ylist1=(point_Astar(2):mesh_edge(2))';
    point_edge1=[repmat(point_Astar(1),[length(ylist1),1]),ylist1];
    yout1=find(calc_distance(point_edge1,point_Astar,Opt_.A_Star.mesh_point)>5.5);
    point_out_edge1=point_edge1(yout1(1),:);

    ylist2=(point_Astar(2):-1:1)';
    point_edge2=[repmat(point_Astar(1),[length(ylist2),1]),ylist2];
    yout2=find(calc_distance(point_edge2,point_Astar,Opt_.A_Star.mesh_point)>5.5);
    point_out_edge2=point_edge2(yout2(1),:);

end
point_out=[point_out_edge1;point_out_edge2];

%point_Astar+mesh_edge.*directions
%Astar2real2(Opt_.A_Star.mesh_point,mesh_edge)
%calc_distance
%obj_ga3(solution,Opt_)
end