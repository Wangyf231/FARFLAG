function real=Astar2real2(mesh_point,Astar)
[a,~]=size(Astar);
real=zeros([a,2]);
for j=1:a
    for i=1:2
       real(j,i)=sum(mesh_point{i}(1:Astar(j,i)-1));
        
    end
end
end