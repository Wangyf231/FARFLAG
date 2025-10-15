function mesh=get_Astar_mesh(Opt_,coordinate)
    coordinate_sort{1}=uniquetol(coordinate(:,1));
    coordinate_sort{2}=uniquetol(coordinate(:,2));
    mesh=cell([1,2]);
    for i=1:2
        before_sum=0;
        mesh_temp=[];
        coordinate_temp=coordinate_sort{i};
        [a,~]=size(coordinate_temp);
        for j=1:a
            point=coordinate_temp(j);
            point_mod=mod(point-before_sum,Opt_.A_Star.mesh(i));
            if point_mod==0
            index=round((point-before_sum)./Opt_.A_Star.mesh(i));
            mesh_temp=[mesh_temp,ones([1,index])*Opt_.A_Star.mesh(i)];

            before_sum=point;
            else
                index=fix((point-before_sum)./Opt_.A_Star.mesh(i));
                mesh_temp=[mesh_temp,ones([1,index])*Opt_.A_Star.mesh(i),point_mod];
                before_sum=point;
            end
        end
        mesh{i}=mesh_temp;
    end
end
