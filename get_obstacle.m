function obstacle=get_obstacle(Device,mesh_piont)

    [~,c]=size(Device);
    obstacle=[];
    for i=1:c  
        edge_Astar=real2Astar2(mesh_piont,Device(i).edge);

        [X_grid, Y_grid] = meshgrid(min(edge_Astar(:,1)):max(edge_Astar(:,1)), min(edge_Astar(:,2)):max(edge_Astar(:,2)));
        obstacle=[obstacle;X_grid(:),Y_grid(:)];
        for point=Device(i).port_name
            point1=cell2mat(point);
            port_area_Astar=real2Astar2(mesh_piont,Device(i).port.(point1).port_area);
            [X_grid, Y_grid] = meshgrid(min( port_area_Astar(:,1)):max( port_area_Astar(:,1)), min( port_area_Astar(:,2)):max( port_area_Astar(:,2)));
            obstacle=[obstacle;X_grid(:),Y_grid(:)];

        end
%         edge_Astar=real2Astar2(mesh_piont,Device(i).edge);
%         [~,c2]=size(min(edge_Astar(:,2)):max(edge_Astar(:,2)));
%         obstacle2=[repmat(edge_Astar(2,1),[c2,1]),(min(edge_Astar(:,2)):max(edge_Astar(:,2)))'];
%         obstacle4=[repmat(edge_Astar(4,1),[c2,1]),(min(edge_Astar(:,2)):max(edge_Astar(:,2)))'];
%         [~,c1]=size(min(edge_Astar(:,1)):max(edge_Astar(:,1)));
%         obstacle1=[(min(edge_Astar(:,1)):max(edge_Astar(:,1)))',repmat(edge_Astar(1,2),[c1,1])];
%         obstacle3=[(min(edge_Astar(:,1)):max(edge_Astar(:,1)))',repmat(edge_Astar(3,2),[c1,1])];
%         obstacle_temp=[obstacle1;obstacle2;obstacle3;obstacle4];
%         obstacle=[obstacle;obstacle_temp];
    end
end