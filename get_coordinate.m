function [Device,coordinate]=get_coordinate(x,Opt_,oth)
if nargin<3
oth=1;
end
    Device=Opt_.Device;
    [~,c]=size(Device);
    coordinate=[];
    offset=reshape(x,[4,c])';
    for i=1:c  
        Device(i).offset=offset(i,1:2);
        Device(i).rot_sym=round(offset(i,3:4));
        Device(i).edge(:,1)=round(Device(i).edge(:,1)*(-1)^Device(i).rot_sym(2),3);
        Device(i).edge_true(:,1)=round(Device(i).edge_true(:,1)*(-1)^Device(i).rot_sym(2),3);

        royal=[cos(Device(i).rot_sym(1)*0.5*pi),-sin(Device(i).rot_sym(1)*0.5*pi);sin(Device(i).rot_sym(1)*0.5*pi),cos(Device(i).rot_sym(1)*0.5*pi)];
        Device(i).edge=round((royal*Device(i).edge')'+Device(i).offset,3);
        Device(i).edge_true=round((royal*Device(i).edge_true')'+Device(i).offset,3);

        coordinate=[coordinate;Device(i).edge];
         for point=Device(i).port_name
             point1=cell2mat(point);
             Device(i).port.(point1).coordinate(:,1)=Device(i).port.(point1).coordinate(:,1)*(-1)^Device(i).rot_sym(2);
             Device(i).port.(point1).coordinate=round((royal*Device(i).port.(point1).coordinate')'+Device(i).offset,3);

             Device(i).port.(point1).coordinate_true(:,1)=round(Device(i).port.(point1).coordinate_true(:,1)*(-1)^Device(i).rot_sym(2),3);
             Device(i).port.(point1).coordinate_true=round((royal*Device(i).port.(point1).coordinate_true')'+Device(i).offset,3);
             Device(i).port.(point1).port_area=[Device(i).port.(point1).coordinate+oth+Opt_.wire_length/2;Device(i).port.(point1).coordinate-oth-Opt_.wire_length/2];
             Device(i).port.(point1).port_area(Device(i).port.(point1).port_area<0)=0;
             coordinate=[coordinate;Device(i).port.(point1).coordinate; Device(i).port.(point1).port_area];%Device(i).port.(point1).coordinate+oth+Opt_.wire_length/2;Device(i).port.(point1).coordinate-oth-Opt_.wire_length/2];
         end
        % [~,c2]=size(min(Device(i).edge(:,2)):Opt_.A_Star.mesh(2):max(Device(i).edge(:,2)));
        % obstacle2=[repmat(Device(i).edge(2,1),[c2,1]),(min(Device(i).edge(:,2)):Opt_.A_Star.mesh(2):max(Device(i).edge(:,2)))'];
        % obstacle4=[repmat(Device(i).edge(4,1),[c2,1]),(min(Device(i).edge(:,2)):Opt_.A_Star.mesh(2):max(Device(i).edge(:,2)))'];
        % [~,c1]=size(min(Device(i).edge(:,1)):Opt_.A_Star.mesh(1):max(Device(i).edge(:,1)));
        % obstacle1=[(min(Device(i).edge(:,1)):Opt_.A_Star.mesh(1):max(Device(i).edge(:,1)))',repmat(Device(i).edge(1,2),[c1,1])];
        % obstacle3=[(min(Device(i).edge(:,1)):Opt_.A_Star.mesh(1):max(Device(i).edge(:,1)))',repmat(Device(i).edge(3,2),[c1,1])];
        % obstacle_temp=[obstacle1;obstacle2;obstacle3;obstacle4];
        % obstacle=[obstacle;obstacle_temp];
     

    end
    if ~isempty(fieldnames(Opt_.Port))
    [~,c]=size(Opt_.Port);
    for i=1:c 
        coordinate=[coordinate;Opt_.Port(i).coordinate];
    end
    end
end


