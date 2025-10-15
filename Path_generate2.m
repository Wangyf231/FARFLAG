function [Optimal_path_total,path_total,path_combine_index,Opt_]=Path_generate2(x,Opt_,oth)
if nargin<3
oth=1;
end
device_num=length(x)/4;
for j=0:device_num-1
x(j*4+1:j*4+2)=round(x(j*4+1:j*4+2),3);
x(j*4+3:j*4+4)=(round(x(j*4+3:j*4+4)));

end
[Device,coordinate]=get_coordinate(x,Opt_,oth);
coordinate=[coordinate;0,0;Opt_.length,Opt_.high];
Port=Opt_.Port;
Opt_.A_Star.mesh_point=get_Astar_mesh(Opt_,coordinate);
obstacle=get_obstacle(Device,Opt_.A_Star.mesh_point);
[~,c]=size(Device);
routed_point={};
Optimal_path_total=[];
%get new edge
coordinate_edge=[];
for i=1:c
    for point=Device(i).port_name
        point1=cell2mat(point);
        point_real=Device(i).port.(point1).coordinate;
        coordinate_edge=[coordinate_edge;point_real];
    end
    coordinate_edge=[coordinate_edge;Device(i).edge];
end
new_edge_max=max(coordinate_edge);
new_edge_min=min(coordinate_edge);
new_edge_real=[new_edge_min;new_edge_max(1),new_edge_min(2);new_edge_max;new_edge_min(1),new_edge_max(2)];
Opt_.new_edge=new_edge_real;
new_edge_Astar=real2Astar2(Opt_.A_Star.mesh_point,new_edge_real);
[~,c2]=size(min(new_edge_Astar(:,2)):max(new_edge_Astar(:,2)));
obstacle2=[repmat(new_edge_Astar(2,1),[c2,1]),(min(new_edge_Astar(:,2)):max(new_edge_Astar(:,2)))'];
obstacle4=[repmat(new_edge_Astar(4,1),[c2,1]),(min(new_edge_Astar(:,2)):max(new_edge_Astar(:,2)))'];
[~,c1]=size(min(new_edge_Astar(:,1)):max(new_edge_Astar(:,1)));
obstacle1=[(min(new_edge_Astar(:,1)):max(new_edge_Astar(:,1)))',repmat(new_edge_Astar(1,2),[c1,1])];
obstacle3=[(min(new_edge_Astar(:,1)):max(new_edge_Astar(:,1)))',repmat(new_edge_Astar(3,2),[c1,1])];
new_edge_final=[obstacle1;obstacle2;obstacle3;obstacle4];
path_total={};
%routing
path_combine_index={};%for leMergeshape command

for i=1:c
    device=Device(i);
    for point=device.port_name
        path_combine_index_temp=[];

        point1=cell2mat(point);
        %
        checkpoint=strcat('Device(',num2str(i),').port.',point1);
        if ~isempty(find(strcmp(routed_point,checkpoint)))
            continue
        end
        routed_point{end+1}=checkpoint;
        Optimal_path=[];

        [~,b]=size(device.port.(point1).connect);
        for j=1:b
            connect_point1=cell2mat(device.port.(point1).connect(j));
            end_point_real=eval(connect_point1).coordinate;
            end_point_Astar=real2Astar2(Opt_.A_Star.mesh_point,end_point_real);
            routed_point{end+1}=connect_point1;
            if j==1

                begin_point_real=device.port.(point1).coordinate;
                begin_point_area_Astar=get_point_area(begin_point_real,Opt_,oth);
                end_point_area_Astar=get_point_area(end_point_real,Opt_,oth);
                obstacle1=setdiff(obstacle,[end_point_area_Astar;begin_point_area_Astar],'rows');
                begin_point_Astar=real2Astar2(Opt_.A_Star.mesh_point,begin_point_real);
                %obstacle1=setdiff(obstacle,[end_point_Astar;begin_point_Astar],'rows');
                [Optimal_path_temp,~,Nopath]=Routing_fun(Opt_,begin_point_Astar,end_point_Astar,obstacle1,10);
                if Nopath==0
                error('unable to route')
                end
                [path_real_temp,Optimal_path_temp]=path_legalization2(Optimal_path_temp,Opt_);


            else
                Optimal_path_real=Astar2real2(Opt_.A_Star.mesh_point,Optimal_path);
                begin_point_Astar=end_point_Astar;
                begin_point_real=end_point_real;

                
                [~,I]=pdist2(Optimal_path_real,end_point_real,'euclidean','Smallest',1);
                end_point_Astar=Optimal_path(I,:);
                end_point_real=Optimal_path_real(I,:);

                begin_point_area_Astar=get_point_area(begin_point_real,Opt_,oth);
                end_point_area_Astar=get_point_area(end_point_real,Opt_,oth);
                obstacle1=setdiff(obstacle,[end_point_area_Astar;begin_point_area_Astar],'rows');
                %obstacle1=setdiff(obstacle,[end_point_Astar;begin_point_Astar],'rows');
                [Optimal_path_temp,~,Nopath]=Routing_fun(Opt_,begin_point_Astar,end_point_Astar,obstacle1,10);
                if Nopath==0

                    connecting_port_real=[];
                    for n=1:length(eval(connect_point1).connect)

                    connecting_port_real=[connecting_port_real;eval(eval(connect_point1).connect{n}).coordinate];
                    end
                [~,I]=pdist2(connecting_port_real,end_point_real,'euclidean','Smallest',1);
                end_point_Astar=real2Astar2(Opt_.A_Star.mesh_point,connecting_port_real(I,:));
                begin_point_area_Astar=get_point_area(begin_point_real,Opt_,oth);
                end_point_area_Astar=get_point_area(connecting_port_real(I,:),Opt_,oth);
                obstacle1=setdiff(obstacle,[end_point_area_Astar;begin_point_area_Astar],'rows');
                %obstacle1=setdiff(obstacle,[end_point_Astar;begin_point_Astar],'rows');
                [Optimal_path_temp,~,Nopath]=Routing_fun(Opt_,begin_point_Astar,end_point_Astar,obstacle1,10);
                if Nopath==0
                error('unable to route')
                end                
                end
                [path_real_temp,Optimal_path_temp]=path_legalization2(Optimal_path_temp,Opt_);

            end
            if contains(connect_point1,'Port')

                if j==1
                    C=intersect(Optimal_path_temp,new_edge_final,'rows');
                    if ~isempty(C)
                        [~,II]=pdist2(C,begin_point_Astar,'euclidean','Smallest',1);
                        end_point_Astar=C(II,:);
                    end
                    end_point_real=Astar2real2(Opt_.A_Star.mesh_point,end_point_Astar);
                    begin_point_area_Astar=get_point_area(begin_point_real,Opt_,oth);
                    end_point_area_Astar=get_point_area(end_point_real,Opt_,oth);
                    obstacle1=setdiff(obstacle,[end_point_area_Astar;begin_point_area_Astar],'rows');
                    %obstacle1=setdiff(obstacle,[end_point_Astar;begin_point_Astar],'rows');
                    [Optimal_path_temp,~,Nopath]=Routing_fun(Opt_,begin_point_Astar,end_point_Astar,obstacle1,10);
                    if Nopath==0
                        error('unable to route')
                    end
                    [path_real_temp,Optimal_path_temp]=path_legalization2(Optimal_path_temp,Opt_);
                    eval([connect_point1,'.coordinate=Astar2real2(Opt_.A_Star.mesh_point,end_point_Astar)']);
                else
                    C=intersect(Optimal_path_temp,new_edge_final,'rows');
                    if ~isempty(C)
                        [~,II]=pdist2(C,begin_point_Astar,'euclidean','Smallest',1);
                        begin_point_Astar=C(II,:);
                    end
                    begin_point_real=Astar2real2(Opt_.A_Star.mesh_point,begin_point_Astar);
                    end_point_area_Astar=get_point_area(end_point_real,Opt_,oth);
                    begin_point_area_Astar=get_point_area(begin_point_real,Opt_,oth);
                    obstacle1=setdiff(obstacle,[end_point_area_Astar;begin_point_area_Astar],'rows');
                    %obstacle1=setdiff(obstacle,[end_point_Astar;begin_point_Astar],'rows');
                    [Optimal_path_temp,~,Nopath]=Routing_fun(Opt_,begin_point_Astar,end_point_Astar,obstacle1,10);
                    if Nopath==0
                        error('unable to route')
                    end
                    [path_real_temp,Optimal_path_temp]=path_legalization2(Optimal_path_temp,Opt_);
                    eval([connect_point1,'.coordinate=Astar2real2(Opt_.A_Star.mesh_point,begin_point_Astar)']);

                end
            end
            Optimal_path=[Optimal_path;Optimal_path_temp];
            Optimal_path_total=[Optimal_path_total;Optimal_path_temp];
            path_total{end+1}=path_real_temp;
            path_combine_index_temp(end+1)=length(path_total);
        end
        Optimal_path_obs=[];
        if ~isempty(Optimal_path)
            for iii=-10:10
                Optimal_path_obs=[Optimal_path_obs;Optimal_path(:,1)+iii,Optimal_path(:,2)];
            end
            Optimal_path_obs(Optimal_path_obs(:,1)<1,1)=1;
            Optimal_path_obs(Optimal_path_obs(:,1)>length(Opt_.A_Star.mesh_point{1})+1,1)=length(Opt_.A_Star.mesh_point{1})+1;

            for iii=-10:10
                Optimal_path_obs=[Optimal_path_obs;Optimal_path(:,1),Optimal_path(:,2)+iii];
            end
            Optimal_path_obs(Optimal_path_obs(:,2)<1,2)=1;
            Optimal_path_obs(Optimal_path_obs(:,2)>length(Opt_.A_Star.mesh_point{2})+1,2)=length(Opt_.A_Star.mesh_point{2})+1;
        end
        obstacle=[obstacle;unique(Optimal_path_obs,"rows")];
            path_combine_index{end+1}=path_combine_index_temp;

        
    end

end
Opt_.Port=Port;
end
