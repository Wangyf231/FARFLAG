function dis=obj_ga_only_rout(x,Opt_)%
%disp(x)
%tic
device_num=length(x)/4;
for j=0:device_num-1
x(j*4+1:j*4+2)=round(x(j*4+1:j*4+2),3);
x(j*4+3:j*4+4)=(round(x(j*4+3:j*4+4)));
end
[Device,coordinate]=get_coordinate(x,Opt_);
coordinate=[coordinate;0,0;Opt_.length,Opt_.high];

Opt_.A_Star.mesh_point=get_Astar_mesh(Opt_,coordinate);
obstacle=get_obstacle(Device,Opt_.A_Star.mesh_point);
[~,c]=size(Device);
routed_point={};
Port=Opt_.Port;
coordinate_Astar=real2Astar2(Opt_.A_Star.mesh_point,coordinate);
dis_path=0;
dis_device=0;
[cs,~]=con_ga2(x,Opt_);
if max(cs)>0
for i=1:c
   
    device=Device(i);
    for point=device.port_name
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
            if isempty(Optimal_path)

                begin_point_real=device.port.(point1).coordinate;
%                 begin_point_Astar=real2Astar2(Opt_.A_Star.mesh_point,begin_point_real);
%                 obstacle1=setdiff(obstacle,[end_point_Astar;begin_point_Astar],'rows');
                if contains(connect_point1,'Device')
                    dis_device=dis_device+pdist2(end_point_real,begin_point_real);
                end
                
                     dis_path=dis_path+1000;
                    
                

            else
                if contains(connect_point1,'Device')
                    dis_device=dis_device+pdist2(end_point_real,begin_point_real);
                end
%                 Optimal_path_real=Astar2real2(Opt_.A_Star.mesh_point,Optimal_path);
%                 [~,I]=pdist2(Optimal_path_real,end_point_real,'euclidean','Smallest',1);
%                 begin_point_Astar=Optimal_path(I,:);
%                 obstacle1=setdiff(obstacle,[end_point_Astar;begin_point_Astar],'rows');
                
                dis_path=dis_path+1000;
                    
              
            end


        end
    end



end
dis_path=dis_path+obj_global_rout(x,Opt_);

else
for i=1:c
   
    device=Device(i);
    for point=device.port_name
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
            if isempty(Optimal_path)

                begin_point_real=device.port.(point1).coordinate;
                begin_point_Astar=real2Astar2(Opt_.A_Star.mesh_point,begin_point_real);
                begin_point_area_Astar=get_point_area(begin_point_real,Opt_);
                end_point_area_Astar=get_point_area(end_point_real,Opt_);
                obstacle1=setdiff(obstacle,[end_point_area_Astar;begin_point_area_Astar],'rows');
                if contains(connect_point1,'Device')
                    dis_device=dis_device+pdist2(end_point_real,begin_point_real);
                end
                  
                 [Optimal_path_temp, fail_distance,NoPath]=Routing_fun(Opt_,begin_point_Astar,end_point_Astar,obstacle1,0.30);
                 if NoPath==1
                [~,Optimal_path_temp]=path_legalization2(Optimal_path_temp,Opt_);

                Optimal_path=[Optimal_path;Optimal_path_temp];
                dis_path=dis_path+calc_route_length(Optimal_path_temp,Opt_.A_Star.mesh_point);

                else
                     dis_path=dis_path+ fail_distance.closest_point_distance+1000;
                    
                end

            else
                if contains(connect_point1,'Device')
                    dis_device=dis_device+pdist2(end_point_real,begin_point_real);
                end
                Optimal_path_real=Astar2real2(Opt_.A_Star.mesh_point,Optimal_path);
                [~,I]=pdist2(Optimal_path_real,end_point_real,'euclidean','Smallest',1);
                begin_point_real=Optimal_path_real(I,:);
                begin_point_Astar=Optimal_path(I,:);
                begin_point_area_Astar=get_point_area(begin_point_real,Opt_);
                end_point_area_Astar=get_point_area(end_point_real,Opt_);
                obstacle1=setdiff(obstacle,[end_point_area_Astar;begin_point_area_Astar],'rows');
                
                 [Optimal_path_temp, fail_distance,NoPath]=Routing_fun(Opt_,begin_point_Astar,end_point_Astar,obstacle1,0.30);
                 if NoPath==1
                [~,Optimal_path_temp]=path_legalization2(Optimal_path_temp,Opt_);

                Optimal_path=[Optimal_path;Optimal_path_temp];
                dis_path=dis_path+calc_route_length(Optimal_path_temp,Opt_.A_Star.mesh_point);
                 else
                    dis_path=dis_path+ fail_distance.closest_point_distance+1000;
                    
                end
                

            end


        end
                Optimal_path_obs=[];

                if ~isempty(Optimal_path)
                    for iii=-10:10
                        Optimal_path_obs=[Optimal_path_obs;Optimal_path(:,1)+iii,Optimal_path(:,2)];
                    end
                    
                    for iii=-10:10
                        Optimal_path_obs=[Optimal_path_obs;Optimal_path(:,1),Optimal_path(:,2)+iii];
                    end
                    Optimal_path_obs(Optimal_path_obs(:,1)<1,1)=1;
                    Optimal_path_obs(Optimal_path_obs(:,1)>length(Opt_.A_Star.mesh_point{1})+1,1)=length(Opt_.A_Star.mesh_point{1})+1;

                    Optimal_path_obs(Optimal_path_obs(:,2)<1,2)=1;
                    Optimal_path_obs(Optimal_path_obs(:,2)>length(Opt_.A_Star.mesh_point{2})+1,2)=length(Opt_.A_Star.mesh_point{2})+1;
                end


    obstacle=[obstacle;unique(Optimal_path_obs,"rows")];
    end


end
end
dis=dis_path;
%toc
end