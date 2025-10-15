function dis=obj_global_rout(x,Opt_)

device_num=length(x)/4;
% for j=0:device_num-1
% x(j*4+1:j*4+2)=round(x(j*4+1:j*4+2),3);
% x(j*4+3:j*4+4)=(round(x(j*4+3:j*4+4)));
% end
[Device,~]=get_coordinate(x,Opt_);

routed_point={};
Port=Opt_.Port;
dis_path=0;
dis_device=0;
[cs,~]=con_ga2(x,Opt_);
if(max(cs)>0)
dis=exp(max(cs+7));

else
for i=1:device_num
    device=Device(i);
    for point=device.port_name
        point1=cell2mat(point);
        %
        checkpoint=strcat('Device(',num2str(i),').port.',point1);
        begin_point_real=device.port.(point1).coordinate;

        if ~isempty(find(strcmp(routed_point,checkpoint), 1))
            continue
        end
        routed_point{end+1}=checkpoint;
        [~,b]=size(device.port.(point1).connect);
        for j=1:b
            connect_point1=cell2mat(device.port.(point1).connect(j));
            end_point_real=eval(connect_point1).coordinate;

            dis_path=dis_path+abs(end_point_real(1)-begin_point_real(1))+abs(end_point_real(2)-begin_point_real(2));
            if contains(connect_point1,'Device')
                    dis_device=dis_device+pdist2(end_point_real,begin_point_real);
            end
            begin_point_real=end_point_real;
        end

    end

end
dis=dis_path+dis_device;
end
end