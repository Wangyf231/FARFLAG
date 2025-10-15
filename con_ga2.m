function [c,ceq]=con_ga2(x,Opt_)%

device_num=length(x)/4;
for j=0:device_num-1
x(j*4+1:j*4+2)=round(x(j*4+1:j*4+2),3);
x(j*4+3:j*4+4)=round(x(j*4+3:j*4+4));
end
Opt_.Port=struct;
[Device,coordinate]=get_coordinate(x,Opt_);
con_max=max(coordinate)-[Opt_.length,Opt_.high];
con_min=-min(coordinate);
carea=0;
for i=1:length(Device)
    for j=i+1:length(Device)
        carea1=calc_overlap_area(Device(i),Device(j));
        carea=carea1+carea;
    end
end
c=[con_max con_min+1e-3-1e-10 carea];

ceq=[];
end