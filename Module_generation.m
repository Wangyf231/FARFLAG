function [Param,Opt_,device,varargout]=Module_generation(Param,Opt_)

% varargout:solution(Optional parameters)
device_num=length(Opt_.Device);


lb1=zeros([1,device_num*4]);
ub1=repmat([Opt_.length,Opt_.high,3,1],1,device_num);
a=[3,4];
b=(0:(device_num-1))*4;
c=repmat(b,2,1)+a';
n_car=reshape(c,1,[]);
confun = @(x)con_ga2(x,Opt_);
obj_=@(x)obj_ga3(x,Opt_);
obj_2=@(x)obj_ga_only_rout(x,Opt_);

% "UseParallel",true,
for i=1:10000
    if mod((i-1),5)==0
        obj_1=@(x)obj_global_rout(x,Opt_);
        options = optimoptions("ga","UseParallel",true,"ConstraintTolerance",1e-10,"PlotFcn",["gaplotdistance",...
            "gaplotbestf"]);
        [solution,~,~] = ga(obj_1,4*device_num,[],[],[],[],lb1,ub1,[],n_car,...
            options);
        options = optimoptions("ga","ConstraintTolerance",1e-10,"UseParallel",true,"PlotFcn",["gaplotdistance",...
         "gaplotbestf"],"MaxGenerations",100,"MaxStallGenerations",30,"FunctionTolerance",0.01,"InitialPopulationMatrix",solution);
        lb=0.7*solution;
        ub=1.3*solution;
        ub(ub>Opt_.high)=Opt_.high;
        lb((0:device_num-1).*4+3)=0;
        lb((0:device_num-1).*4+4)=0;
        ub((0:device_num-1).*4+3)=3;
        ub((0:device_num-1).*4+4)=1;

        lb((0:device_num-1).*4+2)=min(lb((0:device_num-1).*4+2));
        lb((0:device_num-1).*4+1)=min(lb((0:device_num-1).*4+1));
        ub((0:device_num-1).*4+2)=max(ub((0:device_num-1).*4+2));
        ub((0:device_num-1).*4+1)=max(ub((0:device_num-1).*4+1));

    else
options = optimoptions("ga","UseParallel",true,"ConstraintTolerance",1e-10,"PlotFcn",["gaplotdistance",...
    "gaplotbestf"],"MaxGenerations",100,"InitialPopulationMatrix",solution,"MaxStallGenerations",30,"FunctionTolerance",0.01);

    end
%tt=tic;

[solution,obj,~,~,populations] = ga(obj_2,4*device_num,[],[],[],[],lb,ub,confun,n_car,...
    options);
aaa=strcat("inter ",num2str(i)," find best objvalue is ",num2str(obj));
disp(aaa)
try 
    [~,~,~,~]=Path_generate2(solution,Opt_);
    disp(aaa)

    disp("break")
    disp(solution)
    break
catch
    disp(aaa)

end
end
disp(obj_(solution))
options = optimoptions("ga","UseParallel",true,"ConstraintTolerance",1e-10,"PlotFcn",["gaplotdistance",...
    "gaplotbestf"],"MaxGenerations",500,"InitialPopulationMatrix",populations);
[solution,objvalue,out,~,populationss] = ga(obj_,4*device_num,[],[],[],[],lb,ub,confun,n_car,...
    options);
disp(objvalue)
mkdir(strcat(Param.currentModel,'/',Opt_.lib_name,'/',Opt_.model_name));
save(strcat(Param.currentModel,'/',Opt_.lib_name,'/',Opt_.model_name,"/solution.mat"),"solution");

if nargout>3
varargout{1}=solution;
end


tic
[Optimal_path,pathtotal,path_combine_index,Opt_]=Path_generate2(solution,Opt_);
toc


for j=0:device_num-1
solution(j*4+1:j*4+2)=round(solution(j*4+1:j*4+2),3);
solution(j*4+3:j*4+4)=(round(solution(j*4+3:j*4+4)));
end
for j=0:device_num-1
solution(j*4+1:j*4+2)=round(solution(j*4+1:j*4+2),3)-Opt_.new_edge(1,:);
end
[Device,coordinate]=get_coordinate(solution,Opt_);
for j=1:length(pathtotal)
pathtotal{j}=pathtotal{j}-Opt_.new_edge(1,:);
end
for j=1:length(Opt_.Port)
Opt_.Port(j).coordinate=Opt_.Port(j).coordinate-Opt_.new_edge(1,:);
end
Opt_.Device=Device;
Opt_.new_edge=Opt_.new_edge-Opt_.new_edge(1,:);


device.lib_name=strcat('"',Opt_.lib_name,'"');
device.model_name=Opt_.model_name;
device.param=struct();
device.edge=Opt_.new_edge;
device.edge_true=Opt_.new_edge;
device.length=device.edge(3,1)-device.edge(1,1);
device.high=device.edge(3,2)-device.edge(1,2);
%device.port=Opt_.Port;
port_name={};
for j=1:length(Opt_.Port)
    name=Opt_.Port(j).name;
    device.port.(name).coordinate_true=Opt_.Port(j).coordinate;
    device.port.(name).coordinate=Opt_.Port(j).coordinate;
    device.port.(name).layer='MTT2';
    device.para_name={};
    port_name{end+1}=name;
end
device.port_name=port_name;
%% call for virtuoso
num = randi([0,99]);tenBase = floor(num/10);base = mod(num,10);
Param=model2skill(Param,Opt_,pathtotal,path_combine_index);
for i=1:5
    system(strcat("cd ",Param.currentModel,";timeout -k 1s 5m virtuoso -64 -nocdsinit -log cdslog",num2str(1),".log -replay ",Param.pathScript," &>/dev/null"));

    if exist(strcat(Param.currentModel,'/',Opt_.lib_name,'/',Opt_.model_name,'/layout/layout.oa'),"file")
       fprintf("Model generation succeed!\n\n");
        break
    else
       fprintf("Model generation filed,try again!\n\n");

    end
end


end