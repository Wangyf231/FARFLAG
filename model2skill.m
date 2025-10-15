function Param=model2skill(Param,Opt_,pathtotal,path_combine_index,pathStyle,mos_flag)
if nargin<6
mos_flag=false;% do not draw mos
end

if nargin<5
    pathStyle='';
end
if ~isempty(pathStyle)
    pathStyle=strcat('"',pathStyle,'"');
end
roy_dic={'R0','R90','R180','R270';'MY','MYR90','MX','MXR90'}';
mkdir(Param.currentModel); % create model folder
%copyfile(Param.pathCdslib,Param.currentModel); % copy cds.lib file to current model foder
Param.pathScript = strcat(Param.currentModel,'/CreateModel_',Opt_.model_name,'.txt');%create script file
fid = fopen(Param.pathScript,'wt');
fprintf(fid,strcat('LibID=dbCreateLib("',Opt_.lib_name,'" "',Param.currentModel,'")\n'));%create Lib
fprintf(fid,strcat('techSetTechLibName(LibID "',Param.techLib,'")\n'));%attach to TechLib
fprintf(fid,strcat('CV=dbOpenCellViewByType("',Opt_.lib_name,'" "',Opt_.model_name,'" "layout" "maskLayout" "w")\n'));%create cell
fprintf(fid,'TechID=techGetTechFile(CV)\n');%via options
fprintf(fid,'constraintGroupId = cstFindConstraintGroupIn(TechID "virtuosoDefaultSetup")\n');%via options
fprintf(fid,'viaOptions = viaGetViaOptions(constraintGroupId)\n');%via options

%% creat devices
port_point={};
for i=1:length(Opt_.Device)
    if (contains(Opt_.Device(i).model_name,'n11ll_ckt_rf') || contains(Opt_.Device(i).model_name,'cascode')) && ~mos_flag
         for point=Opt_.Device(i).port_name
            point1=cell2mat(point);
            fprintf(fid,strcat('dbCreateLabel(CV',' "',Opt_.Device(i).port.(point1).layer,'"'," ",num2str(Opt_.Device(i).port.(point1).coordinate(1)),":",num2str(Opt_.Device(i).port.(point1).coordinate(2)), " ",'"',Opt_.Device(i).name,'_',point1,'" "centerCenter" "R0" "roman" 1)\n'));
         end
        

    else
        fprintf(fid,strcat('deviceCV=dbOpenCellViewByType(',Opt_.Device(i).lib_name," ",'  "', Opt_.Device(i).model_name,'" "layout")\n'));%load device
        fprintf(fid,strcat('InstCv=dbCreateExtParamInst(CV deviceCV nil list(',num2str(Opt_.Device(i).offset(1)) ," ",num2str(Opt_.Device(i).offset(2)),') "',roy_dic{Opt_.Device(i).rot_sym(1)+1,Opt_.Device(i).rot_sym(2)+1},'" 1 )\n'));%create cell

        if ~isempty(Opt_.Device(i).para_name)
            fprintf(fid,'cdfgData=cdfGetInstCDF(InstCv) \n');%create cell
            fprintf(fid,'PasCdfFormInit(cdfgData) \n');%
            for j=1:length(Opt_.Device(i).para_name)
                switch Opt_.Device(i).para_type{j}
                    case 'string'
                        fprintf(fid,strcat('InstCv~>',Opt_.Device(i).para_name{j},'="',num2str(Opt_.Device(i).param.(Opt_.Device(i).para_name{j})),Opt_.Device(i).para_unit{j},'"\n'));
                    case 'flonum'
                        fprintf(fid,strcat('InstCv~>',Opt_.Device(i).para_name{j},'=',num2str(Opt_.Device(i).param.(Opt_.Device(i).para_name{j})),Opt_.Device(i).para_unit{j},'\n'));
                end
                fprintf(fid,strcat('param=cdfFindParamByName(cdfgData "', Opt_.Device(i).para_name{j},'")\n'));%load device
                fprintf(fid,'when( param~>callback evalstring(param~>callback))\n');%load device
            end

        end
    end
    for point=Opt_.Device(i).port_name
        point1=cell2mat(point);
        port_layer='"MTT2"';

        if ~isempty(Opt_.Device(i).port.(point1).connect)
            if isfield(eval(strcat('Opt_.',Opt_.Device(i).port.(point1).connect{1})),"name")
            if length(Opt_.Device(i).port.(point1).connect)<2 && strcmp(eval(strcat('Opt_.',Opt_.Device(i).port.(point1).connect{1},'.name')),'GND')
                port_layer='"ALPA"';
            end
            end
        end
        if contains(Opt_.Device(i).model_name,'n11ll_ckt_rf') && contains(point,"G")
            %point1=cell2mat(point);
            point_middle=(Opt_.Device(i).port.(point1).coordinate+Opt_.Device(i).port.(point1).coordinate_true)/2;
            pointtem1=strcat(' (',num2str(Opt_.Device(i).port.(point1).coordinate(1))," ",num2str(Opt_.Device(i).port.(point1).coordinate(2)),') ');
            pointtem2=strcat(' (',num2str(Opt_.Device(i).port.(point1).coordinate_true(1))," ",num2str(Opt_.Device(i).port.(point1).coordinate_true(2)),') ');
            pointtem3=strcat(' (',num2str(point_middle(1))," ",num2str(point_middle(2)),') ');
            port_point{end+1}=pointtem1;

            pointtem=strcat(pointtem1,pointtem2);
            pointtem1=strcat(pointtem3,pointtem1);%for MTT2 drawing
            fprintf(fid,strcat('dbCreatePath(CV list(',port_layer,' "drawing") ''(',pointtem1,' )'," ", num2str(Opt_.wire_length)," ",' )\n' ));
            fprintf(fid,strcat('dbCreatePath(CV list("',Opt_.Device(i).port.(point1).layer,'" "drawing") ''(',pointtem,' )'," ", num2str(Opt_.wire_length)," ",' )\n' ));
        else
            %point1=cell2mat(point);
            pointtem1=strcat(' (',num2str(Opt_.Device(i).port.(point1).coordinate(1))," ",num2str(Opt_.Device(i).port.(point1).coordinate(2)),') ');
            pointtem2=strcat(' (',num2str(Opt_.Device(i).port.(point1).coordinate_true(1))," ",num2str(Opt_.Device(i).port.(point1).coordinate_true(2)),') ');
            pointtem=strcat(pointtem1,pointtem2);
            port_point{end+1}=pointtem1;
            fprintf(fid,strcat('dbCreatePath(CV list(',port_layer,' "drawing") ''(',pointtem,' )'," ", num2str(Opt_.wire_length)," ",' )\n' ));
            fprintf(fid,strcat('dbCreatePath(CV list("',Opt_.Device(i).port.(point1).layer,'" "drawing") ''(',pointtem,' )'," ", num2str(Opt_.wire_length)," ",' )\n' ));
        end

    end

end

for ii=1:length(port_point)
    fprintf(fid,strcat('viaGenerateViasAtPoint(CV ''',port_point{ii},' viaOptions)\n' ));

end
%% create gnd(if exist)
if isfile(strcat(Param.currentModel,'/out_gnd.gds'))
   fprintf(fid,strcat('deviceCV=dbOpenCellViewByType("model"   "gnd" "layout")\n'));%load device
   fprintf(fid,strcat('InstCv=dbCreateExtParamInst(CV deviceCV nil list(0 0) "R0" 1 )\n'));%create cell

end
%% create path
for i=1:length(pathtotal)
pointtem='';
[r,~]=size(pathtotal{i});
    for j=1:r
        pointtem=strcat(pointtem,' (',num2str(pathtotal{i}(j,1)),32,num2str(pathtotal{i}(j,2)),') ');
    end

    
point_list=[];
for m=1:length(Opt_.Port)
if strcmp(Opt_.Port(m).name,'GND')
    point_list(end+1,:)=Opt_.Port(m).coordinate;

end
end
[flag,index]=is_closed_to_point(point_list,pathtotal{i});
if flag
    path_layer='"ALPA"';
else
    path_layer='"MTT2"';
end 
fprintf(fid,strcat('line',num2str(i),'=dbCreatePath(CV list(',path_layer,' "drawing") ''(',pointtem,' ) 5'," " ,pathStyle,')\n' ));

if flag
    [le,~]=size(pathtotal{i});
    fprintf(fid,strcat('viaGenerateViasAtPoint(CV ''',' (',num2str(pathtotal{i}(le-index+1,1)),32,num2str(pathtotal{i}(le-index+1,2)),') ',' viaOptions)\n' ));

end
end

for i=1:length(path_combine_index)
    path_combine_index_temp=path_combine_index{i};
    if length(path_combine_index_temp)>1
        line_index_string='';
        for j=1:length(path_combine_index_temp)
            line_index_string=strcat(line_index_string," ","line",num2str(path_combine_index_temp(j)));
        end
        fprintf(fid,strcat('leMergeShapes(list(',line_index_string,'))\n' ));

    end
end

%% create lable
for i=1:length(Opt_.Port)
            point1=Opt_.Port(i).name;
            if strcmp(point1,'GND')
            fprintf(fid,strcat('dbCreateLabel(CV',' "ALPA"'," ",num2str(Opt_.Port(i).coordinate(1)),":",num2str(Opt_.Port(i).coordinate(2)), " ",'"',Opt_.model_name,'_',point1,'" "centerCenter" "R0" "roman" 1)\n'));

            else
            fprintf(fid,strcat('dbCreateLabel(CV',' "MTT2"'," ",num2str(Opt_.Port(i).coordinate(1)),":",num2str(Opt_.Port(i).coordinate(2)), " ",'"',Opt_.model_name,'_',point1,'" "centerCenter" "R0" "roman" 1)\n'));

            end
end
fprintf(fid,'dbSave(CV) \n');

fprintf(fid,strcat('xstSetField("strmFile" "',Param.currentModel,'/out.gds")\n'));
fprintf(fid,strcat('xstSetField("layerMap" "',Param.pathMap,'")\n'));
fprintf(fid,strcat('xstSetField("library" "',Opt_.lib_name,'")\n'));
fprintf(fid,'xstOutDoTranslate()\n');

fprintf(fid,'exit \n');

end