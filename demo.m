%clc;clear;
%% Set Process Parameters
Param.techLib = "";

% set metal layers
% input cds.lib
Param.pathCdslib = "/data/wyf/EXP/layout_auto/layout_auto_1103/cds.lib";
% constrains group name
Param.constGroup = "virtuosoDefaultSetup";
% input the path to the process file used for the EMX simulation
Param.pathProc = "";
% input the path of layermap file
Param.pathMap = "";

Param.currentModel='';
mkdir(Param.currentModel);

copyfile(Param.pathCdslib,Param.currentModel);
black=2.5;
wire_black=2.5;
Param.black=black;
Param.wire_black=wire_black;

C1=cap_generate(29,Param,Param.black);
C1.name='C1';
C1.port.output.connect={'Device(3).port.input','Device(2).port.input'};
C1.port.input.connect={'Port(1)'};

C2=cap_generate(49,Param,Param.black);
C2.name='C2';
C2.port.input.connect={'Device(1).port.output','Device(3).port.input'};
C2.port.output.connect={'Port(2)'};

%load geom_0676
%load("Param_ind.mat","Param_ind");
L1=inductor_info(geom_0676,Param_ind,black);
L1.name='L1';
L1.lib_name='"myLib_ind"';
L1.model_name='Inductor_0676';
L1.port.input.connect={'Device(1).port.output','Device(2).port.input'};
L1.port.output.connect={'Port(3)'};

Device=[C1,C2,L1];
Opt_.Device=Device;
%% Optimization edge
Opt_.length=max(C1.length,C1.high)+max(C2.length,C2.high)+max(L1.length,L1.high)+10;
Opt_.high=max(C1.length,C1.high)+max(C2.length,C2.high)+max(L1.length,L1.high)+10;
%% Input and Output port srtting
IN.coordinate=[0,Opt_.high*3/4];
IN.connect={'Device(1).port.input'};
IN.name='IN';

OUT.coordinate=[Opt_.length,Opt_.high*3/4];
OUT.connect={'Device(2).port.output'};
OUT.name='OUT';

GND.coordinate=[Opt_.length/3,0];
GND.connect={'Device(3).port.output'};
GND.name='GND';

Port=[IN,OUT,GND];
Opt_.Port=Port;
Opt_.lib_name='model';
Opt_.model_name='model_in';
Opt_.wire_length=5;
%% A* setting
A_Star.edge=[max(C1.high,C1.length)+max(C2.high,C2.length)+10,max(C1.high,C1.length)+max(C2.high,C2.length)];
A_Star.mesh=[1,1];
Opt_.A_Star=A_Star;
%% optimization setting;
poolObj = gcp('nocreate');
if isempty(poolObj) == 0
    delete(poolObj);
end
 parpool('local',16);
[Param,~,model_in]=Module_generation(Param,Opt_);



