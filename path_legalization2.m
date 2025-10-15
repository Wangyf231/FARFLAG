function [path_real,path_astar]=path_legalization2(pathbefore,Opt_)
%%Avoid diagonal routing
pathbefore=unique(pathbefore,'rows','stable');
[len,~]=size(pathbefore);
if len==1
path_astar=[pathbefore;pathbefore];
path_real=Astar2real2(Opt_.A_Star.mesh_point,path_astar);
else
grad=double(pathbefore(2:end,2)-pathbefore(1:end-1,2))./double(pathbefore(2:end,1)-pathbefore(1:end-1,1));
% grad=[grad(1);grad];
[aaa,~]=find(grad~=0 & abs(grad)~=inf);
if ~isempty(aaa)
    lena=length(aaa);
    path_1=[pathbefore(1:aaa(1),:);pathbefore(aaa(1),1),pathbefore(aaa(1)+1,2)];
    for i=2:lena
    path_1=[path_1;pathbefore(aaa(i-1)+1:aaa(i),:);pathbefore(aaa(i),1),pathbefore(aaa(i)+1,2)];
    end
    path_1=[path_1;pathbefore(aaa(lena)+1:end,:)];
else
    path_1=pathbefore;
end
%%path legalization
len=length(path_1);
grad=double((path_1(2:end,2)-path_1(1:end-1,2)))./double((path_1(2:end,1)-path_1(1:end-1,1)));
grad=[grad(1);grad];
grad(abs(grad)>1e6)=inf;%
directionss=(path_1(2:end,:)-path_1(1:end-1,:));
i=1;
temp_grad=grad(1);
path_1_real=Astar2real2(Opt_.A_Star.mesh_point,path_1);
path_real=path_1_real(1,:);
path_astar=path_1(1,:);
while i<=len
    [diff_grad,~]=find(grad(i:end,:)~=temp_grad);
    if isempty(diff_grad)
        path_real(end+1,:)=path_1_real(end,:);
        path_astar=[path_astar;gen_path(path_real(end-1,:),path_real(end,:),Opt_)];
        i=len+1;
    else
        path_real(end+1,:)=path_1_real(diff_grad(1)-1+i-1,:); 
        path_astar=[path_astar;gen_path(path_real(end-1,:),path_real(end,:),Opt_)];

        direction=path_1(diff_grad(1)+i-1,:)-path_1(diff_grad(1)-1+i-1,:);
        [out__edge_point,~]=find(sum((path_1_real(diff_grad(1)+i-1:end,:)-path_1_real(diff_grad(1)+i-1-1,:)).*double(direction),2)>=3);
        if isempty(out__edge_point)
            [Reverse_point,~]=find(directionss(diff_grad(1)+i-1:end,1)==-direction(1)&directionss(diff_grad(1)+i-1:end,2)==-direction(2));%need change 0902
            if ~isempty(Reverse_point)
                if ~ismember(double(direction).*path_1_real(diff_grad(1)-1+i-1,:)+([1,1]-double(abs(direction))).*path_1_real(i+diff_grad(1)-1+Reverse_point(1)-1,:),path_real,"rows")
                path_real(end+1,:)=double(abs(direction)).*path_1_real(diff_grad(1)-1+i-1,:)+([1,1]-double(abs(direction))).*path_1_real(i+diff_grad(1)-1+Reverse_point(1)-1,:);
                path_astar=[path_astar;gen_path(path_real(end-1,:),path_real(end,:),Opt_)];

                end
                [Reverse_point_after_flag,Reverse_point_after]=ismembertol(path_real(end,:),path_1_real(diff_grad(1)-1+Reverse_point(1)+i-1:end,:),'ByRows',true);
                if Reverse_point_after_flag
                    i=i+diff_grad(1)-1+Reverse_point(1)-1+Reverse_point_after-1;
                else
                i=i+diff_grad(1)-1+Reverse_point(1)-1;
                end
                temp_grad=grad(i);

            else
                if ~ismember((path_1_real(end,:)-path_1_real(diff_grad(1)-1+i-1,:)).*([1,1]-abs(double(direction)))+path_1_real(diff_grad(1)+i-1-1,:),path_real,"rows")

                  path_real(end+1,:)=(path_1_real(end,:)-path_1_real(diff_grad(1)+i-1-1,:)).*([1,1]-abs(double(direction)))+path_1_real(diff_grad(1)+i-1-1,:);
                  path_astar=[path_astar;gen_path(path_real(end-1,:),path_real(end,:),Opt_)];
                end
                i=len+1;
            end

        else
        if ~ismember([path_real(end,1),path_1_real(out__edge_point(1)-1+diff_grad(1)+i-1,2)],path_real,"rows")

            path_real(end+1,:)=[path_real(end,1),path_1_real(out__edge_point(1)-1+diff_grad(1)+i-1,2)];%%???
            path_astar=[path_astar;gen_path(path_real(end-1,:),path_real(end,:),Opt_)];
        end
        i=out__edge_point(1)-1+diff_grad(1)+i-1;
        if ~ismember(path_1_real(i,:),path_real,"rows")

            path_real(end+1,:)=path_1_real(i,:);
            path_astar=[path_astar;gen_path(path_real(end-1,:),path_real(end,:),Opt_)];
        end

        temp_grad=grad(i);
        end
    end
end

%% 冗余修正
[uniqueElements,~,indexInUnique]=unique(path_astar,"rows");
abc= find(ismember(indexInUnique, find(histc(indexInUnique, 1:numel(uniqueElements)) > 1)));
amm=[];
for mm=1:length(abc)-1
    if path_astar(abc(mm),:)==path_astar(abc(mm+1),:)
        amm=[amm,abc(mm):abc(mm+1)-1];
    end
end
path_astar(amm,:)=[];

path_real_grad=abs((path_real(2:end,2)-path_real(1:end-1,2))./(path_real(2:end,1)-path_real(1:end-1,1)));
index=[true;path_real_grad(2:end)~=path_real_grad(1:end-1);true];
path_real=path_real(index,:);
end

end


function path_astar=gen_path(begin_point,end_point,Opt_)
begin_point=real2Astar2(Opt_.A_Star.mesh_point,begin_point);
end_point=real2Astar2(Opt_.A_Star.mesh_point,end_point);

n=max(abs(begin_point-end_point));
t = linspace(0, 1, n+1); 
path_astar = ((1-t)' .* begin_point + (t)' .* end_point);
path_astar=int64(path_astar);
path_astar=path_astar(2:end,:);
end