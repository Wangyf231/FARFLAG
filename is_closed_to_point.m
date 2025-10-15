function [flag,index]=is_closed_to_point(point_list,path_real)
if isempty(point_list)
    flag=false;
    index=0;
    return
end
[D,I]=pdist2(path_real([1,end],:),point_list,"euclidean","Smallest",1);
if D<2.5001
    flag=true;
    if I==1
        index=1;
    elseif I==2
        [index,~]=size(path_real);
    else
        index=0;
    end
else

    flag=false;
    index=0;
end



end