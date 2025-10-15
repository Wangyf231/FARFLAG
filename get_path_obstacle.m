function path_astar=get_path_obstacle(path_Astar,Opt_)
directions=path_Astar(2:end,:)-path_Astar(1:end-1,:);
ind=find(directions(2:end,1)-directions(1:end-1,1)~=0&directions(2:end,2)-directions(1:end-1,2)~=0);
ind=ind+1;
temp_ind=1;
    path_astar=[];

for i=1:length(ind)-1
    path_Astar_temp=path_Astar(temp_ind:ind(i),:);
    point_edge_Astar=find_nearest_Astar_point(path_Astar(temp_ind,:),directions(temp_ind,:),Opt_);
    n=max(abs(point_edge_Astar(2,:)-point_edge_Astar(1,:)));
    path_Astar_temp1=double(path_Astar_temp.*abs(directions(temp_ind,:)))+([1,1]-double(abs(directions(temp_ind,:)))).*double(point_edge_Astar(1,:));
    path_Astar_temp2=double(path_Astar_temp.*abs(directions(temp_ind,:)))+([1,1]-double(abs(directions(temp_ind,:)))).*double(point_edge_Astar(2,:));


    t = linspace(0, 1, n+1); 
    [m,~]=size(path_Astar_temp2);
    %path_astar=[];
    for ii=1:m
    path_astar_temp = ((1-t)' .* double(path_Astar_temp2(ii,:)) + (t)' .* double(path_Astar_temp1(ii,:)));
    path_astar=[path_astar;int64(path_astar_temp)];
    %path_astar=path_astar(2:end,:);

    end




    temp_ind=ind(i);

end

end

