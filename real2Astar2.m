function index=real2Astar2(mesh_point,real)
[a,~]=size(real);
index=zeros([a,2]);
for j=1:a
    for i=1:2
        if real(j,i)==0
            index(j,i)=1;
        else
            left = 1;
            right = length(mesh_point{i});
            while left <= right
                mid = floor((left + right)/2);
                if abs(sum(mesh_point{i}(1:mid))- real(j,i))<1e-5
                    index(j,i) = mid+1;
                    break;
                elseif sum(mesh_point{i}(1:mid))< real(j,i)
                    left = mid + 1;
                else
                    right = mid - 1;
                end
            end
        end
    end
end
end