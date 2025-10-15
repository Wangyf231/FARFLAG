function [X,Y,K] = modify_coord(X,Y)
X = [X X(:,1)];
Y = [Y Y(:,1)];

for jj = 1:size(X,1)
    for ii = 1:size(X,2)-1
        if X(jj,ii+1)~=X(jj,ii) && Y(jj,ii+1)~=Y(jj,ii)
            % fine tuning of coordinates based on slope
            k = (Y(jj,ii+1)-Y(jj,ii))/(X(jj,ii+1)-X(jj,ii));
            % if the line is not absolutely vertical
            if abs(k) > 10
                X(jj,ii+1) = X(jj,ii);
            % if the line is not absolutely horizontal
            elseif abs(k)<0.1
                Y(jj,ii+1) = Y(jj,ii);
            % if the line is not absolutely 45 degrees
            elseif k>0.9 && k<1
                Y(jj,ii+1) = Y(jj,ii)+(X(jj,ii+1)-X(jj,ii));
            elseif k<1.1 && k>1
                X(jj,ii+1) = X(jj,ii)+(Y(jj,ii+1)-Y(jj,ii));
            elseif k<-0.9 && k>-1
                Y(jj,ii+1) = Y(jj,ii)+(X(jj,ii)-X(jj,ii+1));
            elseif k>-1.1 && k<-1
                X(jj,ii+1) = X(jj,ii)+(Y(jj,ii)-Y(jj,ii+1));
            end
        end
        K(jj,ii) = (Y(jj,ii+1)-Y(jj,ii))/(X(jj,ii+1)-X(jj,ii));
    end
end

X = X(:,1:end-1);
Y = Y(:,1:end-1);
end