function points = linesToPointSet(vertices, width, numPointsPerSegment)
    % vertices: µã¼¯£¬¶¨ÒåÏßµÄÂ·¾¶£¬Ã¿ÐÐÊÇÒ»¸öµã [x, y]
    % width: Ïß¿í
    % numPointsPerSegment: Ã¿¶ÎÏßÉÏµÄµãÊý
    
    numVertices = size(vertices, 1);
    if numVertices < 2
        error('abcd');
    end
    
    % Ô¤·ÖÅäµã¼¯Êý×é
    leftPoints = [];
    rightPoints = [];
    
    % ´¦ÀíÃ¿Ò»¶ÎÏß¶Î
    for i = 1:numVertices-1
        % µ±Ç°Ïß¶ÎµÄÁ½¸ö¶Ëµã
        p1 = vertices(i, :);
        p2 = vertices(i+1, :);
        
        % ¼ÆËãÏß¶ÎµÄ·½ÏòÏòÁ¿
        d = p2 - p1;
        L = norm(d);
        d = d / L; % µ¥Î»·½ÏòÏòÁ¿
        
        % ¼ÆËãÓëÏß¶Î´¹Ö±µÄµ¥Î»ÏòÁ¿
        perp = [-d(2), d(1)];
        
        % Ïß¶ÎµÄÆ«ÒÆÁ¿
        offset = (width / 2) * perp;
        
        % Éú³ÉÏß¶ÎÉÏµÄµã
        t = linspace(0, L, numPointsPerSegment);
        segmentLeftPoints = zeros(numPointsPerSegment, 2);
        segmentRightPoints = zeros(numPointsPerSegment, 2);
        
        for j = 1:numPointsPerSegment
            pointOnLine = p1 + d * t(j);
            segmentLeftPoints(j, :) = pointOnLine + offset;
            segmentRightPoints(j, :) = pointOnLine - offset;
        end
        
        % ½«×óÓÒ±ß½çµãºÏ²¢µ½×Üµã¼¯ÖÐ
        leftPoints = [leftPoints; segmentLeftPoints];
        rightPoints = [rightPoints; segmentRightPoints];
    end
    
    % Á¬½Ó×óÓÒ±ß½çµÄµã¼¯
    % Á¬½ÓÏß¶Î±ß½ç
    points = [leftPoints; flipud(rightPoints)];
    
    % ¿ÉÑ¡£º¿ÉÊÓ»¯½á¹û
    figure;
    hold on;
    plot(vertices(:, 1), vertices(:, 2), 'k--o'); % »æÖÆÔ­Ê¼µã¼¯Á¬½ÓµÄÏß
    scatter(points(:, 1), points(:, 2), 'r.'); % »æÖÆÉú³ÉµÄµã¼¯
    axis equal;
    hold off;
end
