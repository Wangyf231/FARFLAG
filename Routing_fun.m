function [Optimal_path, fail_distance,NoPath] = Routing_fun(Opt_, begin_point, end_point, obstacle,timeout)
    %% 
if nargin<5
    timeout = 1.75;
end
global isTimeout;
    isTimeout = false;
    t = timer('StartDelay', timeout, 'TimerFcn', @timeoutCallback);
    cleanupObj = onCleanup(@() cleanup(t));
    start(t);

    %% 2D
    edge = Opt_.A_Star.edge;
    mesh_point = Opt_.A_Star.mesh_point;
    Grid_X = length(mesh_point{1}) + 1;
    Grid_Y = length(mesh_point{2}) + 1;

    MAP_2D = 2 * (ones(Grid_X, Grid_Y));
    Obs = zeros(Grid_X, Grid_Y);
    obstacle = int64(obstacle);
    begin_point = int64(begin_point);
    end_point = int64(end_point);

    %% 
    xTarget = begin_point(1);
    yTarget = begin_point(2);
    MAP_2D(xTarget, yTarget) = 0;

    xStart = end_point(1);
    yStart = end_point(2);
    MAP_2D(xStart, yStart) = 1;

    %% 
    for ii = 1:size(obstacle, 1)
        MAP_2D(obstacle(ii, 1), obstacle(ii, 2)) = -1;
        Obs(obstacle(ii, 1), obstacle(ii, 2)) = 1;
    end

    %% OPENCLOSED
    OPEN_LIST = [];
    CLOSED_LIST = [];
    counter = 1;
    for m = 1:Grid_X
        for n = 1:Grid_Y
            if (MAP_2D(m, n) == -1)
                CLOSED_LIST(counter, 1) = m;
                CLOSED_LIST(counter, 2) = n;
                counter = counter + 1;
            end
        end
    end
    CLOSED_COUNT = size(CLOSED_LIST, 1);

    %% 
    xPonit = xStart;
    yPoint = yStart;
    OPEN_COUNT = 1;
    Road_cost = 0;
    goal_distance = distance(xPonit, yPoint, xTarget, yTarget, mesh_point);

    OPEN_LIST(OPEN_COUNT, :) = openlist_insert(xPonit, yPoint, xPonit, yPoint, Road_cost, goal_distance, goal_distance);
    OPEN_LIST(OPEN_COUNT, 1) = 0;
    CLOSED_COUNT = CLOSED_COUNT + 1;
    CLOSED_LIST(CLOSED_COUNT, 1) = xPonit;
    CLOSED_LIST(CLOSED_COUNT, 2) = yPoint;
    NoPath = 1;

    %% A*
    while ((xPonit ~= xTarget || yPoint ~= yTarget) && NoPath == 1)

        if isTimeout
            NoPath = 0; % 
            break;
        end

        expand_map = Expand_function(xPonit, yPoint, Road_cost, xTarget, yTarget, CLOSED_LIST, Grid_X, Grid_Y, mesh_point);
        exp_count = size(expand_map, 1);

        for m = 1:exp_count
            flag = 0;
            for n = 1:OPEN_COUNT
                if (expand_map(m, 1) == OPEN_LIST(n, 2) && expand_map(m, 2) == OPEN_LIST(n, 3))
                    OPEN_LIST(n, 8) = min(OPEN_LIST(n, 8), expand_map(m, 5));
                    if OPEN_LIST(n, 8) == expand_map(m, 5)
                        OPEN_LIST(n, 4) = xPonit;
                        OPEN_LIST(n, 5) = yPoint;
                        OPEN_LIST(n, 6) = expand_map(m, 3);
                        OPEN_LIST(n, 7) = expand_map(m, 4);
                    end
                    flag = 1;
                end
            end
            if flag == 0
                OPEN_COUNT = OPEN_COUNT + 1;
                OPEN_LIST(OPEN_COUNT, :) = openlist_insert(expand_map(m, 1), expand_map(m, 2), xPonit, yPoint, expand_map(m, 3), expand_map(m, 4), expand_map(m, 5));
            end
        end

        %% f(n)
        index_min_node = minimum_fn(OPEN_LIST, OPEN_COUNT, xTarget, yTarget);
        if (index_min_node ~= -1)
            xPonit = OPEN_LIST(index_min_node, 2);
            yPoint = OPEN_LIST(index_min_node, 3);
            Road_cost = OPEN_LIST(index_min_node, 6);

            CLOSED_COUNT = CLOSED_COUNT + 1;
            CLOSED_LIST(CLOSED_COUNT, 1) = xPonit;
            CLOSED_LIST(CLOSED_COUNT, 2) = yPoint;
            OPEN_LIST(index_min_node, 1) = 0;
        else
            NoPath = 0;  % 
        end
    end

    %% 
    if NoPath == 0
        fail_distance.closest_point_distance = min(calc_distance(CLOSED_LIST, [xTarget, yTarget], mesh_point));
        fail_distance.estimated_shortest_distance = distance(xStart, yStart, xTarget, yTarget, mesh_point);
        fail_distance.actual_distance = sum(sqrt(diff(CLOSED_LIST(:, 1)).^2 + diff(CLOSED_LIST(:, 2)).^2));
        fail_distance.distance_gap = fail_distance.actual_distance - fail_distance.estimated_shortest_distance;
        Optimal_path = [];


    %% 
    else
       fail_distance = [];

        m = size(CLOSED_LIST, 1);
        Optimal_path = [];
        point_x = CLOSED_LIST(m, 1);
        point_y = CLOSED_LIST(m, 2);
        m = 1;
        Optimal_path(m, 1) = point_x;
        Optimal_path(m, 2) = point_y;
        m = m + 1;

            inode = 0;
            parent_x = OPEN_LIST(return_node_index(OPEN_LIST, point_x, point_y), 4);
            parent_y = OPEN_LIST(return_node_index(OPEN_LIST, point_x, point_y), 5);

            while (parent_x ~= xStart || parent_y ~= yStart)
                Optimal_path(m, 1) = parent_x;
                Optimal_path(m, 2) = parent_y;
                inode = return_node_index(OPEN_LIST, parent_x, parent_y);
                parent_x = OPEN_LIST(inode, 4);
                parent_y = OPEN_LIST(inode, 5);
                m = m + 1;
            end
            Optimal_path = [Optimal_path; xStart, yStart];

    
    end

    clear global isTimeout;
end

function timeoutCallback(~, ~)
    global isTimeout;
    isTimeout = true;
end

function cleanup(t)
    stop(t);
    delete(t);
end
