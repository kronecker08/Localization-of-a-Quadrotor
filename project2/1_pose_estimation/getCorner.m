function res = getCorner(id)
%% CHANGE THE NAME OF THE FUNCTION TO getCorner
    %% Input Parameter Description
    % id = List of all the AprilTag ids detected in the current image(data)
    Corner_str = load('Graph.mat');
    Corner_array = Corner_str.('Graph');
    id_for_corner  = id + 1;
    n = length(id);
    res = zeros(n,8);
    for i =  1:n
        id_curr = id_for_corner(i);
        values = Corner_array(id_curr,2:9);
        res(i,:) = values;
    end
end