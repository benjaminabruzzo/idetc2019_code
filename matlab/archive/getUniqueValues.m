function out = getUniqueValues(vector)
i = 1;
    while ~isempty(vector)
        % use first value as first unique value
        out(i,1) = vector(1);
        out(i,2) = length(vector(vector==vector(1)));
        
        % remove all elements with this value
        vector(vector == vector(1)) = [];
        i = i+1;
    end
end