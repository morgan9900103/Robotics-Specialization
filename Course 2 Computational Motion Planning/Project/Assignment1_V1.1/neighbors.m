function [A] = neighbors(row, col, N)
    A = [];
    [r, c] = size(N);
    index = sub2ind(size(N), row, col);
    
    % Neighbor on the left
    if (index - r >= 1)
        A = [A,index - r];
    end
    
    % Neighbor on the top
    if (index - 1 > r*(col - 1))
        A = [A, index - 1];
    end
    
    % Neighbor on the bottom
    if (index + 1 <= r * col)
        A = [A,index + 1];
    end
    
    % Neighbor on the right
    if (index + r <= (r * c))
        A = [A,index + r];
    end
end

