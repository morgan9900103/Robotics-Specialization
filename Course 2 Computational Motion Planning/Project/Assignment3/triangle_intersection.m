function flag = triangle_intersection(P1, P2)
% triangle_test : returns true if the triangles overlap and false otherwise

%%% All of your code should be between the two lines of stars.
% *******************************************************************

% d = (x - x1)*(y2 - y1) - (y - y1)*(x2 - x1);
% d = (x - P1(1,1))*(P1(2,2) - P1(1,2)) - (y - P1(1,2))*(P1(2,1) - P1(1,1));

flags = [1,1,1];

for i=1:3
    det1 = (P1(mod(i,3)+1,1)   - P1(mod(i+1,3)+1, 1))*(P1(mod(i+2,3)+1, 2) - P1(mod(i+1,3)+1, 2)) - (P1(mod(i,3)+1,2)   - P1(mod(i+1,3)+1, 2))*(P1(mod(i+2,3)+1, 1) - P1(mod(i+1,3)+1, 1));
    det2 = (P2(mod(i,3)+1,1)   - P1(mod(i+1,3)+1, 1))*(P1(mod(i+2,3)+1, 2) - P1(mod(i+1,3)+1, 2)) - (P2(mod(i,3)+1,2)   - P1(mod(i+1,3)+1, 2))*(P1(mod(i+2,3)+1, 1) - P1(mod(i+1,3)+1, 1));
    det3 = (P2(mod(i+1,3)+1,1) - P1(mod(i+1,3)+1, 1))*(P1(mod(i+2,3)+1, 2) - P1(mod(i+1,3)+1, 2)) - (P2(mod(i+1,3)+1,2) - P1(mod(i+1,3)+1, 2))*(P1(mod(i+2,3)+1, 1) - P1(mod(i+1,3)+1, 1));
    det4 = (P2(mod(i+2,3)+1,1) - P1(mod(i+1,3)+1, 1))*(P1(mod(i+2,3)+1, 2) - P1(mod(i+1,3)+1, 2)) - (P2(mod(i+2,3)+1,2) - P1(mod(i+1,3)+1, 2))*(P1(mod(i+2,3)+1, 1) - P1(mod(i+1,3)+1, 1));
    
    if ((det1<0 && det2>0 && det3>0 && det4>0) || (det1>0 && det2<0 && det3<0 && det4<0))
        flags(i) = 0;
    end
end

if (flags(1) == 0|| flags(2) == 0 || flags(3) == 0)
    flag = false;
else
    flag = true;
end

% *******************************************************************
end