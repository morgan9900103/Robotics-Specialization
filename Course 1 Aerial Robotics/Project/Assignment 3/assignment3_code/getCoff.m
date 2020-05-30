function [ coff, A, b ] = getCoff(waypoints)
    n = size(waypoints, 1) - 1;
    A = zeros(8*n, 8*n);
    b = zeros(1, 8*n);
    
    %% Fill in b matrices with values
    for i = 1:n
        b(1,i) = waypoints(i);
        b(1,i+n) = waypoints(i+1);
    end
    
    %% Fill in A matrices with values
    % P1(0) = W1; P2(0) = W2; P3(0) = W3; P4(0) = W4 - 4 equations
    row = 1;
    for i = 1:n
        A(row,8*(i-1)+1:8*i) = polyT(8,0,0);
        row = row + 1;
    end
    
    % P1(1) = W2; P2(1) = W3; P3(1) = W4; P4(1) = W5 - 4 equations
    for i = 1:n
        A(row,8*(i-1)+1:8*i) = polyT(8,0,1);
        row = row + 1;
    end
    
    % P1_dot(0) = 0; P1_ddot(0) = 0; P1_dddot(0) = 0 - 3 equations
    for i = 1:3
        A(row,1:8) = polyT(8,i,0);
        row = row + 1;
    end
    
    % P4_dot(1) = 0; P4_ddot(1) = 0; P4_dddot(1) = 0 - 3 equations
    for i = 1:3
        A(row,8*(n-1)+1:8*n) = polyT(8,i,1);
        row = row + 1;
    end
    
    % P1_dot(1) = P2_dot(0); P2_dot(1) = P3_dot(0); P3_dot(1) = P4_dot(0);
    % - 3 equations
    for i = 1:n-1
        A(row,8*(i-1)+1:8*i)   = polyT(8,1,1);
        A(row,8*(i)+1:8*(i+1)) = (-1) * polyT(8,1,0);
        row = row + 1;
    end
    
    % P1_ddot(1) = P2_ddot(0); P2_ddot(1) = P3_ddot(0); P3_ddot(1) =
    % P4_ddot(0) - 3 equations
    for i = 1:n-1
        A(row,8*(i-1)+1:8*i)   = polyT(8,2,1);
        A(row,8*(i)+1:8*(i+1)) = (-1) * polyT(8,2,0);
        row = row + 1;
    end
    
    % P1_3dot(1) = P2_3dot(0); P2_3dot(1) = P3_3dot(0); P3_3dot(1) =
    % P4_3dot(0); - 3 equations
    for i = 1:n-1
        A(row,8*(i-1)+1:8*i)   = polyT(8,3,1);
        A(row,8*(i)+1:8*(i+1)) = (-1) * polyT(8,3,0);
        row = row + 1;
    end
    
    % P1_4dot(1) = P2_4dot(0); P2_4dot(1) = P3_4dot(0); P3_4dot(1) =
    % P4_4dot(0); - 3 equations
    for i = 1:n-1
        A(row,8*(i-1)+1:8*i)   = polyT(8,4,1);
        A(row,8*(i)+1:8*(i+1)) = (-1) * polyT(8,4,0);
        row = row + 1;
    end
    
    % P1_5dot(1) = P2_5dot(0); P2_5dot(1) = P3_5dot(0); P3_5dot(1) =
    % P4_5dot(0); - 3 equations
    for i = 1:n-1
        A(row,8*(i-1)+1:8*i)   = polyT(8,5,1);
        A(row,8*(i)+1:8*(i+1)) = (-1) * polyT(8,5,0);
        row = row + 1;
    end
    
    % P1_5dot(1) = P2_5dot(0); P2_5dot(1) = P3_5dot(0); P3_5dot(1) =
    % P4_5dot(0); - 3 equations
    for i = 1:n-1
        A(row,8*(i-1)+1:8*i)   = polyT(8,6,1);
        A(row,8*(i)+1:8*(i+1)) = (-1) * polyT(8,6,0);
        row = row + 1;
    end
    
    coff = A\b';
end

