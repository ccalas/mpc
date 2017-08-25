function P = computePPred(A, C, ny)
    P_tmp = cell(ny,1);
    tmp = eye(8);
    
    for i=1:ny
        tmp = tmp*A;
        P_tmp{i} = C*tmp;
    end
    
    P = cell2mat(P_tmp);
end