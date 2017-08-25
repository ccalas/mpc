function Px = computePxPred(A, nu)
    Px_tmp = cell(nu,1);
    tmp = eye(6);
    
    for i=1:nu
        tmp = tmp*A;
        Px_tmp{i} = tmp;
    end
    
    Px = cell2mat(Px_tmp);
end