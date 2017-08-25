function L = computeL(nu)
    L_tmp = cell(nu,1);
    
    for i=1:nu
       L_tmp{i,1} = eye(2); 
    end
    
    L = cell2mat(L_tmp);
end