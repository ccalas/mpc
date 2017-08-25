function L = computeLPred(ny)
    L_tmp = cell(ny,1);
    
    for i=1:ny
       L_tmp{i,1} = eye(3); 
    end
    
    L = cell2mat(L_tmp);
end