function E = computeE(nu)
    E_tmp = cell(nu,nu);
    
    for i=1:nu
        for j=1:nu
            if i <= j
                E_tmp{i,j} = eye(2);
            else
                E_tmp{i,j} = zeros(2,2);
            end
        end
    end
    
    E = cell2mat(E_tmp);
end