function Rbar = computeRbar(R, nu)
    Rbar_tmp = cell(nu, nu);
    
    for i=1:nu
        for j=1:nu
            if i == j
                Rbar_tmp{i,j} = R;
            else
                Rbar_tmp{i,j} = zeros(size(R));
            end
        end
    end
    
    Rbar = cell2mat(Rbar_tmp);
end