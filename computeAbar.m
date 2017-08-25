function Abar = computeAbar(A, ny)
    Abar_tmp = cell(ny,1);
    tmp = A;
    
    for i=1:ny
        Abar_tmp{i,1} = tmp;
        tmp = tmp*A;
    end
    
    Abar = cell2mat(Abar_tmp);
        
end