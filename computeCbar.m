function Cbar = computeCbar(A, B, ny, nu)
    Cbar_tmp = cell(ny,nu);
    tmp = B;
    
    
    for i=1:nu
       for k=1:(i-1)
           Cbar_tmp{k,i} = zeros(size(B));
       end
       
       for j=i:ny
           Cbar_tmp{j,i} = tmp;
           tmp = A*tmp;
       end
       
       tmp = B;
    end
    
    Cbar = cell2mat(Cbar_tmp);
end