function H = computeHPred(A, B, C, ny, nu)
    H_tmp = cell(ny,nu);
    tmp = B;
    
    
    for i=1:nu
       for k=1:(i-1)
           H_tmp{k,i} = zeros(3,2);
       end
       
       for j=i:ny
           H_tmp{j,i} = C*tmp;
           tmp = A*tmp;
       end
       
       tmp = B;
    end
    
    H = cell2mat(H_tmp);
end