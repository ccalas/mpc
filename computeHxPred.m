function Hx = computeHxPred(A, B, nu)
    Hx_tmp = cell(nu);
    tmp = B;
    
    for i=1:nu
       for k=1:(i-1)
           Hx_tmp{k,i} = zeros(6,2);
       end
       
       for j=i:nu
           Hx_tmp{j,i} = tmp;
           tmp = A*tmp;
       end
       
       tmp = B;
    end
    
    Hx = cell2mat(Hx_tmp);
end