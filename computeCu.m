function Cu = computeCu(nu)
    Cu_tmp1 = cell(nu,nu);
    Cu_tmp2 = cell(nu,nu);
    
    for i=1:nu
       for j=1:nu 
          if i==j
              Cu_tmp1{i,j} = eye(2);
              Cu_tmp2{i,j} = -eye(2);
          else
              Cu_tmp1{i,j} = zeros(2,2);
              Cu_tmp2{i,j} = zeros(2,2);
          end
       end
    end
    
    Cu = [cell2mat(Cu_tmp1);cell2mat(Cu_tmp2)];
end