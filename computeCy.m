function Cy = computeCy(ny,n_state)
    Cy_tmp1 = cell(ny,ny);
    Cy_tmp2 = cell(ny,ny );
    %Avant 5 a la place de 7
    for i=1:ny
       for j=1:ny
          if i==j
              Cy_tmp1{i,j} = eye(n_state);
              Cy_tmp2{i,j} = -eye(n_state);
          else
              Cy_tmp1{i,j} = zeros(n_state,n_state);
              Cy_tmp2{i,j} = zeros(n_state,n_state);
          end
       end
    end
    
    Cy = [cell2mat(Cy_tmp1);cell2mat(Cy_tmp2)];
end