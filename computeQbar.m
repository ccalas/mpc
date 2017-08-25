function Qbar = computeQbar(C, Q, S, ny)
    Qbar_tmp = cell(ny, ny);
    
    for i=1:ny
       for j=1:ny
          if i==j
              if i == ny
                  Qbar_tmp{ny,ny} = C'*S*C;
              else
                  Qbar_tmp{i,j} = C'*Q*C;
              end
          else
              Qbar_tmp{i,j} = zeros(size(C'*Q*C));
          end
       end
    end
       
    Qbar = cell2mat(Qbar_tmp);
end