function Tbar = computeTbar(Q, C, S, ny)
    Tbar_tmp = cell(ny, ny);
    
    for i=1:ny
       for j=1:ny
          if i==j
              if i==ny
                  Tbar_tmp{ny,ny} = S*C;
              else
                  Tbar_tmp{i,j} = Q*C; 
              end
          else
              Tbar_tmp{i,j} = zeros(size(Q*C)); 
          end
       end
    end
    
    Tbar = cell2mat(Tbar_tmp);
end