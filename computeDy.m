function Dy = computeDy(Xmin, Xmax, ny)
    Dy_tmp = cell(2*ny);
    
    for i=1:2*ny
        if i <= ny
            Dy_tmp{i,1} = Xmax;
        else
            Dy_tmp{i,1} = -Xmin;
        end
    end
    
    Dy =cell2mat(Dy_tmp);
end