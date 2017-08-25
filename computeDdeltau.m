function Ddu = computeDdeltau(Dumax, Dumin, nu)
    Ddu = [Dumax*ones(2*nu,1);-Dumin*ones(2*nu,1)];
end