function Du = computeDu(umax, umin, nu)
    Du = [umax*ones(2*nu,1);-umin*ones(2*nu,1)];
end