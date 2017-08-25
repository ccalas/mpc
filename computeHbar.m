function Hbar = computeHbar(Cbar, Qbar, Rbar)
    Hbar = Cbar'*Qbar*Cbar + Rbar;
end