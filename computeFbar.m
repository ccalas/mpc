function Fbar = computeFbar(Abar, Qbar, Cbar, Tbar)
    Fbar = [Abar'*Qbar*Cbar; -Tbar*Cbar];
end