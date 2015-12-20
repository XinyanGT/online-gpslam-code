function q = calc_Q(Qc, tao)
        q = [1/3 * tao^3 * Qc, 1/2 * tao^2 * Qc; ...
             1/2 * tao^2 * Qc, tao * Qc];
end