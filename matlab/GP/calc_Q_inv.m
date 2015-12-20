function q_inv = calc_Q_inv(Qc, tao)
    Qc_inv = inv(Qc);
    q_inv = [12 * tao^(-3) * Qc_inv, -6 * tao^(-2) * Qc_inv; ...
             -6 * tao^(-2) * Qc_inv,  4 * tao^(-1) * Qc_inv];
end