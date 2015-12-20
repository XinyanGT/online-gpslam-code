function phi = calc_Phi(tao)
    phi  = [eye(3), tao * eye(3); ...
            zeros(3), eye(3)];
end