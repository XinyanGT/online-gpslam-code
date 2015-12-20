function plot_ellipse(c, A, style, u)
    
    A = inv(A);
    [V, D] = eig(A);

    a = 1/sqrt(D(1,1));
    b = 1/sqrt(D(2,2));
    
    npoints = 300;
    theta = linspace(-pi, pi, npoints)';
    x = a * cos(theta);
    y = b * sin(theta);
    
    points = [x';y'];
    points = u * (V' * points);
    
    points = points + repmat(c, 1, npoints);
    points = points(:);
    plot2d(points, style, 1);
    
   
end