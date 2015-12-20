function h = plot2d(data, type, linewidth)
    data = reshape(data, 2, []);
    h = plot(data(1,:), data(2,:), type, 'LineWidth', linewidth);
end