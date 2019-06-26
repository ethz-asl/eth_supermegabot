function plotDataWithName(data, idx, color)
    plot(data(idx).time, data(idx).data, color{:});
end