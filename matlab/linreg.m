function [xlin, ylin] = linreg(xdata1, ydata1)
    fitResults1 = polyfit(xdata1,ydata1,1);
    xlin = linspace(xdata1(1), xdata1(end));
    ylin = polyval(fitResults1,xlin);
end