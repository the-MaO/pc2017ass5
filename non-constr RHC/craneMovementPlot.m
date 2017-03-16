function craneMovementPlot(x ,y, x_ang, y_ang, xLow, xHigh, yLow, yHigh, con, stringLength, figTitle)
% craneMovementPlot(x ,y, x_ang, y_ang, xLow, xHigh, yLow, yHigh, margin)
% plots the movement of the gantry crane, and of the pendulum in XY plane
% also plots corners of the square to be tracked and the set constraints
% requires "hline and vline" toolbox
% Legend: 
% green - crane
% blue - pendulum
% red point - square corner
% red dotted line - constraint

figure;
scatter(x+stringLength*sin(x_ang),y+stringLength*sin(y_ang), '.b'); % pendulum
hold on
scatter(x,y,'.g'); % crane

% corners of the square
scatter(xLow,yLow, 'r');
scatter(xLow,yHigh, 'r');
scatter(xHigh,yHigh, 'r');
scatter(xHigh,yLow, 'r');

% constraint lines
hline(con(2,2));
hline(con(2,4));
hline(con(2,6));
hline(con(2,8));

vline(con(1,1));
vline(con(1,3));
vline(con(1,5));
vline(con(1,7));

% plot elements
legend({'pendulum','cart'});
title(figTitle);

end