fig = figure;
ax = axes(fig);                          % create axes FIRST
hold(ax, 'on');
axis(ax, 'equal');
xlim(ax, [-1.2 1.2]); ylim(ax, [-2 0.5]);

mass    = plot(ax, -0.5, -0.5, 'o', 'MarkerSize', 20, 'MarkerFaceColor', 'r');
rodline = plot(ax, [0, -0.5], [0, -0.5], 'LineWidth', 1.5);
cart = plot(ax, [-1.1, 1.1], [0.1, 0.1], LineWidth=20, Color='b');
l = 1;
tspan = [0 20];
x0 = [pi/2; 0];                          % column vector
angle = linspace(0,2*pi,100); 
disk_x = 0.2*sin(angle);
disk_y = 0.2+0.2*cos(angle);
patch(ax, 'XData', disk_x, 'YData', disk_y, 'FaceColor', [0.3 0.3 0.6]);

[t, x] = ode45(@odefunction, tspan, x0); % array form: x is nTimePoints x nStates

for k = 1:length(t)
    theta = x(k,1);                      % column 1 = theta, row k = time step k
    xp = l*sin(theta);
    yp = -l*cos(theta);

    set(mass,    'XData', xp,       'YData', yp); 
    set(rodline, 'XData', [0, xp],  'YData', [0, yp]);
    drawnow limitrate;
    pause(0.01);
end

function dx = odefunction(t, x)
    theta     = x(1);
    theta_dot = x(2);
    g = 9.81; l = 1;
    dx = [theta_dot; -(g/l)*sin(theta)];   % column vector!
end