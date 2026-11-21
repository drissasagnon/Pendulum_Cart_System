subplot(2,2,1);
fig = figure(1);
ax1 = axes(fig); 
x = linspace(-3.8,3.8);
y_cos = cos(x);
plot(ax1, x,y_cos);
title('Subplot 1: Cosine')

subplot(2,2,2);
fig2 = figure(2);
ax2 = axes(fig2); 
y_poly = 1 - x.^2./2 + x.^4./24;
plot(ax2,x,y_poly,'g');
title('Subplot 2: Polynomial')

subplot(2,2,[3,4]);

fig3 = figure(3);
ax3 = axes(fig3); 
plot(x,y_cos,'b',x,y_poly,'g');
title('Subplot 3 and 4: Both')