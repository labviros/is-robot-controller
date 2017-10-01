%clc, close all, clear all;

Ax = 3200.0;
Ay = 1600.0;
syms t w phi x0 y0;

ax = (2 * Ax) ./ (3 - cos(2*(w*t + phi)));
ay = (Ay / 0.35) ./ (3 - cos(2*(w*t + phi)));
x = ax.*cos(w*t + phi)/2 + x0;
y = ay.*sin(2*(w*t + phi))/2 + y0;
dx = diff(x, t);
dy = diff(y, t);

f = 2.5;
T =  1/f;
tf = 25;
step = tf/T;

t = 0:T:tf;
w = 2*pi/tf;
phi = pi/3;
x0 = -200.0;
y0 = 0.0;

X = eval(x);
Y = eval(y);
dX = eval(dx);
dY = eval(dy);
v = sqrt(dX.*dX + dY.*dY);

positions = [X;Y];
speeds = [dX;dY];
save('positions.mat', 'positions', '-ascii');
save('speeds.mat', 'speeds', '-ascii');

return
figure
for i=1:length(t)
    subplot(2,1,1)
    cla
    hold on;
    plot(X(1),Y(1),'ok');
    plot(X(i),Y(i),'sk');
    plot(X(1:i),Y(1:i));
    axis([(-Ax/2+x0) (Ax/2+x0) (-Ay/2+y0) (Ay/2+y0)]);
    grid on;
    
    subplot(2,1,2)
    cla
    hold on;
    plot(t(i),v(i),'sk');
    plot(t(1:i), v(1:i));
    axis([0 t(end) 0 max(v)]);
    grid on;
    
    %pause(T)
end