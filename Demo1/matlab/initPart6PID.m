%Testing control sys - pt 6

%Below are Motor variables
K=.66666;
sig=20;
Kt=.5;
Ke=.5;
Ra=1;
J=.05;
b=.5;

phi_0=0;x_0=0;y_0=0;

rhoDot_d=.1; %ft/s
x_s=[0 0 6 10];
y_s=[0 0.25 0 0];

%run sim
open_system('Part6PID');
out=sim('Part6PID');

%sim('Part6PID')

close all;
%plot results
figure
plot(out.posx);

figure
plot(out.posy);
figure
plot(out.phi_Des);
figure
plot(out.phiDot_Des);
figure
plot(out.rhoDot_des);

