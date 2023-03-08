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
x_s=[0 3 6 10];
y_s=[0 0 3 3];

%run sim
open_system('Part6PID');
out=sim('Part6PID');

%plot results
figure
plot(out.pos);