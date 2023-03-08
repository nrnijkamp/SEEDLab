%Testing control sys - pt 6

%Below are Motor variables
K=.66666;
sig=20;
Kt=.5;
Ke=.5;
Ra=1;
J=.05;
b=.5;


rhoDot_d=[0.0 5.0];

%run sim
open_system('Part6PID');
out=sim('Part6PID');

%plot results
figure
plot(out.pos);