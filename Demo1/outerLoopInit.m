%Outer loop control - pt 5

%Below is Motor variables
K=.66666;
sig=20;
Kt=.5;
Ke=.5;
Ra=1;
J=.05;
b=.5;

%run sim
open_system('outerLoop');
out=sim('outerLoop');

%plot results
figure
plot(out.turnSpeed);
figure
plot(out.steerSpeed);

% 3/3/23 untested implementation