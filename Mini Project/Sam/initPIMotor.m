%declare k and sigma
K=.66666;
sig=20;
Kt=.5;
Ke=.5;
Ra=1;
J=.05;
b=.5;

%run the simulation
open_system('PImotor');
out=sim('PImotor');

%plot results
figure
%plot(out.Velocity);
plot(out.position);