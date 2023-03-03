%Demo 1 Layout
%This is inner loop control - pt2

%Below is Motor variables
K=.66666;
sig=20;
Kt=.5;
Ke=.5;
Ra=1;
J=.05;
b=.5;

%run sim
open_system('rho');
out=sim('rho');

%plot results
figure
%plot(out.Velocity);
plot(out.latSpeed);
figure
plot(out.turnSpeed);

% As of 3/1/23, code theoretically works
% Code is very basic and may not be
%designed correctly. Needs to be implemented
% into arduino to test 