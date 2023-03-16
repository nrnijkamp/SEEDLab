% Copyright (C) 2023 Dawson J. Gullickson All rights reserved.

in_per_cm = 0.3937008;
ft_per_cm = in_per_cm/12;

% Motor parameters
K = .66666;
sig = 20;
Kt = .5;
Ke = .5;
Ra = 1;
J = .05;
b = .5;

% Block paramters
xs = [0 3 6 10];
ys = [0 0 3 3];
rho_dot_des = 0.2; % ft/s
r = 7.5 * ft_per_cm; % wheel radius; ft
d = 28 * ft_per_cm; % turn diameter; ft

% Run simulation
open_system("Demo1System");
out = sim("Demo1System");

% Plot responses
close all;
figure;
hold on;
plot(out.posx);
plot(out.posy);
legend(["x", "y"]);
title("Position");
hold off;

figure;
hold on;
plot(out.phi);
plot(out.phi_des);
legend(["phi", "phi_des"]);
title("Angle");
hold off;

figure;
hold on;
plot(out.rho_dot);
yline(rho_dot_des);
legend(["rho_dot", "rho_dot_des"]);
title("Speed");
hold off;

figure;
hold on;
plot(out.posx.data, out.posy.data);
plot(xs, ys, ".");
title("Robot Path");
hold off;