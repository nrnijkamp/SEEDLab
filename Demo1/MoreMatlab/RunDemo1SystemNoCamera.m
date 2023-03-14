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
phi_des = 180*pi/180; %radians
rho_dot_des = 0.5; % ft/s
r = 7.5 * ft_per_cm; % wheel radius; ft
d = 28 * ft_per_cm; % turn diameter; ft

% Run simulation
open_system("Demo1SystemNoCamera");
out = sim("Demo1SystemNoCamera");

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
yline(phi_des);
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
plot(out.posx.data, out.posy.data);
title("Robot Path");