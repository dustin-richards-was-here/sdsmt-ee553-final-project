clc
clear
%close all

%pkg load control

% load the data from a file
data = csvread("log.csv");

% separate out the columns into their own matrices
t = data(:, 1) / 1000;
vel = data(:, 2);
power = data(:, 3);

setpoint = 2;

% plot velocity data
figure
subplot(2, 1, 1)
plot(t, vel)
ylim([0 Inf])
xlim([0 Inf])
hold on
title("Compensated Motor Step Response")
xlabel("Time (s)")
ylabel("Output Shaft Velocity (rev/sec)")

% plot power data
subplot(2, 1, 2)
plot(t, -power .* 100)
ylim([0 Inf])
xlim([0 Inf])
xlabel("Time (s)")
ylabel("Motor Power (%)")