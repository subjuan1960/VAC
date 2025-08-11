clear; clc; close all;

%% Parameters
dt = 0.001;
T = 4;
t = 0:dt:T;
N = numel(t);

% Human dynamics
Mh = 0.1*sin(t) + 1.5;
Dh = 0.05*sin(0.5*t) + 0.1;

% Robot dynamics
Ma = 2;
Da_init = 0.4;  % initial damping

Dmin = 0.05;
Dmax = 100.0;

%% Motion profile (human velocity intent)
freq = 4;
amp   = 4;  % [m/s] amplitude

xhd  = amp * sin(2*pi*freq*t);
xhdd = amp * 2*pi * freq * cos(2*pi*freq*t);

% A = 30;
% k = 0.05;
% xhd = A * k ./ (1 + (k * t).^2);
% xhdd = -2 * A * k^3 * t ./ (1 + (k * t).^2).^2;

%% Preallocation
x   = zeros(1, N);
xd  = zeros(1, N);
xdd = zeros(1, N);

fh  = zeros(1, N);
fm  = zeros(1, N);
fm_filt = zeros(1, N);

D   = zeros(1, N);
Dd  = zeros(1, N);
S   = zeros(1, N);

%% Sliding mode / adaptation parameters
eps     = 1e-6;
gamma   = 0.05;  % power term weight
ks      = 1;     % sliding gain
phi     = 0.01;  % boundary layer width
Dd_max  = 50;    % max rate of change of D [N·s/m^2]
alpha_f = 0.05;  % low-pass filter coeff for fm

sat = @(z) max(min(z,1),-1);  % saturation function

%% Initial conditions (match human at t=0 to avoid big initial force)
xd(1)  = xhd(1);
xdd(1) = xhdd(1);
D(1)   = Da_init;

% Initial human force and filtered fm
fh(1)     = Mh(1)*xhdd(1) + Dh(1)*xhd(1);
fm(1)     = fh(1) - Mh(1)*xdd(1) - Dh(1)*xd(1);
fm_filt(1)= fm(1);

%% Simulation loop
for k = 2:N-1
    % 1) Human force at this step
    fh(k) = Mh(k)*xhdd(k) + Dh(k)*xhd(k);

    % 2) Interaction force from mismatch
    fm(k) = fh(k) - Mh(k)*xdd(k) - Dh(k)*xd(k);

    % 3) Low-pass filter fm
    fm_filt(k) = (1-alpha_f)*fm_filt(max(k-1,1)) + alpha_f*fm(k);

    % 4) Sliding surface
    S(k) = fm_filt(k)*xd(k) / (abs(fm_filt(k))+eps) - gamma*fm_filt(k)^2;
    % S(k) = (fm(k) - fm(k-1))*xd(k);
    % 5) Damping adaptation with boundary layer + rate limit
    Dd(k) = -ks * sat(S(k)/phi);
    % Dd(k) = -ks * S(k);
    Dd(k) = max(min(Dd(k), Dd_max), -Dd_max);
    D(k)  = D(k) + Dd(k)*dt;
    D(k)  = min(max(D(k), Dmin), Dmax);

    % 6) Admittance integration
    xdd(k+1) = (1/Ma) * (fh(k) - D(k) * xd(k));
    xd(k+1)  = xd(k) + xdd(k+1)*dt;
end

%% Plot results
figure;
subplot(3,1,1);
plot(t, S, 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('S');
title('Sliding Surface Convergence'); grid on;

subplot(3,1,2);
plot(t, xhd, 'r--', t, xd, 'b');
xlabel('Time [s]'); ylabel('Velocity [m/s]');
legend('Human intent', 'Robot');
title('Velocity Tracking');

subplot(3,1,3);
plot(t, fm);
xlabel('Time [s]'); ylabel('f_m [N]');
title('Interaction Force');



% %% energy consumption
% % 6) Cumulative human work & passivity observer
% Wh  = cumtrapz(t, abs(fm .* xd));   % W_h = ∫ F_h * \dot x dt
% 
% figure;
% subplot(2,1,1);
% plot(t, Wh, 'm', 'LineWidth',1.2);
% ylabel('W_h [J]'); title('Cumulative Human Work');
% grid on;

figure;
subplot(3,1,1);
plot(t, fm,'LineWidth',1.2);
ylabel('Power J/s'); title('motion intent');
grid on;

subplot(3,1,2);
plot(t, fh,'LineWidth',1.2);
ylabel('F/N'); title('intended human force');
grid on;

subplot(3,1,3);
plot(t, D);
xlabel('Time [s]'); ylabel('Damping [N·s/m]');
title('Adapted Damping');
