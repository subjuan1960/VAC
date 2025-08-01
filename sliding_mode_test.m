clear; clc; close all;

%% Simulation parameters
dt = 0.001;     % Timestep [s] 1000hz
T = 20;         % Sim time [s]
N = T / dt;     % Number of steps

% Human Impedance parameters
M_h = 1.0;      % Human inertia [kg]
B_h = 10.0;     % Human damping [Ns/m]
K_h = 50.0;     % human stiffness[N/m]

Kph = 20;    % human P intent gain
Kdh = 5;     % human D intent gain
Kh  = 50;     % human passive stiffness
Bh  = 10;     % human passive damping

% Admittance parameters
M_a = 1.0;
B_a = zeros(1, N);
B_a(1) = 20;
Bmin = 1.0;
Bmax = 100.0;
k_slide = 500.0;
gamma = 0.05;

% plant parameters (1 DOF mass-damper)
M_r = 1.0;
B_r = 5.0;
Kp = 200;
Kd = 20;

epsilon = 1e-2;

%% preallocate vector
x = zeros(1, N);
xdot = zeros(1, N);

x_r = zeros(1, N);
xdot_r = zeros(1, N);
xddot  = zeros(1, N);

S_arr       = zeros(1,N);
Fh_arr  = zeros(1,N);
%% Simulation loop
% x_des = zeros(1, N);
% x_des(time>=1 & time<5) = 0.5;
x_des = 0.5 * ones(1, N);

for k = 1:N-1
    % 1） Human model: computes Fh based on current x, xdot, and target
    % x_des
    Fh = human_model(x(k), xdot(k), xddot(k), x_des(k), M_h, B_h, K_h, Kph, Kdh);
    Fh_arr(k) = Fh;

    % 2) SMC-VAC: compute S, update Ba, and evaluate admittance
    S = (Fh*xdot_r(k)) / (abs(Fh)+epsilon)  -  gamma*Fh;
    S_arr(k)      = S;

    B_a(k+1) = B_a(k) + k_slide*S*dt;
    B_a(k+1) = min(max(B_a(k+1), Bmin), Bmax);
    xddot_r = (Fh - B_a(k+1)*xdot_r(k)) / M_a;

    xdot_r(k+1) = xdot_r(k) + xddot_r*dt;
    x_r(k+1) = x_r(k) + xdot_r(k)*dt;

    % 3) Inner 1-DOF plant tracks xdot_r
    v_err = xdot_r(k) - xdot(k);

    u = Kp*v_err - Kd*xdot(k);
    xddot(k+1) = (u - B_r *xdot(k)) / M_r;
    xdot(k+1) = xdot(k) + xddot(k+1)*dt;
    x(k+1) = x(k) + xdot(k+1)*dt;
end


%% Plots
% time vector
time = (0:N-1)*dt;

% 1) Sliding surface
figure;
plot(time, S_arr, 'LineWidth',1.5);
xlabel('Time [s]'); ylabel('S = F_h·\dot x_r - \gamma F_h^2');
title('Sliding Surface Convergence');
grid on;

% 2) Adaptive damping
figure;
plot(time, B_a, 'LineWidth',1.5);
xlabel('Time [s]'); ylabel('B_a [Ns/m]');
title('Adaptive Virtual Damping');
grid on;

% 3) Reference vs actual velocity
figure;
plot(time, xdot_r, 'b', time, xdot, 'r--', 'LineWidth',1.2);
xlabel('Time [s]'); ylabel('Velocity [m/s]');
legend('\dot x_r (ref)', '\dot x (plant)');
title('Velocity Tracking');
grid on;

% 4) Reference vs actual position
figure;
plot(time, x_r, 'b', time, x, 'r--', 'LineWidth',1.2);
xlabel('Time [s]'); ylabel('Position [m]');
legend('x_r (ref)', 'x (plant)');
title('Position Tracking');
grid on;

% 5) Human interaction force
figure;
plot(time, Fh_arr, 'k', 'LineWidth',1.2);
xlabel('Time [s]'); ylabel('F_h [N]');
title('Human Interaction Force');
grid on;

% 6) Cumulative human work & passivity observer
Wh  = cumtrapz(time, Fh_arr .* xdot);   % W_h = ∫ F_h * \dot x dt
Epo = cumtrapz(time, Fh_arr .* xdot);   % same integral

figure;
subplot(2,1,1);
plot(time, Wh, 'm', 'LineWidth',1.2);
ylabel('W_h [J]'); title('Cumulative Human Work');
grid on;

subplot(2,1,2);
plot(time, Epo, 'c', 'LineWidth',1.2);
ylabel('E_{PO} [J]'); xlabel('Time [s]');
title('Passivity Observer (E_{PO} \ge 0)');
grid on;

%% Simple Animation of the 1-DOF Mass
figure('Color','w');
ax = axes('XLim',[0 0.6],'YLim',[-0.1 0.1],'YTick',[]);
xlabel('Position [m]'); title('Transparent Push—Firm Stop');

hDot  = plot(0,0,'o','MarkerSize',20,'MarkerFaceColor','b');
hText = text(0.02,0.8,'','Units','normalized');

for k = 1:50:N
    set(hDot,'XData', x(k), 'YData', 0);
    set(hText,'String',sprintf('t=%.2f s\nB_a=%.1f Ns/m', k*dt, B_a(k)));
    drawnow;
    pause(0.01);
end

%% Human model (simple impedance with PID)
function Fh = human_model(x, xdot, xddot, x_des, M_h, B_h, K_h, Kph, Kdh)
    % PD "intent" towards x_des: lets say +5m to the right
    Fcmd = Kph*(x_des - x) + Kdh*(0 - xdot);
    % Passive impedance
    % Fimp = -Kh*(x - x_des) - Bh*xdot;
    % Fimp = M_h* xddot ...
    %     + B_h* xdot ...
    %     + K_h*(x - x_des);
    
    % interaction force
    Fh = Fcmd;
end