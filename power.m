% VAC Heuristic Simulation in MATLAB
clear; clc;

% Parameters
dt = 0.001;     % time step
T = 10;         % total time
N = T / dt;

% Preallocate variables
x = zeros(1, N);
x_dot = zeros(1, N);
x_ddot = zeros(1, N);
Fh = zeros(1, N);
B = zeros(1, N);
Ph = zeros(1, N);

% Admittance parameters
M = 1.0;         % virtual mass
B_max = 50.0;    % max damping
B_min = 5.0;     % min damping
k = 50.0;        % gain for damping update

% Force profile
for i = 1:N
    t = (i-1)*dt;
    if t >= 1 && t < 3
        Fh(i) = 5.0;
    elseif t >= 4 && t < 6
        Fh(i) = -5.0;
    elseif t >= 7 && t < 9
        Fh(i) = 5.0;
    else
        Fh(i) = 0.0;
    end
end

% Initial damping
B(1) = B_max;

% Simulation loop
for i = 1:N-1
    % Power
    Ph(i) = Fh(i) * x_dot(i);
    
    % Damping adaptation heuristic
    if abs(Fh(i)) < 0.05
        B_dot = 10.0;
    else
        B_dot = -k * sign(Ph(i));
    end
    
    % Update damping with saturation
    B(i+1) = B(i) + B_dot * dt;
    B(i+1) = min(max(B(i+1), B_min), B_max);
    
    % Admittance dynamics
    x_ddot(i) = (Fh(i) - B(i) * x_dot(i)) / M;
    x_dot(i+1) = x_dot(i) + x_ddot(i) * dt;
    x(i+1) = x(i) + x_dot(i) * dt;
end

% Plot results
time = (0:N-1)*dt;
figure;
subplot(3,1,1);
plot(time, Fh, 'LineWidth', 1.5); ylabel('F_h (N)');
title('Human Force and Velocity'); grid on;

subplot(3,1,2);
plot(time, x_dot, 'LineWidth', 1.5); ylabel('x\_dot (m/s)');
grid on;

subplot(3,1,3);
plot(time, B, 'LineWidth', 1.5); ylabel('Damping B'); xlabel('Time (s)');
title('Adaptive Damping Behavior'); grid on;

% --- Define sliding surface ---
P0 = 0.1;  % desired power
S = Fh .* x_dot - P0;

% --- Plot Sliding Surface Over Time ---
figure;
plot(time, S, 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Sliding Surface S');
title('Sliding Surface Convergence');
grid on;

% --- Plot Force-Velocity Phase Portrait ---
figure;
plot(x_dot, Fh, '.', 'MarkerSize', 5);
xlabel('Velocity \dot{x}');
ylabel('Human Force F_h');
title('Force vs Velocity Phase Plot');
grid on;

% --- Optional: Derivative of Sliding Surface ---
S_dot = [diff(S)/dt, 0];  % numerical derivative with padding
figure;
plot(time, S_dot, 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('S\_dot');
title('Time Derivative of Sliding Surface');
grid on;
