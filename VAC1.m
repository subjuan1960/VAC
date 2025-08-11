clear; clc; close all;
%%
clamp = @(x, minVal, maxVal) max(min(x, maxVal), minVal);

dt = 0.001;
T = 4;
t = 0:dt:T;
N = numel(t);

% Human dynamics parameters
Mh = 0.1*sin(t) + 1.5;
Dh = 0.05*sin(0.5*t) + 0.1;
% Robot dynamics parameters
Ma = 2;
Da = 0.4;

Dmin = 0.05;
Dmax = 50.0;
Fstd = 0;
%% preallocation
x = zeros(1, N);
xd = zeros(1, N);
xdd = zeros(1, N);

fh = zeros(1, N);
fm = zeros(1, N);
lam = zeros(1, N);
lamh = zeros(1, N);

D = zeros(1, N);

fcomp = zeros(1, N);
%% Sinusoidal motion profile
freq = 0.5;
amp   = 1;  % [m] amplitude

xhd = amp * sin(2*pi*freq*t);
xhdd = amp * 2*pi * freq * cos(2*pi*freq*t);
%% sim loop
xd(1) = xhd(1);
xdd(1) = xhdd(1);

Fs = Mh(1) * xhdd(1) + Dh(1) * xhd(1);

D(1) = Da;

for k = 2:N
    Fs = Mh(k) * xhdd(k) + Dh(k) * xhd(k);

    D(k) = clamp(Fstd / abs(xd(k-1)+ 1e-6), Dmin, Dmax);
    
    xdd(k) = (1/Ma) * (Fs - D(k)*xd(k-1));
    xd(k) = xd(k-1) + xdd(k) * dt;

    fm(k) = Mh(k) * (xhdd(k) - xdd(k)) + Dh(k) * (xhd(k) - xd(k));
    fcomp(k) = Ma * xdd(k) + (D(k) - D(k-1)) * xd(k);
end

%% Plot
figure;
subplot(4,1,1); plot(t, xhd, 'r--', t, xd, 'b'); title('Velocity Tracking');
legend('xhd (human)', 'xd (robot)');

subplot(4,1,2); plot(t, D); title('Adapted Damping D(t)');

subplot(4,1,3); plot(t, fm); title('Interaction Force f_m(t)');

subplot(4,1,4); plot(t, fcomp); title('Compensation Force fcomp(t)');

%%
% Prepare data for export
data = table(...
    t', ...
    xhd', ...       % Human velocity
    xd', ...        % Robot velocity
    fm', ...        % Interaction force
    'VariableNames', {'Time', 'HumanVelocity', 'RobotVelocity', 'InteractionForce'} ...
);

% Save to CSV
writetable(data, 'VAC1_results.csv');
