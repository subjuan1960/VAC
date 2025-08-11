clear; clc; close all;
%% Validation of input compensator
dt = 0.001;
T = 4;
% N = T/dt;
t = 0:dt:T;
N = numel(t);

% Human dynamics parameters
Mh = 0.1*sin(t) + 1.5;
Dh = 0.05*sin(0.5*t) + 0.1;
% Robot dynamics parameters
Ma = 2;
Da = 0.4;

%% preallocation 
% motion related (xr, xrd, xrdd, xh, xhd, xhdd)
% Fh, lam, lamh

x = zeros(1, N);
xd = zeros(1, N);
xdd = zeros(1, N);

fh = zeros(1, N);
fm = zeros(1, N);
lam = zeros(1, N);
lamh = zeros(1,N);
A = zeros(1, N);

%% Select scenario: 1=sinusoidal, 2=move-and-stop
scenario = 1;

switch scenario
  case 1
    % sinusoidal motion at 0.5, 1.0, and 4.0 Hz
    freq = 0.5;
    amp   = 1;  % [m] amplitude

    xhd = amp * sin(2*pi*freq*t);
    xhdd = amp * 2*pi * freq * cos(2*pi*freq*t);
    

  case 2
    A = 0.05;
    k = 30;
    % move-and-stop using arctan profile
    xh = A * atan(k * t);
    xhd = A * k ./ (1 + (k * t).^2);
    xhdd = -2 * A * k^3 * t ./ (1 + (k * t).^2).^2;
    
  otherwise
    error('Unknown scenario');
end

%% Main simulation loop

xrd = [xhd(1), xhd(1:end-1)];
xrdd = [xhdd(1), xhdd(1:end-1)];
xd(1)  = xhd(1);
xdd(1) = xhdd(1);

% initial conditions
lamh(1) = Ma * xhdd(1) + Da * xhd(1);  % Eq. 10

fh(1)   = Mh(1)*xhdd(1) + Dh(1)*xhd(1);  % Eq. 24
fm(1)   = fh(1) - Mh(1)*xdd(1) - Dh(1)*xd(1);


for k = 2:N
  fh(k) = Mh(k) * xhdd(k) + Dh(k) * xhd(k);
  fm(k) = Mh(k) * (xhdd(k) - xrdd(k)) + Dh(k) * (xhd(k) - xrd(k));

  lam(k) = Ma * xrdd(k) + Da * xrd(k) - fm(k);

  A(k) = xrd(k) / xhd(k) * (xhd(k) - xhd(k-1))^2 / (xrd(k) - xrd(k-1))^2;

  lamh(k) = A(k) * (lam(k) + fm(k));

  xdd(k) = (1/Ma) * (lamh(k) - Da * xd(k-1));
  xd(k) = xd(k-1) + xdd(k)*dt;
end

%   xdd(2) = (1/Ma) * (lamh(1) - Da * xd(1));
%   xd(2) = xd(1) + xdd(2)*dt; 
% 
% for k = 2:N-1
%   fh(k) = Mh(k) * xhdd(k) + Dh(k) * xhd(k);
%   fm(k) = Mh(k) * (xhdd(k) - xdd(k)) + Dh(k) * (xhd(k) - xd(k));
% 
%   lam(k) = Ma * xdd(k) + Da * xd(k) - fm(k);
% 
%   A(k) = xd(k) / xhd(k) * (xhd(k) - xhd(k-1))^2 / (xd(k) - xd(k-1))^2;
% 
%   lamh(k) = A(k) * (lam(k) + fm(k));
% 
%   xdd(k+1) = (1/Ma) * (lamh(k) - Da * xd(k));
%   xd(k+1) = xd(k) + xdd(k+1)*dt;
% end

%% Plot results
figure;
subplot(4,1,1);
plot(t, xhd, 'r--', t, xrd, 'b');
xlabel('Time [s]'); ylabel('velocity [m/s]');
legend('x_hd (intent)', 'xd (robot)');
title('Position tracking');

subplot(4,1,2);
plot(t, fm);
xlabel('Time [s]'); ylabel('Interaction Force [N]');
title('Interaction force f_m');

subplot(4,1,3);
plot(t, lamh);
xlabel('Time [s]'); ylabel('Compensation \lambda_h [N]');
title('Input compensation');

subplot(4,1,4);
plot(t, xhd, 'r--', t, xd, 'b');
xlabel('Time [s]'); ylabel('velocity [m/s]');
legend('x_hd (intent)', 'xd (robot)');
title('Position tracking');



%%
% Prepare data for export
data = table(...
    t', ...
    xhd', ...       % Human velocity
    xrd', ...        % Robot velocity
    fm', ...        % Interaction force
    'VariableNames', {'Time', 'HumanVelocity', 'RobotVelocity', 'InteractionForce'} ...
);

% Save to CSV
writetable(data, 'freespace_results.csv');