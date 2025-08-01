clear; clc; close all;
%% Validation of input compensator
dt = 0.001;
T = 20;
% N = T/dt;
time = 0:dt:T;
N = numel(time);

% Human dynamics parameters
Mh = @(t) 0.1*sin(t) + 1.5;
Dh = @(t) 0.05*sin(0.5*t) + 0.1;
% Robot dynamics parameters
Ma = 2;
Da = 0.4;

% preallocation 
% motion related (xr, xrd, xrdd, xh, xhd, xhdd)
% Fh, lam, lamh
xr = zeros(1, N);
xrd = zeros(1, N);
xrdd = zeros(1, N);

xhd = zeros(1, N);
xhd = zeros(1, N);
xhdd = zeros(1, N);

fh = zeros(1, N);
lam = zeros(1, N);
lamh = zeros(1,N);

%% Select scenario: 1=sinusoidal, 2=move-and-stop
scenario = 1;

switch scenario
  case 1
    % sinusoidal motion at 0.5, 1.0, and 4.0 Hz
    freqs = [0.5, 1.0, 4.0];
    amp   = 0.1;  % [m] amplitude
    
    for k = 1:N
      t = time(k);
      xh(k) = amp * sum(sin(2*pi*freqs*t));
    end
    
  case 2
    % move-and-stop using arctan profile
    for k = 1:N
      t = time(k);
      xh(k) = 0.05 * atan(30 * t);
    end
    
  otherwise
    error('Unknown scenario');
end

% Compute human velocity and acceleration (finite differences)
for k = 2:N-1
  xhd(k)  = (xh(k+1)-xh(k-1))/(2*dt);
  xhdd(k) = (xh(k+1)-2*xh(k)+xh(k-1))/(dt^2);
end

%% Main simulation loop

% Compute the initial interaction force

for k = 1:N-1
  t = time(k);
  
  % 1) Compute interaction force Fh via human dynamics
  Fh(k) = Mh(t)*xhdd(k) + Dh(t)*xhd(k);
  
  fm(k) = Fh(k);

  if k==1
      fh(1) = Fh(1);
      lam(1) = Fh(1);
  else
      fh(k) = fh(k-1) + fm(k);
      lam(k) = (fh(k-1) + fm(k))...
          * (1/fh(k-1))...
          * (lam(k-1) + fm(k));
  end

  % 2) Reconstruct lambda_h (fill in per paper’s Eqn. steps)
  %    ----------------------------------------------------------------
  %    lambda(k) = <YOUR ENERGY-BASED / SMC INVERSION HERE>;
  %    ----------------------------------------------------------------
  
  % 3) Admittance: M_a·xddot_r + D_a·xdot_r = lambda_h
  %    → xddot_r = (lambda(k) - Da*xdot_r(k)) / Ma
  xrdd(k) = (lambda(k) - Da*xrd(k)) / Ma;
  
  % 4) Integrate robot ref motion
  xrd(k+1) = xrd(k) + xrdd(k)*dt;
  xr(k+1)    = xr(k)    + xrd(k)*dt;
end

%% Plot results
figure;
subplot(3,1,1);
plot(time, xh, time, xr, '--','LineWidth',1.2);
legend('Human motion','Robot ref');
xlabel('Time [s]'); ylabel('Position [m]');

subplot(3,1,2);
plot(time, Fh, 'LineWidth',1.2);
xlabel('Time [s]'); ylabel('Interaction force [N]');

subplot(3,1,3);
plot(time, lambda, 'LineWidth',1.2);
xlabel('Time [s]'); ylabel('\lambda_h [N]');

sgtitle('SMC–VAC Simulation Skeleton');



