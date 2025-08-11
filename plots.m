SMCdata = readtable("SMC_VAC_results.csv");
VACdata = readtable("VAC1_results.csv");
freedata = readtable("freespace_results.csv");

%% plot interaction forces
figure;
subplot(2,1,1);
plot(SMCdata.Time, SMCdata.InteractionForce, 'r-',  'LineWidth', 1.5); hold on;
plot(VACdata.Time, VACdata.InteractionForce, 'b--', 'LineWidth', 1.5);
plot(freedata.Time, freedata.InteractionForce, 'g-.', 'LineWidth', 1.5);
hold off;

xlabel('Time [s]');
ylabel('Interaction Force [N]');
title('Interaction Force Comparison');
legend('SMC-VAC', 'VAC1', 'Freespace');
grid on;

%% plot velocity profiles
% figure;
subplot(2,1,2);
% Human intent (should be identical across all datasets)
plot(SMCdata.Time, SMCdata.HumanVelocity, 'k:', 'LineWidth', 2); hold on;

plot(SMCdata.Time, SMCdata.RobotVelocity, 'r-',  'LineWidth', 1.5);
plot(VACdata.Time, VACdata.RobotVelocity, 'b--', 'LineWidth', 1.5);
plot(freedata.Time, freedata.RobotVelocity, 'g-.', 'LineWidth', 1.5);
hold off;

xlabel('Time [s]');
ylabel('Velocity [m/s]');
title('Velocity Tracking Comparison');
legend('Human Intent', 'SMC-VAC', 'VAC1', 'Freespace');
grid on;

