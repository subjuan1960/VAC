##### **Controller Comparison:**

Input Compensator: check 26, 52, 71 \[AC1, AC2, AC3]

VAC: check 11, 72 \[VAC1, VAC2]



AC1 - measures human impedance through simple exp and compensates (offline)

AC2 - dynamically estimate and compensate through optimization (online?)

AC3 - computes real-time human influence and compensates (online)



VAC1 - variable damping for direct/indirect human intentions

VAC2 - variable damping from intention estimated based on energy consumption



##### **Simulation setting:**

human dynamics parameters:

Mh: 0.1sin(t)+1.5

Dh: 0.05sin(0.5t) + 0.1



Robot admittance parameters:

Ma: 2

Da: 0.4



Timestep: 1ms



scenario:

1 - sinusoidal motion (0.5, 1.0, 4.0 Hz)

2 - move-and-stop motion(arctan function)



Evaluation experiments:

Circle / Square


Expandable Ideas:

after VAC updated interaction force, include a little force compensation and see if the error can be mitigated.

-------------------------------------------------------
derive lyapunov stability from system
look at further improving sliding surface
how to adaptively tune the force compensator gain