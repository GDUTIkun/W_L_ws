# Measurement and estimation contract

## Current facts

- Simulation: MuJoCo supplies an unwrapped hinge qpos. Adapter applies only sign and configured
  offset; qvel is read independently. There is no numerical differentiation of wrapped q.
- Experimental STM32 C620 wheel path: `Data_Process` detects encoder half-range crossings, updates
  `Total_Round`, forms `Total_Encoder`, then emits multi-turn `Now_Angle`; `Now_Omega` comes directly
  from the motor speed field (`firmware/stm32/Hardware/dvc_motor_dji.cpp:457-492`).
- Experimental UART payload sends only float joint position and velocity
  (`firmware/stm32/App/uart_protocol_test.cpp:624-636`). It does not send modulo angle, revolution
  count, wrap epoch, domain tag or validity flag.
- ROS `RobotState` and `JointState` paths forward q/dq verbatim. The repository’s experimental
  UART2 path is not the approved production protocol.

## Contract consequence

R0/R3 fit both existing simulation and experimental C620 semantics. R1 cannot replace raw q at the
shared state boundary: a ±π wrap would make ordinary residuals jump by about 2π and could create a
false differentiated velocity spike in a consumer that reconstructs dq. R2 is implementable only
after adding a separately transported integer revolution count/wrap epoch; that schema does not
exist today.

Accumulated rotation is not needed by current nominal WBC/NMPC physical evaluation. If future
odometry or maintenance counters require it, retain it separately from local physical phase and
from xi. Real-hardware reset, power-cycle, packet-loss and encoder rollover semantics remain
unfrozen and must be resolved with the formal Raspberry Pi–STM32 protocol Phase.
