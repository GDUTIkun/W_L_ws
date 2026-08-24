% Startup values for a symmetric two-wheel-leg floating-base LQR + QP demo.
%
% Coordinate convention:
%   - hip is the origin
%   - +x points right, +z poi nts up
%   - qh = 0 and qk = 0 put both links vertically downward
%   - positive qh swings the thigh toward +x
%   - positive qk bends the shank further toward +x relative to the thigh
%   - qw is the wheel spin relative to the shank/wheel fork
%   - positive qw is counterclockwise in the x-z plane

clc;
clear;

% Default command for the Stage-1 body-force injection chain in source.slx.
% The From Workspace block is named stage1ExternalForce but intentionally
% retains its existing variable expression, simin.  A two-sample, zero
% command keeps all legacy simulations disturbance-free unless a runner
% overrides simin after startup.
simin = [0, 0, 0, 0; 1.0e4, 0, 0, 0];

thisFile = mfilename("fullpath");
simulateDir = fileparts(thisFile);
addpath(simulateDir);
modelDir = fileparts(fileparts(simulateDir));
addpath(fullfile(modelDir, "code"));

leg = struct();
leg.L1 = 0.35;
leg.L2 = 0.35;
leg.c1 = leg.L1 / 2;
leg.c2 = leg.L2 / 2;
leg.m1 = 1.20;
leg.m2 = 0.80;
leg.width = 0.04;
leg.depth = 0.04;
leg.g = 9.81;

% Uniform rectangular rods, inertia about the out-of-plane joint axis.
leg.I1 = leg.m1 * (leg.L1^2 + leg.width^2) / 12;
leg.I2 = leg.m2 * (leg.L2^2 + leg.width^2) / 12;

% Wheel parameters. The wheel spin inertia is about the wheel axle.
leg.r = 0.08;
leg.mw = 0.35;
leg.Iw = 0.5 * leg.mw * leg.r^2;
baseBodyMass = 3.0;

traj = struct();
% First choose the nominal wheel height from the old hip-centered geometry;
% the horizontal offset is then corrected by the full-robot balance solve.
traj.nominalOffset = deg2rad([-19; 38]);
traj.defaultHeightReduction = 0.08;
nominalKin = wheel_leg_kinematics([traj.nominalOffset; 0], ...
    zeros(3, 1), zeros(3, 1), leg);
traj.offset = wheel_leg_inverse_kinematics( ...
    nominalKin.pO + [0; traj.defaultHeightReduction], ...
    zeros(2, 1), zeros(2, 1), leg);
traj.qw0 = 0;

q_joint0 = traj.offset;
dq_joint0 = zeros(2, 1);
ddq_joint0 = zeros(2, 1);

kin0 = wheel_leg_kinematics([q_joint0; traj.qw0], [dq_joint0; 0], ...
    [ddq_joint0; 0], leg);

% A bent leg with unequal link masses does not have its CoM on the hip-wheel
% line. Solve the symmetric static balance condition xCoM = xWheel so the
% nominal wheel position is a true equilibrium of the underactuated plant.
wheelHeight0 = kin0.pO(2);
robotMass = baseBodyMass + 2 * (leg.m1 + leg.m2 + leg.mw);
wheelOffset0 = kin0.pO(1);
for iteration = 1:30
    q_joint0 = wheel_leg_inverse_kinematics( ...
        [wheelOffset0; wheelHeight0], zeros(2, 1), zeros(2, 1), leg);
    thighComX = leg.c1 * sin(q_joint0(1));
    shankComX = leg.L1 * sin(q_joint0(1)) ...
        + leg.c2 * sin(sum(q_joint0));
    wheelOffset0 = 2 * (leg.m1 * thighComX ...
        + leg.m2 * shankComX + leg.mw * wheelOffset0) / robotMass;
end
q_joint0 = wheel_leg_inverse_kinematics( ...
    [wheelOffset0; wheelHeight0], zeros(2, 1), zeros(2, 1), leg);
kin0 = wheel_leg_kinematics([q_joint0; traj.qw0], [dq_joint0; 0], ...
    [ddq_joint0; 0], leg);
traj.offset = q_joint0;

traj.qJoint0 = q_joint0;
traj.dqJoint0 = dq_joint0;
traj.ddqJoint0 = ddq_joint0;
traj.xO0 = kin0.pO(1);
traj.zO0 = kin0.pO(2);
traj.thetaWheelBase0 = sum(q_joint0);
% Scheme 1: use the final upper-layer body force to generate a bounded wheel
% equilibrium, then approach it through a stateful second-order governor.
traj.wheelPositionPlanning = true;
traj.wheelPositionPlanner = "lqr";
traj.wheelPositionForceSource = "reference_acceleration";
traj.wheelPositionForceScale = 0.20;
traj.wheelPositionKneeMin = deg2rad(25);
traj.wheelPositionFrequencyHz = 2.0;
traj.wheelPositionDamping = 1.0;
traj.wheelPositionVelocityMax = 0.15;
traj.wheelPositionAccelerationMax = 0.5;
traj.wheelLqrQ = [4; 1];
traj.wheelLqrR = 200;

ctrl = struct();
ctrl.Ts = 0.005;
% Phase 07 G1 froze the restoring polarity and conservative scalar amplitude
% limit. G2 remains strict identity until a later evidence gate approves gains.
ctrl.differentialDriftStabilizer = struct( ...
    "enabled", false, ...
    "Kxi", 0, ...
    "Kd", 0, ...
    "polarity", 1, ...
    "amplitudeLimit", 0.60089315546799282, ...
    "rateLimit", 0.60089315546799282/ctrl.Ts, ...
    "Ts", ctrl.Ts);
% Legacy plant-side anti-split actuator. The paper-equation upper model and
% the aligned WBC wheel-acceleration feedforward make this outer correction
% unnecessary; keep it available only for explicit comparison studies.
ctrl.differentialLegForceStabilizer = struct( ...
    "enabled", false, ...
    "Kxi", 1000, ...
    "Kd", 100, ...
    "polarity", 1, ...
    "amplitudeLimit", 50, ...
    "rateLimit", 1000, ...
    "Ts", ctrl.Ts);
differentialIdentification = struct("enabled", false);
ctrl.bandwidthHz = [0.2651399877; 2.928535979; 3.753740554];
ctrl.wn = 2 * pi * ctrl.bandwidthHz;
ctrl.zeta = [0.7852988453; 0.7852988453; 0.7852988453];
ctrl.Kp = diag(ctrl.wn.^2);
ctrl.Kd = diag(2 .* ctrl.zeta .* ctrl.wn);
ctrl.commonModeBandwidthHz = [0.8; ctrl.bandwidthHz(2:3)];
ctrl.commonModeZeta = [1.0; ctrl.zeta(2:3)];
ctrl.commonModeWn = 2 * pi * ctrl.commonModeBandwidthHz;
ctrl.commonModeKp = diag(ctrl.commonModeWn.^2);
ctrl.commonModeKd = diag(2 .* ctrl.commonModeZeta .* ctrl.commonModeWn);
ctrl.differentialModeKp = ctrl.commonModeKp;
ctrl.differentialModeKd = ctrl.commonModeKd;
ctrl.tauMax = [160; 160; 45];
ctrl.tauSign = [1; 1; 1];
ctrl.constraintDamping = 1e-9;
% The analytic rolling constraint is rigid, while the Simscape contact is
% compliant. A high Baumgarte velocity gain makes the two contact models
% fight and excites pitch during velocity reversals.
ctrl.constraintVelocityGain = 5;
ctrl.qpWbaseQdd = 1e-3 * [1; 1; 1];
% Full spatial Weighted WBC. Level 0 is enforced by hard dynamics, contact,
% friction, and actuator constraints. Levels 1--4 use dimensionless task
% priorities after each residual is divided by its physical scale.
% Level 1: preserve independent left/right NMPC interaction-wrench tracking.
ctrl.spatialQpCommonWrenchScale = [140; 100; 140; 100; 160; 100];
ctrl.spatialQpDifferentialWrenchScale = ...
    ctrl.spatialQpCommonWrenchScale;
ctrl.spatialQpWrenchPenalty = 1e5;
% Level 2: soft contact residuals [rolling; lateral; normal]. Lateral uses
% the smallest scale, so it remains the strongest contact direction without
% restoring a rigid no-slip acceleration constraint.
ctrl.spatialQpContactAccelScale = [5; 2; 5];
% Large-yaw tests showed that the former [50;20;50] setting let the QP
% satisfy its speed task with accelerations that violated rolling/normal
% contact and therefore were not realized by the compliant Simscape plant.
% The rolling channel is normalized to the same effective weight as the
% speed task (25000/5^2 = 1000); lateral and normal remain soft at ten times
% their original settings so compliant-contact motion is not overconstrained.
ctrl.spatialQpContactAccelWeight = [25000; 200; 500];
% Flat tests keep the world-horizontal contact basis. Terrain studies can
% replace this with their known piecewise surface tangent without changing
% the accepted source.slx plant.
ctrl.terrainContactMap = struct("enabled", false);
% Legacy reduced/planar-QP acceleration regularization. The spatial WBC uses
% directional soft-contact tasks and does not impose rigid rolling kinematics.
ctrl.qpWqdd = [1; 1; 0.01];
% With no differential posture reserve, keep the common hip/knee near their
% reference instead of allowing wrench tracking to wind the leg repeatedly.
ctrl.commonModeQpWqdd = [100; 100; 0.01];
ctrl.differentialModeQpWqdd = ctrl.commonModeQpWqdd;
ctrl.differentialModeQpWqdd(3) = 0;
% Level 3: spatial leg posture and wheel-position tasks. The common wheel
% reference is shared by both sides; no steering differential is introduced.
ctrl.spatialQpLegAccelScale = [20; 20; 50];
ctrl.spatialQpCommonLegTaskWeight = [5; 5; 0.1];
ctrl.spatialQpDifferentialLegTaskWeight = [1; 1; 0];
% In the coupled QP the body wrench is tracked through the floating-base
% dynamics, so all actuator torques remain regularization terms.
ctrl.qpWtau = 1e-5 * [1; 1; 1];
ctrl.qpWFc = [0.0002433157215; 0.0002433157215];
% The two compliant Simscape contacts need a stronger differential-force
% regularizer than the rigid-contact QP model.  This suppresses the
% unobservable left/right load-sharing mode without hard-constraining it.
ctrl.differentialModeQpWFc = 10 * ones(2, 1);
ctrl.commonWheelPositionBandwidthHz = 0.5;
ctrl.commonWheelPositionKp = ...
    (2*pi*ctrl.commonWheelPositionBandwidthHz)^2;
ctrl.commonWheelPositionKd = ...
    2*(2*pi*ctrl.commonWheelPositionBandwidthHz);
ctrl.differentialWheelPositionBandwidthHz = 0.5;
ctrl.differentialWheelPositionKp = ...
    (2*pi*ctrl.differentialWheelPositionBandwidthHz)^2;
ctrl.differentialWheelPositionKd = ...
    2*(2*pi*ctrl.differentialWheelPositionBandwidthHz);
% Make the WBC wheel task realize the accelerations predicted by the
% paper-equation upper model, then apply its position/velocity feedback.
ctrl.paperWheelAccelerationFeedforwardEnabled = true;
ctrl.spatialQpWheelAccelScale = [5; 5];
% Keep the existing Level-2 wheel-configuration task, but give its
% differential xi_delta=0 channel enough authority for sustained turns.
ctrl.spatialQpWheelConfigurationWeight = [5; 500];
% Stage-1 common rolling-speed stabilization. The QP task acts on body
% forward acceleration, while the bounded common-force feedback removes the
% long-horizon longitudinal drift without changing the yaw-producing force
% difference between the two sides.
ctrl.commonRollingSpeedTracker = struct( ...
    "enabled", true, ...
    "Kp", 40, ...
    "accelerationMax", 2.0, ...
    "taskWeight", 1000, ...
    "accelerationScale", 1.0, ...
    "forceFeedbackEnabled", true, ...
    "forceOverrideEnabled", false);
% Low-level drift arrest for the two floating-base coordinates that are
% otherwise only regularized, not feedback tracked, by the WBC.
ctrl.baseHeightPitchTracker = struct( ...
    "enabled", true, ...
    "heightKp", 20, ...
    "heightKd", 8, ...
    "heightAccelerationMax", 2, ...
    "pitchKp", 40, ...
    "pitchKd", 8, ...
    "pitchAccelerationMax", 6, ...
    "taskWeight", [1000; 1000], ...
    "accelerationScale", [1; 3]);
% The legacy planar QP keeps its direct xi-difference task disabled. The
% spatial WBC uses the normalized low-bandwidth task above instead.
ctrl.differentialWheelPositionQpWeight = 0;
% Level 4: zero-centered acceleration, torque, and contact-force
% regularization. These select among otherwise comparable solutions and do
% not track base attitude or duplicate the NMPC loop.
ctrl.spatialQpBaseAccelScale = [10; 10; 10; 20; 20; 20];
ctrl.spatialQpBaseAccelRegularizationWeight = 0.1*ones(6, 1);
ctrl.spatialQpTorqueScale = ctrl.tauMax;
ctrl.spatialQpTorqueRegularizationWeight = 0.1*ones(3, 1);
ctrl.spatialQpContactForceScale = [140; 140; 160];
ctrl.spatialQpCommonContactForceRegularizationWeight = 0.1*ones(3, 1);
ctrl.spatialQpDifferentialContactForceRegularizationWeight = ones(3, 1);
% Observer-only WBC soft-task attribution. When enabled by a calibration
% study, the QP computes the constrained KKT sensitivity of common-Fx slack
% to uniform log-weight perturbations of each soft-task group. It is off in
% the deployed baseline and does not alter the QP objective or constraints.
ctrl.wbcTaskAttributionEnabled = false;
% Default-off pairwise hierarchy proof-of-concept. Stage 1 removes only the
% common-Fx common-mode slack penalty while retaining every other baseline
% objective. Stage 2 restores the complete baseline objective and locks the
% rolling acceleration achieved by Stage 1. This changes only the priority
% relation between rolling acceleration and common-Fx wrench realization.
ctrl.wbcRollingFxHierarchyEnabled = false;
ctrl.qpSlackScale = [140; 140; 160];
ctrl.qpWslack = 1e9 * [1; 1; 1];
% Strict common mode has no differential posture to absorb a reversed body
% moment, so limit the allowed pitch-wrench tracking error explicitly.
ctrl.commonModeMomentSlackMax = 1.0;
ctrl.commonModeContactStiffness = 4e4;
ctrl.commonModeContactDamping = 300;
ctrl.commonModeContactPreload = robotMass * leg.g ...
    / ctrl.commonModeContactStiffness;
% Per-physical-leg viscous damping. The strict common model represents two
% identical joints with one Simscape joint, so its configured coefficient is
% twice this vector. The common QP adds the same damping to h(q,dq).
ctrl.commonModeJointDamping = [1.00; 0.70; 0.04];
ctrl.qpWarmStart = true;
ctrl.qpSolver = "quadprog";
ctrl.kneeGuardEnabled = true;
ctrl.kneeGuardMin = deg2rad(10);
ctrl.kneeGuardFrequencyHz = 3.0;
ctrl.kneeGuardDamping = 1.0;
% Match the Simscape Spatial Contact Force block conservatively
% (static friction = 0.5, dynamic friction = 0.3 in source.slx).
ctrl.mu = 0.45;
% Maps base pitch into the absolute thigh angle used by the analytic leg
% dynamics: qh_abs = qh_rel + ctrl.basePitchToAbsHipSign * thetaB.
ctrl.basePitchToAbsHipSign = 1;
ctrl.discreteExecution = true;

leg.M = @(q) wheel_leg_dynamics(q, zeros(3, 1), leg, "M");
leg.C = @(q, dq) wheel_leg_dynamics(q, dq, leg, "C");
leg.G = @(q) wheel_leg_dynamics(q, zeros(3, 1), leg, "G");
leg.dynamics = @(q, dq) wheel_leg_dynamics(q, dq, leg);
leg.kinematics = @(q, dq, ddq) wheel_leg_kinematics(q, dq, ddq, leg);

base = struct();
base.g = leg.g;
base.body = struct();
base.body.mass = baseBodyMass;
base.body.lengthX = 0.45;
base.body.widthY = 0.45;
base.body.heightZ = 0.32;
base.body.comPositionBody = [0; 0];
% The reduced x-z-pitch model projects both hips onto the body CoM. The
% physical Simscape model connects the left leg at Z = -0.2 m and the
% right leg at Z = +0.2 m (Rigid Transform10 and Rigid Transform5).
base.body.hipPositionBody = [0; 0];
base.body.hipPositionBodyLeft3D = [0; 0; -0.2];
base.body.hipPositionBodyRight3D = [0; 0; 0.2];
base.body.inertiaIyy = base.body.mass * ...
    (base.body.lengthX^2 + base.body.heightZ^2) / 12;
base.m = base.body.mass;
base.Iyy = base.body.inertiaIyy;
base.rHBody = base.body.hipPositionBody - base.body.comPositionBody;
base.legCount = 2;
base.symmetricLoadShare = 1 / base.legCount;
base.thetaEq = 0;
base.xEq = [0; 0; 0; 0; 0; 0];
base.xRef = [0; 0; 0; 0; 0; 0];
base.x0 = zeros(6, 1);
base.initialQuaternion = [1, 0, 0, 0];
base.initialAngularVelocity = zeros(1, 3);
legPairMass = 2 * (leg.m1 + leg.m2 + leg.mw);
lateralHipHalfSpacing = abs(base.body.hipPositionBodyLeft3D(3));
base.Iroll = base.body.mass * ...
    (base.body.heightZ^2 + base.body.widthY^2) / 12 ...
    + legPairMass*lateralHipHalfSpacing^2;
base.Iyaw = base.body.mass * ...
    (base.body.lengthX^2 + base.body.widthY^2) / 12 ...
    + legPairMass*lateralHipHalfSpacing^2;
base.Ts = ctrl.Ts;
base.controllerType = "discrete";
base.hipRef = base.xRef(1:2) + base.rHBody;
base.simscapeGroundTopY = 0.025;
base.simscapeWorldYOffset = base.simscapeGroundTopY + leg.r ...
    - (base.rHBody(2) + traj.zO0);

base.Q = diag([25, 80, 120, 8, 16, 10]);
base.R = diag([1/80^2, 1/140^2, 1/60^2]);
base.forceMax = [140; 140];
base.momentMax = ctrl.tauMax(1);
base.thetaIntegralGain = 80;
base.thetaIntegralLimit = 0.5;
% Horizontal constant-speed comparison: forward, stop, then reverse home.
base.trajectory = struct();
base.trajectory.enabled = true;
base.trajectory.mode = "stand";
base.trajectory.settleTime = 1.0;
base.trajectory.cruiseVelocity = 0.5;
% Strict common mode needs the bounded 0.5 m/s^2 velocity transition used
% by the validated Simulink case. Keep startup-direct and scripted runs
% identical when trajectory.mode is changed to "velocity" above.
base.trajectory.accelDuration = 1.0;
base.trajectory.cruiseDuration = 1.5;
base.trajectory.decelDuration = 1.0;
base.trajectory.turnHoldDuration = 0.5;
% Steering is disabled for every accepted baseline unless a turning case
% explicitly enables it. xiL/xiR remain a common leg-configuration target;
% differential rolling is produced by the NMPC left/right wrench split.
base.trajectory.turning = struct( ...
    "enabled", false, ...
    "mode", "single", ...
    "yaw0", 0, ...
    "yawRate", 0, ...
    "startTime", 2.0, ...
    "rampDuration", 0.5, ...
    "holdDuration", 1.0, ...
    "zeroHoldDuration", 0.5, ...
    "minimumSpeed", 0.02);
base.trajectory.crouchDepth = 0;
base.trajectory.crouchDownDuration = base.trajectory.settleTime;
base.trajectory.crouchRecoverStart = 6.5;
base.trajectory.crouchRecoverDuration = 1.0;
if string(base.trajectory.mode) == "velocity"
    assert(base.trajectory.accelDuration >= 1.0 ...
        && base.trajectory.decelDuration >= 1.0, ...
        "The validated velocity setup requires at least 1 s acceleration " + ...
        "and deceleration ramps.");
end

baseLqr = floating_base_lqr_design(base);
base.command = @(x) floating_base_lqr_command(x, baseLqr);
wheelLqr = wheel_position_lqr_design(base, leg, traj);
% The accepted model uses a bounded second-order governor.  Disable only in
% the dedicated paper Eq. (21) A/B to expose the raw planned wheel position.
wheelLqr.governorEnabled = true;

% Planar base-plus-wheel-position NMPC configuration. Solver generation is
% deliberately kept out
% of startup; use build_base_nmpc_solver when the generated S-Function is absent.
baseNmpc = struct();
baseNmpc.enabled = true;
% The generated solver takes about 4--9 ms on the current host.  Run the
% upper NMPC at 100 Hz and retain a 0.30 s prediction horizon; faster lower
% QP and plant rates remain independent of this supervisory sample time.
baseNmpc.Ts = 0.01;
baseNmpc.N = 30;
baseNmpc.Q = blkdiag(base.Q, 50, 5);
baseNmpc.R1 = diag([0.02, 0.01, 0.02]);
baseNmpc.R2 = diag([0.20, 0.10, 0.20]);
baseNmpc.model = base_wheel_state_space(base, leg, traj);
% With the planar hip projection at the CoM, xB/theta/xi have two wrench
% channels (Fx and My) and therefore one neutral uncontrollable combination.
% A finite-horizon NMPC remains valid, but an infinite-horizon DARE terminal
% cost does not. Use the finite state weight as the terminal weight.
baseNmpc.W_e = baseNmpc.Q;
baseNmpc.uMin = [-80; 0; -40];
baseNmpc.uMax = [ 80; 100; 40];
baseNmpc.driveCoefficient = ctrl.mu;
baseNmpc.xiMin = wheelLqr.positionMin;
baseNmpc.xiMax = wheelLqr.positionMax;
% The measured relative speed contains a short contact-settling transient;
% keep the planner reference conservative without making the OCP infeasible.
baseNmpc.dxiMax = 2.0;
baseNmpc.maxSolveTime = baseNmpc.Ts;
baseNmpc.solverName = "base_wheel_8state_nmpc";
baseNmpc.sfunName = "acados_solver_sfunction_" + baseNmpc.solverName;
tsTag = replace(string(sprintf("%.9g", baseNmpc.Ts)), ...
    [".", "-", "+"], ["p", "m", ""]);
baseNmpc.buildTag = "Ts_" + tsTag + "_N_" + string(baseNmpc.N) + ...
    "_paper_common_v2";
baseNmpc.generatedDir = fullfile(simulateDir, "generated", ...
    baseNmpc.solverName, baseNmpc.buildTag);
baseNmpc.referenceSize = 14*baseNmpc.N + 8;
baseNmpc.available = isfile(fullfile(baseNmpc.generatedDir, ...
    baseNmpc.sfunName + "." + mexext));
if baseNmpc.available
    addpath(baseNmpc.generatedDir);
end
% Full 8-DoF upper model from the 3D base-wheel-position note: six base
% coordinates plus independent left/right relative wheel positions produce
% a 16-state first-order model. Keep the paper's 12D per-side interaction
% wrench input. Force components are body-aligned and include lateral force
% for curved world motion; unavailable direct roll/yaw torque channels stay
% fixed to zero through input bounds.
fullBaseNmpc = baseNmpc;
fullBaseNmpc.variant = "full8dof";
% The 16-state/12-input SQP-RTI problem is larger than the planar one. Run
% this full supervisor at 50 Hz with a 0.40 s horizon; the measured solve
% time then remains comfortably inside its 20 ms deadline.
fullBaseNmpc.Ts = 0.02;
fullBaseNmpc.N = 20;
fullBaseNmpc.maxSolveTime = fullBaseNmpc.Ts;
fullBaseNmpc.model = full_base_wheel_state_space(base, leg, traj);
fullBaseNmpc.Q = diag([25, 10, 80, 200, 120, 250, ...
    80, 20, 16, 20, 10, 80, 5, 5, 0.5, 0.5]);
activeInputWeight = [0.04, 0.04, 0.02, 1, 0.04, 1];
fullBaseNmpc.R1 = diag([activeInputWeight, activeInputWeight]);
activeIncrementWeight = [0.40, 0.40, 0.20, 1, 0.40, 1];
fullBaseNmpc.R2 = diag([activeIncrementWeight, activeIncrementWeight]);
fullBaseNmpc.incrementCostMode = "previous_applied_anchor";
fullBaseNmpc.ocpStateSize = 16;
fullBaseNmpc.W_e = fullBaseNmpc.Q;
perSideMin = [-40; 0; 0; 0; -20; 0];
perSideMax = [ 40; 0; 70; 0;  20; 0];
fullBaseNmpc.uMin = repmat(perSideMin, 2, 1);
fullBaseNmpc.uMax = repmat(perSideMax, 2, 1);
fullBaseNmpc.solverName = "full_base_wheel_16state_nmpc";
fullBaseNmpc.sfunName = "acados_solver_sfunction_" ...
    + fullBaseNmpc.solverName;
fullBaseNmpc.buildTag = "paper_eq12_v1";
fullBaseNmpc.generatedDir = fullfile(simulateDir, "generated", ...
    fullBaseNmpc.buildTag);
fullBaseNmpc.referenceSize = 40*fullBaseNmpc.N + 16;
fullBaseNmpc.available = isfile(fullfile( ...
    fullBaseNmpc.generatedDir, fullBaseNmpc.sfunName + "." + mexext));
if fullBaseNmpc.available
    addpath(fullBaseNmpc.generatedDir);
end

assignin("base", "leg", leg);
assignin("base", "ctrl", ctrl);
assignin("base", "traj", traj);
assignin("base", "base", base);
assignin("base", "baseLqr", baseLqr);
assignin("base", "wheelLqr", wheelLqr);
assignin("base", "baseNmpc", baseNmpc);
assignin("base", "fullBaseNmpc", fullBaseNmpc);
assignin("base", "differentialIdentification", differentialIdentification);

[base, leg, baseLqr] = set_initial_base_state(base.x0);

if bdIsLoaded("source")
    suppress_scope_windows("source");
end

fprintf("Loaded symmetric two-wheel-leg parameters into base workspace.\n");
fprintf("Initial q0 = [%.4f; %.4f; %.4f] rad.\n", ...
    leg.q0(1), leg.q0(2), leg.q0(3));
fprintf("Initial dq0 = [%.4f; %.4f; %.4f] rad/s.\n", ...
    leg.dq0(1), leg.dq0(2), leg.dq0(3));
fprintf("Floating base body: %.2f x %.2f x %.2f m, m = %.2f kg, Iyy = %.4f kg*m^2.\n", ...
    base.body.lengthX, base.body.widthY, base.body.heightZ, ...
    base.body.mass, base.body.inertiaIyy);
fprintf("Planar hip projection: [%.4f; %.4f] m; lateral hips: +/-%.4f m.\n", ...
    base.rHBody(1), base.rHBody(2), ...
    abs(base.body.hipPositionBodyLeft3D(3)));
fprintf("Upper NMPC equilibrium [FHx; FHz; MBy] = [%.4f; %.4f; %.4f].\n", ...
    baseNmpc.model.uEq(1), baseNmpc.model.uEq(2), baseNmpc.model.uEq(3));
fprintf("Wheel-position planner: %s, scheduled height %.3f to %.3f m.\n", ...
    traj.wheelPositionPlanner, wheelLqr.heightGrid(1), wheelLqr.heightGrid(end));
if baseNmpc.available
    fprintf("Upper-layer 8-state NMPC S-Function ready, Ts = %.4f s, N = %d.\n", ...
        baseNmpc.Ts, baseNmpc.N);
else
    fprintf("Direct upper-layer NMPC S-Function is not built. Run " + ...
        "build_base_nmpc_solver(true).\n");
end
if fullBaseNmpc.available
    fprintf("Full 16-state/12-input NMPC S-Function ready, Ts = %.4f s, N = %d.\n", ...
        fullBaseNmpc.Ts, fullBaseNmpc.N);
else
    fprintf("Full 16-state/12-input NMPC S-Function is not built.\n");
end
