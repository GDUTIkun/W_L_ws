function result = test_coordinate_frame_contract()
%TEST_COORDINATE_FRAME_CONTRACT Verify frozen Phase-02 algebraic mappings.
% Joint signs and real IMU installation are deliberately not claimed.

toolDir = fileparts(mfilename("fullpath"));
repoRoot = fileparts(fileparts(toolDir));
baselineModelDir = fullfile(repoRoot, "simulation", ...
    "simulink_baseline", "model", "simulate", "two_legs");
addpath(baselineModelDir, "-begin");
tolerance = 1e-12;

% Simscape S=[forward,up,right] -> canonical N=[forward,left,up].
R_N_from_S = [1, 0, 0; 0, 0, -1; 0, 1, 0];
assertNear(R_N_from_S.'*R_N_from_S, eye(3), tolerance, ...
    "Simscape-to-canonical mapping is not orthonormal.");
assertNear(det(R_N_from_S), 1, tolerance, ...
    "Simscape-to-canonical mapping must be a proper rotation.");
assertNear(R_N_from_S.'*(R_N_from_S*[0.3; -0.2; 0.7]), ...
    [0.3; -0.2; 0.7], tolerance, ...
    "Simscape-to-canonical mapping failed round-trip.");

% Legacy Controller translation fields serialize [Sx,Sz,Sy].
P_Cfields_from_S = [1, 0, 0; 0, 0, 1; 0, 1, 0];
P_Cfields_from_N = P_Cfields_from_S*R_N_from_S.';
assertNear(P_Cfields_from_S*P_Cfields_from_S, eye(3), tolerance, ...
    "Controller field pack/unpack is not self-inverse in Simscape order.");
assertNear(P_Cfields_from_N, diag([1, -1, 1]), tolerance, ...
    "Canonical FLU to legacy Controller field mapping changed.");
assertNear(det(P_Cfields_from_N), -1, tolerance, ...
    "Controller field ordering must remain an improper field pack.");

% Current MuJoCo world is already canonical FLU.
R_N_from_M = eye(3);
assertNear(det(R_N_from_M), 1, tolerance, ...
    "MuJoCo-to-canonical mapping must be a proper rotation.");

% Positive baseline yaw: -S_z = +N_y (left) = negative legacy right.
yaw = 0.2;
R_S_from_B = controller_attitude_kinematics([0; 0; yaw]);
forwardInS = R_S_from_B*[1; 0; 0];
forwardInN = R_N_from_S*forwardInS;
forwardFields = P_Cfields_from_S*forwardInS;
assertNear(forwardInS, [cos(yaw); 0; -sin(yaw)], tolerance, ...
    "Baseline positive-yaw direction changed.");
assertNear(forwardInN, [cos(yaw); sin(yaw); 0], tolerance, ...
    "Positive yaw must be a left turn in canonical FLU.");
assert(forwardFields(2) < 0, ...
    "Positive yaw must turn toward negative legacy right-lateral field.");

% A wrench requires both the proper frame rotation and the moment arm.
forceS = [12.0; -4.0; 7.0];
torqueAtOS = [0.8; -1.2; 0.3];
rPtoOS = [0.11; -0.06; 0.04];
forceN = R_N_from_S*forceS;
torqueAtON = R_N_from_S*torqueAtOS;
rPtoON = R_N_from_S*rPtoOS;
torqueAtPN = torqueAtON + cross(rPtoON, forceN);
recoveredForceS = R_N_from_S.'*forceN;
recoveredTorqueAtOS = R_N_from_S.'*( ...
    torqueAtPN - cross(rPtoON, forceN));
assertNear(recoveredForceS, forceS, tolerance, ...
    "Force frame round-trip failed.");
assertNear(recoveredTorqueAtOS, torqueAtOS, tolerance, ...
    "Wrench frame/origin round-trip failed.");

result = struct();
result.passed = true;
result.simscapeToCanonicalDeterminant = det(R_N_from_S);
result.controllerFieldPackDeterminant = det(P_Cfields_from_N);
result.mujocoToCanonicalDeterminant = det(R_N_from_M);
result.positiveYawForwardVectorS = forwardInS;
result.positiveYawForwardVectorN = forwardInN;
result.positiveYawForwardVectorControllerFields = forwardFields;
result.wrenchRoundTripPassed = true;
result.dynamicGatesCovered = false;

fprintf("Coordinate frame algebraic contract: PASS\n");
fprintf("  det(R_N_from_S) = %.0f (proper rotation)\n", ...
    result.simscapeToCanonicalDeterminant);
fprintf("  det(P_Cfields_from_N) = %.0f (field pack, not rotation)\n", ...
    result.controllerFieldPackDeterminant);
fprintf("  det(R_N_from_M) = %.0f (MuJoCo world already FLU)\n", ...
    result.mujocoToCanonicalDeterminant);
fprintf("  positive yaw forward in N = [%.6f %.6f %.6f]\n", ...
    forwardInN);
fprintf("  wrench rotation + origin shift round-trip: PASS\n");
fprintf("  Joint zero offsets and real-IMU installation are transferred.\n");
end

function assertNear(actual, expected, tolerance, message)
if any(abs(actual(:) - expected(:)) > tolerance)
    error("test_coordinate_frame_contract:AssertionFailed", "%s", message);
end
end
