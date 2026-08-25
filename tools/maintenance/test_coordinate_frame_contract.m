function result = test_coordinate_frame_contract()
%TEST_COORDINATE_FRAME_CONTRACT Verify frozen Phase-02 algebraic mappings.
%
% This test covers only decisions already supported by the Simulink model
% and static MJCF audit. It deliberately does not claim that the candidate
% MuJoCo X direction, joint signs or IMU semantics have passed dynamics.

toolDir = fileparts(mfilename("fullpath"));
repoRoot = fileparts(fileparts(toolDir));
baselineModelDir = fullfile(repoRoot, "simulation", ...
    "simulink_baseline", "model", "simulate", "two_legs");
addpath(baselineModelDir, "-begin");

tolerance = 1e-12;

% Controller translation fields serialize [Sx, Sz, Sy].
P_Cfields_from_S = [1, 0, 0; 0, 0, 1; 0, 1, 0];
assertNear(P_Cfields_from_S*P_Cfields_from_S, eye(3), tolerance, ...
    "Controller field pack/unpack is not self-inverse.");
assertNear(det(P_Cfields_from_S), -1, tolerance, ...
    "Controller field ordering must remain an improper permutation.");

% Candidate native MuJoCo FLU -> canonical Simscape forward-up-right.
R_S_from_M = [1, 0, 0; 0, 0, 1; 0, -1, 0];
assertNear(R_S_from_M.'*R_S_from_M, eye(3), tolerance, ...
    "Candidate MuJoCo mapping is not orthonormal.");
assertNear(det(R_S_from_M), 1, tolerance, ...
    "Candidate MuJoCo mapping must be a proper rotation.");
assertNear(R_S_from_M.'*(R_S_from_M*[0.3; -0.2; 0.7]), ...
    [0.3; -0.2; 0.7], tolerance, ...
    "Candidate MuJoCo mapping failed round-trip.");

% Baseline positive yaw rotates forward toward physical -Z (left), which
% is negative Controller right-lateral field.
yaw = 0.2;
R_S_from_B = controller_attitude_kinematics([0; 0; yaw]);
forwardInS = R_S_from_B*[1; 0; 0];
assertNear(forwardInS, [cos(yaw); 0; -sin(yaw)], tolerance, ...
    "Baseline positive-yaw direction changed.");
forwardFields = P_Cfields_from_S*forwardInS;
assert(forwardFields(2) < 0, ...
    "Positive yaw must turn toward negative Controller lateral field.");

result = struct();
result.passed = true;
result.controllerFieldPermutationDeterminant = det(P_Cfields_from_S);
result.candidateMujocoRotationDeterminant = det(R_S_from_M);
result.positiveYawForwardVectorS = forwardInS;
result.positiveYawForwardVectorControllerFields = forwardFields;
result.dynamicGatesCovered = false;

fprintf("Coordinate frame algebraic contract: PASS\n");
fprintf("  det(P_Cfields_from_S) = %.0f (field order, not rotation)\n", ...
    result.controllerFieldPermutationDeterminant);
fprintf("  det(R_S_from_M) = %.0f (candidate proper rotation)\n", ...
    result.candidateMujocoRotationDeterminant);
fprintf("  positive yaw forward in S = [%.6f %.6f %.6f]\n", ...
    forwardInS);
fprintf("  Dynamic MuJoCo/joint/IMU gates remain OPEN.\n");
end

function assertNear(actual, expected, tolerance, message)
if any(abs(actual(:) - expected(:)) > tolerance)
    error("test_coordinate_frame_contract:AssertionFailed", "%s", message);
end
end
