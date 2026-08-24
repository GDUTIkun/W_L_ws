function contract = coupled_two_leg_qp_signal_contract()
%COUPLED_TWO_LEG_QP_SIGNAL_CONTRACT Versioned append-only QP diagnostics.

contract = struct();
contract.version = "08-04-PAIR-HQP";
contract.legacyWidth = 85;
contract.width = 198;
contract.appendedNames = [ ...
    "uDiffScalarRequested"
    "uDiffScalarApplied"
    "uDiffScalarFinal"
    "uDiffScalarQpFeasible"
    "uDiffProjectedRequested"
    "uDiffProjectedApplied"
    "uDiffProjectedFinal"
    "uDiffProjectedQpFeasible"
    "uDiffResidualRmsRequested"
    "uDiffResidualRmsApplied"
    "uDiffResidualRmsFinal"
    "uDiffResidualRmsQpFeasible"
    "qpFeasibleControllerSide"
    "plantWrenchUnavailable"
    "wbcTaskSensitivity"
    "wbcTaskCost"
    "wbcTaskResidual"
    "wbcTaskAttributionValid"
    "wbcTaskGradientClosure"
    "wbcRollingFxHierarchyRequested"
    "wbcRollingFxHierarchyApplied"
    "wbcRollingFxHierarchyStage1Feasible"
    "wbcRollingFxHierarchyStage1RollingAcceleration"
    "wbcRollingFxHierarchyLockResidual"
];
contract.appendedIndices = (86:contract.width).';
contract.appendedSizes = [1; 1; 1; 1; 12; 12; 12; 12; 1; 1; 1; 1; 1; 1; ...
    16; 16; 16; 1; 1; 1; 1; 1; 1; 1];
contract.attributionNames = [ ...
    "base_accel_regularization"
    "common_leg_acceleration"
    "differential_leg_acceleration"
    "torque_regularization"
    "common_contact_force_regularization"
    "differential_contact_force_regularization"
    "common_fx_slack_regularization"
    "other_wrench_slack_regularization"
    "wheel_position_common"
    "wheel_position_differential"
    "common_rolling_acceleration"
    "base_height"
    "base_pitch"
    "contact_rolling"
    "contact_lateral"
    "contact_normal"
];
end
