function [outputs, widths] = coupled_two_leg_qp_demux_outputs()
%COUPLED_TWO_LEG_QP_DEMUX_OUTPUTS Current append-only QP diagnostic split.

% Legacy (01-85):
%   3 tauLeft | 3 tauRight | 12 wrenchSlack | 12 wrenchFeasible |
%   1 wrenchSlackNorm | 1 qpFeasible | 3 FcLeft | 3 FcRight |
%   1 exitflag | 3 residual norms | 6 qddBase | 3 contactResDir |
%   3 common-wheel tracking values | 3 tauDiff |
%   3 common-wheel acceleration values | 4 friction margins |
%   6 torque margins | 4 differential-wheel values |
%   11 Phase-07 scalar diagnostics.
% Phase 08 appends 4 scalar coordinates, 4x12 projected vectors,
% 4 residual RMS values, 2 flags, 3x16 task vectors, 2 attribution
% diagnostics, and 5 pairwise-hierarchy diagnostics.
widths = [3, 3, 12, 12, 1, 1, 3, 3, 1, 1, 1, 1, 6, 3, 3, 3, 3, ...
    4, 6, ones(1, 15), ones(1, 4), 12, 12, 12, 12, ones(1, 6), ...
    16, 16, 16, 1, 1, ones(1, 5)];
contract = coupled_two_leg_qp_signal_contract();
assert(sum(widths) == contract.width && numel(widths) == 58, ...
    "coupled_two_leg_qp_demux_outputs:ContractMismatch", ...
    "The QP Demux must expose 58 ports and match the signal contract.");
outputs = char("[" + strjoin(string(widths), " ") + "]");
end
