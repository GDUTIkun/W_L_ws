function y = wheel_leg_tracking_signal(x)
%WHEEL_LEG_TRACKING_SIGNAL Simulink-friendly relative joint reference.
%
% Input:
%   x = [t;
%        xB; zB; thetaB; dxB; dzB; dthetaB;
%        qh; qk; qw; dqh; dqk; dqw;
%        FHx_ext; FHz_ext; MBy_des;
%        optional xiRef; dxiRef; ddxiRef; xiRaw]
%
% Output:
%   y = [q_rel_des; dq_rel_des]
%
% The leg dynamics reference qd_abs uses an absolute hip/thigh angle. The
% Simscape hip sensor reports the hip joint angle relative to the floating
% base, so qh_des = qh_abs_des - thetaB and dqh_des =
% dqh_abs_des - dthetaB.

x = double(x(:));
if ~ismember(numel(x), [16, 20])
    error("wheel_leg_tracking_signal:InvalidInput", ...
        "Expected the 16D legacy or 20D planned-wheel input vector.");
end

t = x(1);
thetaB = x(4);
dthetaB = x(7);

wheelReference = [];
if numel(x) >= 20
    wheelReference = x(17:19);
end
[qdAbs, dqdAbs, ~] = floating_base_leg_reference(t, x(2:7), ...
    [], [], [], [], x(14:15), false, wheelReference);

qRelDes = [qdAbs(1) - thetaB; qdAbs(2); qdAbs(3)];
dqRelDes = [dqdAbs(1) - dthetaB; dqdAbs(2); dqdAbs(3)];

y = [qRelDes; dqRelDes];
end
