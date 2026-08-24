function y = common_mode_qp_signal(x)
%COMMON_MODE_QP_SIGNAL Simulink adapter for the strict common-mode QP.

[tau, debug] = coupled_two_leg_qp_core(x, "common");
y = [tau; debug.wrenchSlack; debug.wrenchFeasible; ...
    debug.wrenchSlackNorm; double(debug.qpFeasible); ...
    debug.FcLeft; debug.FcRight];
end
