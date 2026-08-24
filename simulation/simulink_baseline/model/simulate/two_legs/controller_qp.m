function tau = controller_qp(x)
%CONTROLLER_QP Small QP inverse dynamics with contact constraints.
%
% Decision variable:
%   z = [qdd(3); tau(3); Fc(2); wrenchSlack(3)]
%
% Equality constraints:
%   M*qdd - tau - Jc'*Fc - JH'*sF = JH'*FH_ext - C - G
%   Jc*qdd = -dJc*dq - aH - Kc*(Jc*dq + vH)
%   tau_h = hipMomentToTauSign*(MBy_des + sM)
%
% Inequality constraints:
%   Fcz >= 0
%   |Fcx| <= mu*Fcz
%
% Input for the maintained floating-base plant:
%   x = [t; xB; zB; thetaB; dxB; dzB; dthetaB;
%        qh; qk; qw; dqh; dqk; dqw; FHx_ext; FHz_ext; MBy_des;
%        optional xiRef; dxiRef; ddxiRef; xiRaw]
%
% FH_ext is the body-on-leg reaction force. If the upper LQR returns the
% force applied by the leg to the body, pass its negative here. MBy_des is
% the desired pure pitch moment applied by the leg to the body.

[tau, ~] = controller_qp_core(x);
end
