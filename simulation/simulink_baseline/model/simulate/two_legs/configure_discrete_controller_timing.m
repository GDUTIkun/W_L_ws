function configure_discrete_controller_timing(doSave)
%CONFIGURE_DISCRETE_CONTROLLER_TIMING Apply symmetric two-leg timing/wiring.
%
% The two-leg configurator owns both QP input ZOH blocks, MATLAB function
% sample times, and the implicit Simscape solver settings. Keep this legacy
% entry point as a thin compatibility wrapper.

if nargin < 1 || isempty(doSave)
    doSave = true;
end
configure_symmetric_two_leg_simulink(doSave);
end
