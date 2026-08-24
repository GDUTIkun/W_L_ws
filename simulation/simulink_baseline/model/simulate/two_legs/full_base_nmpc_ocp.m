function ocp = full_base_nmpc_ocp(base, leg, fullBaseNmpc)
%FULL_BASE_NMPC_OCP Build the 16-state, 12-input 8-DoF acados OCP.

arguments
    base (1, 1) struct
    leg (1, 1) struct
    fullBaseNmpc (1, 1) struct
end
if ~isfield(base, "g") || abs(base.g - fullBaseNmpc.model.g) > 1e-12
    error("full_base_nmpc_ocp:GravityMismatch", ...
        "Base and upper-model gravity must match.");
end

import casadi.*

incrementCostMode = "previous_applied_anchor";
if isfield(fullBaseNmpc, "incrementCostMode")
    incrementCostMode = string(fullBaseNmpc.incrementCostMode);
end
exactDeltaInput = incrementCostMode == "state_memory";
if exactDeltaInput
    x = SX.sym('x', 28, 1);
    xPhysical = x(1:16);
    uPrevious = x(17:28);
else
    x = SX.sym('x', 16, 1);
    xPhysical = x;
    uPrevious = SX.zeros(12, 1);
end
u = SX.sym('u', 12, 1);
fExpl = full_base_body_dynamics( ...
    xPhysical, u, fullBaseNmpc.model, leg);

model = AcadosModel();
if exactDeltaInput
    model.name = 'fb28du';
else
    model.name = 'full_base_two_wheel_leg_body';
end
model.x = x;
model.u = u;
if exactDeltaInput
    model.disc_dyn_expr = vertcat(rk4Step( ...
        xPhysical, u, fullBaseNmpc.model, leg, fullBaseNmpc.Ts), u);
else
    xdot = SX.sym('xdot', 16, 1);
    model.xdot = xdot;
    model.f_expl_expr = fExpl;
    model.f_impl_expr = xdot - fExpl;
end
mu = fullBaseNmpc.driveCoefficient/sqrt(2);
model.con_h_expr = [
    u(1) - mu*u(3); -u(1) - mu*u(3);
    u(2) - mu*u(3); -u(2) - mu*u(3);
    u(7) - mu*u(9); -u(7) - mu*u(9);
    u(8) - mu*u(9); -u(8) - mu*u(9)
];

if exactDeltaInput
    stageCost = vertcat(xPhysical, u, u - uPrevious);
else
    stageCost = vertcat(xPhysical, u, u);
end
W = blkdiag(fullBaseNmpc.Q, fullBaseNmpc.R1, fullBaseNmpc.R2);
ocp = AcadosOcp();
ocp.name = char(fullBaseNmpc.solverName);
ocp.model = model;
ocp.cost.cost_type_0 = 'NONLINEAR_LS';
ocp.model.cost_y_expr_0 = stageCost;
ocp.cost.W_0 = W;
ocp.cost.yref_0 = zeros(40, 1);
ocp.cost.cost_type = 'NONLINEAR_LS';
ocp.model.cost_y_expr = stageCost;
ocp.cost.W = W;
ocp.cost.yref = zeros(40, 1);
ocp.cost.cost_type_e = 'NONLINEAR_LS';
ocp.model.cost_y_expr_e = xPhysical;
ocp.cost.W_e = fullBaseNmpc.W_e;
ocp.cost.yref_e = zeros(16, 1);

ocp.constraints.idxbu = (0:11).';
ocp.constraints.lbu = fullBaseNmpc.uMin(:);
ocp.constraints.ubu = fullBaseNmpc.uMax(:);
ocp.constraints.lh = -1e9*ones(8, 1);
ocp.constraints.uh = zeros(8, 1);
ocp.constraints.idxbx = (12:15).';
ocp.constraints.lbx = [repmat(fullBaseNmpc.xiMin, 2, 1); ...
    repmat(-fullBaseNmpc.dxiMax, 2, 1)];
ocp.constraints.ubx = [repmat(fullBaseNmpc.xiMax, 2, 1); ...
    repmat(fullBaseNmpc.dxiMax, 2, 1)];
ocp.constraints.idxbx_e = ocp.constraints.idxbx;
ocp.constraints.lbx_e = ocp.constraints.lbx;
ocp.constraints.ubx_e = ocp.constraints.ubx;
if exactDeltaInput
    ocp.constraints.x0 = [fullBaseNmpc.model.xEq(:); ...
        fullBaseNmpc.model.uEq(:)];
else
    ocp.constraints.x0 = fullBaseNmpc.model.xEq(:);
end

ocp.solver_options.N_horizon = fullBaseNmpc.N;
ocp.solver_options.tf = fullBaseNmpc.N*fullBaseNmpc.Ts;
ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM';
ocp.solver_options.hessian_approx = 'GAUSS_NEWTON';
if exactDeltaInput
    ocp.solver_options.integrator_type = 'DISCRETE';
else
    ocp.solver_options.integrator_type = 'ERK';
end
ocp.solver_options.nlp_solver_type = 'SQP_RTI';

simulinkOpts = AcadosOcpSimulinkOptions();
simulinkOpts.inputs = setAllFields(simulinkOpts.inputs, 0);
simulinkOpts.inputs.lbx_0 = 1;
simulinkOpts.inputs.ubx_0 = 1;
simulinkOpts.inputs.y_ref_0 = 1;
simulinkOpts.inputs.y_ref = 1;
simulinkOpts.inputs.y_ref_e = 1;
simulinkOpts.outputs = setAllFields(simulinkOpts.outputs, 0);
simulinkOpts.outputs.u0 = 1;
simulinkOpts.outputs.solver_status = 1;
simulinkOpts.outputs.CPU_time = 1;
simulinkOpts.samplingtime = 't0';
simulinkOpts.show_port_info = 1;
ocp.simulink_opts = simulinkOpts;
ocp.code_gen_options.code_export_directory = char(fullBaseNmpc.generatedDir);
end

function next = rk4Step(x, u, model, leg, dt)
k1 = full_base_body_dynamics(x, u, model, leg);
k2 = full_base_body_dynamics(x + 0.5*dt*k1, u, model, leg);
k3 = full_base_body_dynamics(x + 0.5*dt*k2, u, model, leg);
k4 = full_base_body_dynamics(x + dt*k3, u, model, leg);
next = x + dt*(k1 + 2*k2 + 2*k3 + k4)/6;
end

function obj = setAllFields(obj, value)
names = properties(obj);
for i = 1:numel(names)
    obj.(names{i}) = value;
end
end
