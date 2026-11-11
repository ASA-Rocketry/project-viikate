function plant_parameters()
% Define every parameter of viikate_rocket.slx in the base workspace.
%
% Values are carried over from ../closedModel/closedparameters.m (which stays
% as it is for the frozen closedmodel), without its controller gains and
% references and without its clear and sim calls.
%
% Frames: body x along the rocket axis towards the nose. The model's
% inertial frame has x up, so gravity is -x. The C adapter maps it to NED as
% [north, east, down] = [z, y, -x].
%
% Parameters named rp_* are tunable in the generated code: they become C
% globals that code/sim/plant/rocket/rocket_plant.c sets at run time. All
% other values are fixed at generation; changing them means regenerating.

here = fileparts(mfilename('fullpath'));

% --- Run-time tunable (Simulink.Parameter, exported to C) -------------------
tunable = {
    'rp_ignition_time_s',        3.0         % time from power-on to ignition
    'rp_thrust_scale',           1.0         % multiplier on the thrust curve
    'rp_mass_full_kg',           1.225 + 0.127   % lift-off mass
    'rp_mass_fuel_kg',           0.063       % propellant mass
    'rp_wind_model_mps',         [0; 5; 5]   % mean wind, model inertial frame
    'rp_gust_variance',          2           % variance of the gust noise
    'rp_gust_seed',              [1 2]       % seeds of the two gust sources
    'rp_servo_tau_s',            0.05        % servo lag time constant
    'rp_servo_rate_limit_radps', deg2rad(600) % servo rate limit
};
for k = 1:size(tunable, 1)
    p = Simulink.Parameter(tunable{k, 2});
    p.CoderInfo.StorageClass = 'ExportedGlobal';
    assignin('base', tunable{k, 1}, p);
end

% --- Fixed at generation -----------------------------------------------------
v = struct();

% Propulsion
v.bt = 1;                                   % motor burn time, s
thrust = load(fullfile(here, '..', 'closedModel', 'thrustcurve.mat'));
f = fieldnames(thrust);
v.thrust_t_s = thrust.(f{1})(1, :);         % thrust curve time, s
v.thrust_n = thrust.(f{1})(2, :);           % thrust curve force, N

% Mass properties
v.Ie = [0.0027 0 0; 0 0.1655 0; 0 0 0.1655]; % empty inertia, kg m^2
v.If = [0.0027 0 0; 0 0.177 0; 0 0 0.177];   % full inertia, kg m^2

% Environment
v.gravity = [-9.819 0 0]';                  % inertial frame (x up)
v.rho = 1.225;                              % air density, kg/m^3

% Aerodynamics
v.Cd0 = 0.635;                              % drag coefficient
v.A = 0.00442;                              % frontal area, m^2
v.Af = 0.0016875;                           % fin area, m^2
lr = 0.03415 * 0.65;                        % centre of lift from fin root, m
fr = 0.075 / 2 + lr;                        % fin moment arm, m
Cp = -0.346;                                % centre of pressure from CG, m
dfins = 0.4689;                             % fin distance from CG, m
v.fin_positions = [dfins 0 fr; dfins -fr 0; dfins 0 -fr; dfins fr 0];
v.r_cp = [Cp 0 0]';
v.N = [0 -1 0; 0 0 -1; 0 1 0; 0 0 1];       % canard lift directions
v.si = [0 0 1; 0 -1 0; 0 0 -1; 0 1 0];      % canard span directions
v.Cla = 1.975;                              % fin lift coefficient slope
v.CNa = 20;                                 % normal force coefficient slope
v.Cmq = -40;                                % pitch damping moment slope
v.Cnr = -40;                                % yaw damping moment slope
v.Clp = -0.3;                               % roll damping moment slope
v.L = 1.27;                                 % rocket length, m
v.b = 0.14;                                 % roll damping reference length, m
v.Vsafe = 0.1;                              % low-airspeed guard, m/s

names = fieldnames(v);
for k = 1:numel(names)
    assignin('base', names{k}, v.(names{k}));
end
end
