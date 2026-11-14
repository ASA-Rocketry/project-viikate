function plant_parameters(design)
% Define every parameter of viikate_rocket.slx in the base workspace.
%
%   plant_parameters()        the baseline design simulation/rockets/viikate_v1.rocket
%   plant_parameters(design)  path of another .rocket file
%
% Rocket values come from the design file and its .eng motor, read by
% read_rocket_design.m with the same rules as the C simulation, so Simulink
% and the SIL fly the same rocket. Format: simulation/rockets/README.md.
%
% Frames: body x along the rocket axis towards the nose. The model's
% inertial frame has x up, so gravity is -x. The C adapter maps it to NED as
% [north, east, down] = [z, y, -x].
%
% Parameters named rp_* are tunable in the generated code: they become C
% globals that code/sim/plant/rocket/rocket_plant.c sets at run time. All
% other values are fixed at generation; changing them means regenerating.

here = fileparts(mfilename('fullpath'));
if nargin < 1
    design = fullfile(here, '..', '..', 'rockets', 'viikate_v1.rocket');
end
d = read_rocket_design(design);

% --- Run-time tunable (Simulink.Parameter, exported to C) -------------------
tunable = {
    'rp_ignition_time_s',        3.0         % time from power-on to ignition
    'rp_thrust_scale',           1.0         % multiplier on the thrust curve
    'rp_mass_full_kg',           d.mass_full_kg   % lift-off mass
    'rp_mass_fuel_kg',           d.mass_fuel_kg   % propellant mass
    'rp_wind_model_mps',         [0; 5; 5]   % mean wind, model inertial frame
    'rp_gust_variance',          2           % variance of the gust noise
    'rp_gust_seed',              [1 2]       % seeds of the two gust sources
    'rp_servo_tau_s',            d.servo_tau_s    % servo lag time constant
    'rp_servo_rate_limit_radps', d.servo_rate_limit_radps % servo rate limit
    % Design values. Derived arrays are computed by read_rocket_design (and
    % by the C applier), so block parameters stay plain variable names.
    'rp_burn_time_s',            d.burn_time_s     % thrust cut-off after ignition
    'rp_thrust_t_s',             d.thrust_t_s      % thrust table time, 1 x 128
    'rp_thrust_n',               d.thrust_n        % thrust table force, 1 x 128
    'rp_inertia_full_kgm2',      d.inertia_full    % full inertia tensor
    'rp_inertia_empty_kgm2',     d.inertia_empty   % empty inertia tensor
    'rp_length_m',               d.length_m        % rocket length
    'rp_frontal_area_m2',        d.frontal_area_m2 % frontal area
    'rp_cd0',                    d.cd0             % drag coefficient
    'rp_cn_alpha',               d.cn_alpha        % normal force slope
    'rp_cmq',                    d.cmq             % pitch damping slope
    'rp_cnr',                    d.cnr             % yaw damping slope
    'rp_clp',                    d.clp             % roll damping slope
    'rp_r_cp_m',                 d.r_cp            % centre of pressure from CG
    'rp_roll_ref_length_m',      d.roll_ref_length_m % roll damping reference
    'rp_canard_area_m2',         d.canard_area_m2  % canard area
    'rp_canard_cl_alpha',        d.canard_cl_alpha % canard lift slope
    'rp_fin_positions_m',        d.fin_positions   % canard centres of lift, 4 x 3
};
for k = 1:size(tunable, 1)
    p = Simulink.Parameter(tunable{k, 2});
    p.CoderInfo.StorageClass = 'ExportedGlobal';
    assignin('base', tunable{k, 1}, p);
end

% --- Fixed at generation: not part of a design ------------------------------
v = struct();
v.gravity = [-9.819 0 0]';                  % inertial frame (x up)
v.rho = 1.225;                              % air density, kg/m^3
v.N = [0 -1 0; 0 0 -1; 0 1 0; 0 0 1];       % canard lift directions
v.si = [0 0 1; 0 -1 0; 0 0 -1; 0 1 0];      % canard span directions
v.Vsafe = 0.1;                              % low-airspeed guard, m/s

names = fieldnames(v);
for k = 1:numel(names)
    assignin('base', names{k}, v.(names{k}));
end
end
