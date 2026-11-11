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
};
for k = 1:size(tunable, 1)
    p = Simulink.Parameter(tunable{k, 2});
    p.CoderInfo.StorageClass = 'ExportedGlobal';
    assignin('base', tunable{k, 1}, p);
end

% --- Fixed at generation -----------------------------------------------------
v = struct();

% From the design
v.bt = d.burn_time_s;                       % motor burn time, s
v.thrust_t_s = d.thrust_t_s;                % thrust table time, s (128 points)
v.thrust_n = d.thrust_n;                    % thrust table force, N
v.Ie = d.inertia_empty;                     % empty inertia, kg m^2
v.If = d.inertia_full;                      % full inertia, kg m^2
v.Cd0 = d.cd0;                              % drag coefficient
v.A = d.frontal_area_m2;                    % frontal area, m^2
v.Af = d.canard_area_m2;                    % fin area, m^2
v.fin_positions = d.fin_positions;          % canard centres of lift from CG, m
v.r_cp = d.r_cp;                            % centre of pressure from CG, m
v.Cla = d.canard_cl_alpha;                  % fin lift coefficient slope
v.CNa = d.cn_alpha;                         % normal force coefficient slope
v.Cmq = d.cmq;                              % pitch damping moment slope
v.Cnr = d.cnr;                              % yaw damping moment slope
v.Clp = d.clp;                              % roll damping moment slope
v.L = d.length_m;                           % rocket length, m
v.b = d.roll_ref_length_m;                  % roll damping reference length, m

% Fixed in the model, not part of a design
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
