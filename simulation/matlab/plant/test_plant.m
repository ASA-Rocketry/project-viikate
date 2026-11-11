function tests = test_plant
% Tests for the rocket plant model viikate_rocket.slx.
%
%   runtests('test_plant.m')
%
% Run before generate_plant. testBackToBack compares against the frozen
% closedmodel.slx, which is loaded and modified in memory only, never saved.
tests = functiontests(localfunctions);
end

function setupOnce(tc)
here = fileparts(mfilename('fullpath'));
tc.TestData.here = here;
tc.TestData.closedDir = fullfile(here, '..', 'closedModel');
addpath(here);
tc.addTeardown(@() rmpath(here));

% Keep Simulink caches out of the source tree.
work = fullfile(here, 'work');
if ~exist(work, 'dir'), mkdir(work); end
cfg = Simulink.fileGenControl('getConfig');
old = {cfg.CacheFolder, cfg.CodeGenFolder};
Simulink.fileGenControl('set', 'CacheFolder', work, 'CodeGenFolder', work);
tc.addTeardown(@() Simulink.fileGenControl('set', ...
    'CacheFolder', old{1}, 'CodeGenFolder', old{2}));
end

function testPadHold(tc)
out = sim_plant(4, '');
t = out.t;
pre = t < 3.0 - 1e-9;
post = t >= 3.002 - 1e-9;
n = nnz(pre);
tc.assertGreaterThan(n, 2900);

tc.verifyEqual(out.pos_model_m(pre, :), repmat(out.pos_model_m(1, :), n, 1), ...
    'AbsTol', 1e-12);
tc.verifyEqual(out.vel_model_mps(pre, :), zeros(n, 3), 'AbsTol', 1e-12);
tc.verifyEqual(out.euler_rad(pre, :), zeros(n, 3), 'AbsTol', 1e-12);
tc.verifyEqual(out.sf_body_mps2(pre, :), repmat([9.819 0 0], n, 1), 'AbsTol', 1e-9);
tc.verifyEqual(out.on_pad(pre), ones(n, 1));
tc.verifyEqual(out.mass_kg(pre), repmat(1.352, n, 1), 'AbsTol', 1e-12);

tc.verifyEqual(out.on_pad(post), zeros(nnz(post), 1));
tc.verifyTrue(all(diff(out.pos_model_m(post, 1)) > 0), ...
    'altitude must increase after ignition');
end

function testThrustNeverExceedsWeight(tc)
out = sim_plant(6, 'rp_thrust_scale.Value = 0.05;');
n = numel(out.t);
tc.verifyEqual(out.on_pad, ones(n, 1));
tc.verifyEqual(out.pos_model_m, repmat(out.pos_model_m(1, :), n, 1), 'AbsTol', 1e-12);
end

function testBackToBack(tc)
% Plant plus closedmodel's roll controller (backtoback_harness.slx) against
% closedmodel itself, zero ignition delay, gust seeds 1 and 2 in both.
% Measured maxima (2026-11-11, R2026b Update 1): position 4.3e-14 m,
% velocity 5.0e-14 m/s, attitude 4.5e-14 deg; closedmodel reaches 224.8 m
% at 8 s. The models integrate the same equations, so any difference above
% rounding is a model change, not noise.
here = tc.TestData.here;
closedDir = tc.TestData.closedDir;

plant_parameters();
load_closedparameters(closedDir);

load_system(fullfile(closedDir, 'closedmodel.slx'));
tc.addTeardown(@() close_system('closedmodel', 0));
set_param('closedmodel/wind/Random Number1', 'Seed', '1');
set_param('closedmodel/wind/Random Number2', 'Seed', '2');
six = 'closedmodel/Simple Variable Mass 6DOF (Quaternion)';
ph = get_param(six, 'PortHandles');
names = {'Ve', 'Xe', 'Euler'};
for k = 1:3
    set_param(ph.Outport(k), 'DataLogging', 'on', ...
        'DataLoggingNameMode', 'Custom', 'DataLoggingName', names{k});
end
oldDir = cd(closedDir); % closedmodel reads thrustcurve.mat by relative path
restoreDir = onCleanup(@() cd(oldDir));
ref = sim('closedmodel', 'StopTime', '8', 'SignalLogging', 'on', ...
    'SignalLoggingName', 'logsout', 'ReturnWorkspaceOutputs', 'on');
clear restoreDir

evalin('base', 'rp_ignition_time_s.Value = 0;');
load_system(fullfile(here, 'backtoback_harness.slx'));
tc.addTeardown(@() close_system('backtoback_harness', 0));
hb = sim('backtoback_harness', 'StopTime', '8', 'ReturnWorkspaceOutputs', 'on');
plant_parameters();

refPos = ref.logsout.getElement('Xe').Values;
refVel = ref.logsout.getElement('Ve').Values;
refEul = ref.logsout.getElement('Euler').Values;
pos = hb.yout.getElement('pos_model_m').Values;
vel = hb.yout.getElement('vel_model_mps').Values;
eul = hb.yout.getElement('euler_rad').Values;

[t, ir, ih] = intersect(round(refPos.Time * 1000), round(pos.Time * 1000));
tc.assertEqual(numel(t), 8001, 'expected 1 ms samples from 0 to 8 s');
dPos = max(abs(squeeze(refPos.Data(ir, :)) - squeeze(pos.Data(ih, :))), [], 'all');
dVel = max(abs(squeeze(refVel.Data(ir, :)) - squeeze(vel.Data(ih, :))), [], 'all');
dEul = rad2deg(max(abs(squeeze(refEul.Data(ir, :)) - squeeze(eul.Data(ih, :))), [], 'all'));
fprintf('back-to-back max |diff|: position %.3g m, velocity %.3g m/s, attitude %.3g deg\n', ...
    dPos, dVel, dEul);
fprintf('closedmodel apogee in 8 s: %.1f m\n', max(refPos.Data(:, 1)) - refPos.Data(1, 1));

tc.verifyLessThanOrEqual(dPos, 0.01);
tc.verifyLessThanOrEqual(dVel, 0.01);
tc.verifyLessThanOrEqual(dEul, 0.01);
end

function testV1MatchesClosedParameters(tc)
% viikate_v1.rocket and its motor reproduce what closedparameters.m and
% thrustcurve.mat give closedmodel, so testBackToBack ties v1 to it.
c = closed_values(tc.TestData.closedDir);
d = read_rocket_design(v1_path(tc));
pairs = {
    'inertia_full', c.If; 'inertia_empty', c.Ie; 'cd0', c.Cd0
    'frontal_area_m2', c.A; 'canard_area_m2', c.Af; 'cn_alpha', c.CNa
    'canard_cl_alpha', c.Cla; 'cmq', c.Cmq; 'cnr', c.Cnr; 'clp', c.Clp
    'length_m', c.L; 'roll_ref_length_m', c.b
    'fin_positions', c.fin_positions; 'r_cp', c.r_cp
    'mass_full_kg', c.mf; 'mass_fuel_kg', c.mfuel; 'burn_time_s', c.bt
    'servo_tau_s', c.tau; 'servo_rate_limit_radps', c.alim
};
for k = 1:size(pairs, 1)
    tc.verifyEqual(d.(pairs{k, 1}), pairs{k, 2}, 'AbsTol', 1e-12, pairs{k, 1});
end
s = load(fullfile(tc.TestData.closedDir, 'thrustcurve.mat'));
f = fieldnames(s);
curve = s.(f{1});
t = 0:0.001:0.999;
ref = interp1(curve(1, :), curve(2, :), t, 'linear', 'extrap');
got = interp1(d.thrust_t_s, d.thrust_n, t, 'linear');
tc.verifyEqual(got, ref, 'AbsTol', 1e-9, 'thrust over the burn');
end

function testReaderRejectsUnknownKey(tc)
path = design_variant(tc, @(text) [text sprintf('foo = 1\n')]);
tc.verifyError(@() read_rocket_design(path), 'rocket:design');
try
    read_rocket_design(path);
catch e
    tc.verifySubstring(e.message, 'unknown key foo');
end
end

function testReaderRejectsMissingKey(tc)
path = design_variant(tc, @(text) regexprep(text, '(?m)^cd0 = .*$', ''));
try
    read_rocket_design(path);
    tc.verifyFail('expected an error');
catch e
    tc.verifyEqual(e.identifier, 'rocket:design');
    tc.verifySubstring(e.message, 'missing key cd0');
end
end

function p = v1_path(tc)
p = fullfile(tc.TestData.here, '..', '..', 'rockets', 'viikate_v1.rocket');
end

function path = design_variant(tc, edit)
% v1 edited by edit(text), written next to a copy of its motor file.
dir = tc.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture).Folder;
mkdir(fullfile(dir, 'motors'));
rockets = fullfile(tc.TestData.here, '..', '..', 'rockets');
copyfile(fullfile(rockets, 'motors', 'viikate_v1.eng'), fullfile(dir, 'motors'));
path = fullfile(dir, 'd.rocket');
fid = fopen(path, 'w');
fwrite(fid, edit(fileread(v1_path(tc))));
fclose(fid);
end

function c = closed_values(closedDir)
% Variables closedparameters.m defines, without its 'clear *' and sim().
lines = splitlines(fileread(fullfile(closedDir, 'closedparameters.m')));
keep = ~startsWith(strtrim(lines), {'clear', 'out = sim'});
c = eval_lines(strjoin(lines(keep), newline));
end

function c = eval_lines(code)
% code is the repo's own frozen closedparameters.m, evaluated the same way
% load_closedparameters does, but into this function's workspace.
eval(code);
c = struct();
for v = who()'
    if ~strcmp(v{1}, 'code')
        c.(v{1}) = eval(v{1});
    end
end
end

function out = sim_plant(stop_s, override)
% Simulate viikate_rocket with zero fin commands. override is MATLAB code run
% in the base workspace after plant_parameters, e.g. 'rp_thrust_scale.Value = 0.05;'.
plant_parameters();
if ~isempty(override)
    evalin('base', override);
end
in = Simulink.SimulationInput('viikate_rocket');
in = in.setExternalInput([0 0 0 0 0]);
in = in.setModelParameter('StopTime', num2str(stop_s), ...
    'SaveOutput', 'on', 'SaveFormat', 'Dataset', 'OutputSaveName', 'yout');
so = sim(in);
plant_parameters();
y = so.yout;
out = struct();
for k = 1:y.numElements
    v = y.getElement(k).Values;
    d = v.Data;
    if ndims(d) == 3
        d = reshape(d, [], size(d, 3)).';
    end
    out.(y.getElement(k).Name) = double(d);
    out.t = v.Time;
end
end

function load_closedparameters(closedDir)
% closedparameters.m without its 'clear *' and trailing sim() call.
lines = splitlines(fileread(fullfile(closedDir, 'closedparameters.m')));
keep = ~startsWith(strtrim(lines), {'clear', 'out = sim'});
evalin('base', strjoin(lines(keep), newline));
end
