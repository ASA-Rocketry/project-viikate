function generate_plant()
% Generate the rocket plant C code into simulation/plant_codegen.
%
%   cd simulation/matlab/plant; runtests('test_plant.m'); generate_plant
%
% Writes, under simulation/plant_codegen:
%   *.c, *.h     viikate_rocket generated with Embedded Coder, plus the
%                MathWorks support headers it includes (from packNGo)
%   reference/   a scripted fin-command scenario simulated in Simulink, which
%                code/sim/tests/test_replay.py replays through the generated C
%   PROVENANCE   SHA-256 of the inputs and of every file above;
%                code/sim/tools/check_provenance.py fails when an input
%                changed without regenerating or an output is missing or edited
%
% The model is loaded, never saved: saving it changes its hash, so a saved
% model always needs a regenerate.

plantDir = fileparts(mfilename('fullpath'));
simDir = fileparts(fileparts(plantDir));            % <repo>/simulation
outDir = fullfile(simDir, 'plant_codegen');
refDir = fullfile(outDir, 'reference');
work = fullfile(plantDir, 'work');
model = 'viikate_rocket';
inputs = {
    fullfile('simulation', 'matlab', 'plant', [model '.slx'])
    fullfile('simulation', 'matlab', 'plant', 'plant_parameters.m')
    fullfile('simulation', 'matlab', 'closedModel', 'thrustcurve.mat')
};

addpath(plantDir);
plant_parameters();
if bdIsLoaded(model), close_system(model, 0); end
load_system(fullfile(plantDir, [model '.slx']));
closeModel = onCleanup(@() close_system(model, 0));

if ~exist(work, 'dir'), mkdir(work); end
cfg = Simulink.fileGenControl('getConfig');
old = {cfg.CacheFolder, cfg.CodeGenFolder};
Simulink.fileGenControl('set', 'CacheFolder', work, 'CodeGenFolder', work);
restoreCfg = onCleanup(@() Simulink.fileGenControl('set', ...
    'CacheFolder', old{1}, 'CodeGenFolder', old{2}));

% --- Code -------------------------------------------------------------------
slbuild(model);
buildDir = fullfile(work, [model '_ert_rtw']);
pkgZip = fullfile(work, [model '_pkg.zip']);
pkgDir = fullfile(work, 'pkg');
if exist(pkgDir, 'dir'), rmdir(pkgDir, 's'); end
packNGo(fullfile(buildDir, 'buildInfo.mat'), 'packType', 'flat', ...
    'nestedZipFiles', false, 'fileName', pkgZip);
unzip(pkgZip, pkgDir);

if ~exist(outDir, 'dir'), mkdir(outDir); end
delete(fullfile(outDir, '*.c'));
delete(fullfile(outDir, '*.h'));
skip = {'ert_main.c', 'rt_main.c', 'rt_logging.c', 'rt_logging.h'};
files = [dir(fullfile(pkgDir, '*.c')); dir(fullfile(pkgDir, '*.h'))];
for k = 1:numel(files)
    if ~any(strcmp(files(k).name, skip))
        copyfile(fullfile(pkgDir, files(k).name), outDir);
    end
end

% --- Reference scenario -----------------------------------------------------
if ~exist(refDir, 'dir'), mkdir(refDir); end
ignition = 0.5;
stop = 8;
d2r = pi / 180;
changes = [            % t_s, fin0..fin3 (rad), each row held until the next
    0.0   0        0  0         0
    1.5   2*d2r    2*d2r  2*d2r  2*d2r
    2.0   1*d2r    0  -1*d2r    0
    2.5   0        0  0         0
];
write_input_csv(fullfile(refDir, 'input.csv'), changes);
fid = fopen(fullfile(refDir, 'params.txt'), 'w');
fprintf(fid, 'ignition_time_s=%g\n', ignition);
fclose(fid);

t = (0:round(stop / 0.001))' * 0.001;
u = zeros(numel(t), 4);
for k = 1:size(changes, 1)
    u(round(t / 0.001) >= round(changes(k, 1) / 0.001), :) = ...
        repmat(changes(k, 2:5), nnz(round(t / 0.001) >= round(changes(k, 1) / 0.001)), 1);
end
evalin('base', sprintf('rp_ignition_time_s.Value = %.17g;', ignition));
restoreParams = onCleanup(@() plant_parameters());
in = Simulink.SimulationInput(model);
in = in.setExternalInput([t u]);
in = in.setModelParameter('StopTime', num2str(stop), 'SaveOutput', 'on', ...
    'SaveFormat', 'Dataset', 'OutputSaveName', 'yout');
so = sim(in);
write_output_csv(fullfile(refDir, 'output.csv'), so.yout);
clear restoreParams

% --- Provenance ---------------------------------------------------------------
% Inputs first, then every file written here, so a changed model and a
% missing or hand-edited output are both caught.
outputs = [dir(fullfile(outDir, '*.c')); dir(fullfile(outDir, '*.h')); ...
    dir(fullfile(refDir, '*'))];
outputs = outputs(~[outputs.isdir]);
relOut = arrayfun(@(f) fullfile('simulation', ...
    extractAfter(fullfile(f.folder, f.name), [simDir filesep])), outputs, ...
    'UniformOutput', false);
listed = [inputs; sort(relOut)];
fid = fopen(fullfile(outDir, 'PROVENANCE'), 'w');
for k = 1:numel(listed)
    [status, out] = system(sprintf('sha256sum "%s"', fullfile(fileparts(simDir), listed{k})));
    assert(status == 0, 'sha256sum failed: %s', out);
    fprintf(fid, 'sha256 %s %s\n', strtok(out), strrep(listed{k}, filesep, '/'));
end
fprintf(fid, 'matlab_release R%s\n', version('-release'));
fprintf(fid, 'target %s\n', get_param(model, 'SystemTargetFile'));
fprintf(fid, 'generated %s\n', char(datetime('now', 'TimeZone', 'UTC', ...
    'Format', 'yyyy-MM-dd''T''HH:mm:ss''Z''')));
fclose(fid);
fprintf('Generated %s\n', outDir);
end

function write_input_csv(path, changes)
fid = fopen(path, 'w');
fprintf(fid, 't_s,fin0_rad,fin1_rad,fin2_rad,fin3_rad\n');
fprintf(fid, '%.12g,%.12g,%.12g,%.12g,%.12g\n', changes.');
fclose(fid);
end

function write_output_csv(path, yout)
t = yout.getElement('sf_body_mps2').Values.Time;
keep = mod(round(t / 0.001), 10) == 0;      % every 10 ms
names = {'sf_body_mps2', 'omega_body_radps', 'pos_model_m', 'vel_model_mps', ...
    'euler_rad', 'dcm_be', 'fin_angle_rad', 'mass_kg', 'on_pad'};
cols = cell(size(names));
for k = 1:numel(names)
    cols{k} = rows_per_sample(yout.getElement(names{k}).Values.Data, numel(t));
end
data = [t, cols{:}];
data = data(keep, :);
header = ['t_s,sf_x,sf_y,sf_z,w_x,w_y,w_z,pos_x,pos_y,pos_z,vel_x,vel_y,vel_z,' ...
    'phi,theta,psi,dcm_11,dcm_12,dcm_13,dcm_21,dcm_22,dcm_23,dcm_31,dcm_32,dcm_33,' ...
    'fin0,fin1,fin2,fin3,mass_kg,on_pad'];
fid = fopen(path, 'w');
fprintf(fid, '%s\n', header);
fmt = [strjoin(repmat({'%.12g'}, 1, size(data, 2)), ','), '\n'];
fprintf(fid, fmt, data.');
fclose(fid);
end

function r = rows_per_sample(d, n)
% Logged data is N x w, or a x b x N for non-vector samples. Return N x (a*b),
% matrices flattened row by row (dcm_11 dcm_12 ... dcm_33).
if ndims(d) == 3 && size(d, 3) == n
    r = reshape(permute(d, [2 1 3]), [], n).';
else
    r = reshape(d, n, []);
end
end
