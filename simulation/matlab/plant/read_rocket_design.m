function d = read_rocket_design(path)
% Read a rocket design file (simulation/rockets/<name>.rocket) and its RASP
% .eng motor file, with the same rules and messages as the C parser
% code/sim/plant/rocket/rocket_design.c. Format: simulation/rockets/README.md.
%
%   d = read_rocket_design(path)
%
% d has every key of the file (lists as row vectors), burn_time_s defaulted
% to the last motor time, and the values the model uses:
%   thrust_points  n x 2, the motor's points after inserting (0, 0)
%   thrust_t_s, thrust_n  1 x 128, padded with t_last + k s and 0 N
%   inertia_full, inertia_empty  3 x 3 tensors from diagonal and products
%   r_cp           3 x 1, [-cp_aft_of_cg_m; 0; 0]
%   fin_positions  4 x 3, [d 0 r; d -r 0; d 0 -r; d r 0]
%   servo_rate_limit_radps
% Errors have the identifier 'rocket:design'.

text = read_text(path);
d = parse_design(text, path);
motorPath = fullfile(fileparts(path), d.motor);
d.thrust_points = parse_eng(read_text(motorPath), motorPath);
if ~isfield(d, 'burn_time_s')
    d.burn_time_s = d.thrust_points(end, 1);
end
if ~(d.mass_fuel_kg < d.mass_full_kg)
    fail('%s: mass_fuel_kg must be less than mass_full_kg', path);
end
d.inertia_full = tensor(d.inertia_full_kgm2, d.inertia_full_products_kgm2);
d.inertia_empty = tensor(d.inertia_empty_kgm2, d.inertia_empty_products_kgm2);
check_pd(d.inertia_full, 'inertia_full_kgm2', path);
check_pd(d.inertia_empty, 'inertia_empty_kgm2', path);

n = size(d.thrust_points, 1);
pad = 128 - n;
d.thrust_t_s = [d.thrust_points(:, 1).', d.thrust_points(end, 1) + (1:pad)];
d.thrust_n = [d.thrust_points(:, 2).', zeros(1, pad)];
d.r_cp = [-d.cp_aft_of_cg_m; 0; 0];
a = d.canard_fwd_of_cg_m;
r = d.canard_lift_radius_m;
d.fin_positions = [a 0 r; a -r 0; a 0 -r; a r 0];
d.servo_rate_limit_radps = deg2rad(d.servo_rate_limit_dps);
end

function k = keys()
% name, count, rule, optional; in the order of the key table.
k = {
    'burn_time_s', 1, 'positive', true
    'mass_full_kg', 1, 'positive', false
    'mass_fuel_kg', 1, 'positive', false
    'inertia_full_kgm2', 3, 'any', false
    'inertia_empty_kgm2', 3, 'any', false
    'inertia_full_products_kgm2', 3, 'any', true
    'inertia_empty_products_kgm2', 3, 'any', true
    'length_m', 1, 'positive', false
    'frontal_area_m2', 1, 'positive', false
    'cd0', 1, 'non_negative', false
    'cn_alpha', 1, 'non_negative', false
    'cp_aft_of_cg_m', 1, 'any', false
    'cmq', 1, 'any', false
    'cnr', 1, 'any', false
    'clp', 1, 'any', false
    'roll_ref_length_m', 1, 'positive', false
    'canard_area_m2', 1, 'positive', false
    'canard_cl_alpha', 1, 'non_negative', false
    'canard_fwd_of_cg_m', 1, 'any', false
    'canard_lift_radius_m', 1, 'positive', false
    'servo_tau_s', 1, 'positive', false
    'servo_rate_limit_dps', 1, 'positive', false
};
end

function d = parse_design(text, path)
table = keys();
d = struct();
lines = split_lines(text);
for ln = 1:numel(lines)
    line = lines{ln};
    hash = find(line == '#', 1);
    if ~isempty(hash), line = line(1:hash - 1); end
    line = strtrim(line);
    if isempty(line), continue; end
    eq = find(line == '=', 1);
    if isempty(eq)
        fail('%s:%d: expected key = value', path, ln);
    end
    key = strtrim(line(1:eq - 1));
    value = strtrim(line(eq + 1:end));
    row = find(strcmp(table(:, 1), key));
    if isempty(row) && ~any(strcmp(key, {'name', 'motor'}))
        fail('%s:%d: unknown key %s', path, ln, key);
    end
    if isfield(d, key)
        fail('%s:%d: duplicate key %s', path, ln, key);
    end
    switch key
        case 'name'
            if isempty(regexp(value, '^[a-z0-9_]{1,63}$', 'once'))
                fail('%s:%d: bad name', path, ln);
            end
            d.name = value;
        case 'motor'
            parts = strsplit(value, '/');
            if isempty(value) || value(1) == '/' || any(strcmp(parts, '..'))
                fail('%s:%d: motor path must be relative without ..', path, ln);
            end
            d.motor = value;
        otherwise
            items = strsplit(value, ',');
            count = table{row, 2};
            if numel(items) ~= count
                fail('%s:%d: expected %d values for %s', path, ln, count, key);
            end
            v = zeros(1, count);
            for i = 1:count
                v(i) = number(items{i});
                if isnan(v(i))
                    fail('%s:%d: bad number for %s', path, ln, key);
                end
            end
            if ~all(in_rule(v, table{row, 3}))
                fail('%s:%d: %s out of range', path, ln, key);
            end
            d.(key) = v;
    end
end
for key = {'name', 'motor'}
    if ~isfield(d, key{1})
        fail('%s: missing key %s', path, key{1});
    end
end
for row = 1:size(table, 1)
    key = table{row, 1};
    if ~isfield(d, key)
        if ~table{row, 4}
            fail('%s: missing key %s', path, key);
        elseif table{row, 2} > 1
            d.(key) = zeros(1, table{row, 2});
        end
    end
end
end

function points = parse_eng(text, path)
lines = split_lines(text);
header = false;
points = zeros(0, 2);
for ln = 1:numel(lines)
    line = lines{ln};
    semi = find(line == ';', 1);
    if ~isempty(semi), line = line(1:semi - 1); end
    fields = strsplit(strtrim(line));
    if isempty(fields{1}), continue; end
    if ~header
        if numel(fields) ~= 7
            fail('%s:%d: missing motor header', path, ln);
        end
        header = true;
        continue;
    end
    t = NaN; n = NaN;
    if numel(fields) == 2
        t = number(fields{1});
        n = number(fields{2});
    end
    if isnan(t) || isnan(n) || t < 0
        if numel(fields) == 7 && ~isempty(points)
            fail('%s:%d: file holds more than one motor', path, ln);
        end
        fail('%s:%d: bad thrust point', path, ln);
    end
    if size(points, 1) == 127
        fail('%s:%d: too many thrust points (max 127)', path, ln);
    end
    if ~isempty(points) && t <= points(end, 1)
        fail('%s:%d: thrust times must increase', path, ln);
    end
    if n < 0
        fail('%s:%d: thrust must be >= 0', path, ln);
    end
    points(end + 1, :) = [t n]; %#ok<AGROW>
end
if ~header
    fail('%s: missing motor header', path);
end
if size(points, 1) < 2
    fail('%s: need at least 2 thrust points', path);
end
if points(1, 1) > 0
    points = [0 0; points];
end
end

function text = read_text(path)
fid = fopen(path, 'r');
if fid < 0
    fail('%s: cannot read file', path);
end
closer = onCleanup(@() fclose(fid));
text = fread(fid, [1 Inf], '*char');
if numel(text) > 65536
    fail('%s: file too large', path);
end
end

function lines = split_lines(text)
lines = strsplit(text, newline, 'CollapseDelimiters', false);
for k = 1:numel(lines)
    if ~isempty(lines{k}) && lines{k}(end) == char(13)
        lines{k} = lines{k}(1:end - 1);
    end
end
end

function v = number(text)
% A whole decimal number, finite; NaN otherwise.
text = strtrim(text);
if isempty(regexp(text, '^[+-]?(\d+\.?\d*|\.\d+)([eE][+-]?\d+)?$', 'once'))
    v = NaN;
    return;
end
v = str2double(text);
if ~isfinite(v)
    v = NaN;
end
end

function ok = in_rule(v, rule)
switch rule
    case 'non_negative', ok = v >= 0;
    case 'positive', ok = v > 0;
    otherwise, ok = true(size(v));
end
end

function J = tensor(diagonal, products)
J = [diagonal(1) products(1) products(2)
     products(1) diagonal(2) products(3)
     products(2) products(3) diagonal(3)];
end

function check_pd(J, key, path)
if ~(J(1, 1) > 0 && det(J(1:2, 1:2)) > 0 && det(J) > 0)
    fail('%s: %s is not positive-definite', path, key);
end
end

function fail(varargin)
error('rocket:design', varargin{:});
end
