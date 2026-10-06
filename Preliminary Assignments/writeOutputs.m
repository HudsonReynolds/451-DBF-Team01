function outTable = writeOutputs(params, excelFile, sheetName)
% WRITEOUTPUTS  Write every calculated value in params to an "Outputs" sheet.
%
%   writeOutputs(params)                          -> SizingParams.xlsx, sheet "Outputs"
%   writeOutputs(params, excelFile)
%   writeOutputs(params, excelFile, sheetName)
%   outTable = writeOutputs(...)                  -> also returns the cell array written
%
% Compares the final params struct against the inputs in Sheet1 (read with
% readParams) and writes one row per value that the analysis produced:
%   - fields that don't exist in Sheet1 (e.g. geometry.S_wing, aero.CD_0)
%   - Sheet1 inputs that the code overwrote (e.g. performance.MTOM after
%     the weight iteration) -- flagged in the Source column with the
%     original sheet value
%
% Nested structs are flattened into dotted names, e.g.
%   stability.lastControlSurfaces.Elevator.margin_deg
% Scalars/logicals/text go in as real cell values. Short vectors are written
% as text (mat2str), long arrays as a size summary, function handles as
% their expression.
%
% Units / Description:
%   - pulled from the trailing comment of the line that assigns the value,
%     when it follows this repo's convention  "... ; % Description [unit]"
%   - anything you type into the Units/Description columns of the Outputs
%     sheet yourself is kept on the next run (your edits win)
%
% Only the Outputs sheet is touched -- Sheet1, MassBudget, Hinge Moments
% (and their formulas) are left alone. The sheet is cleared and rewritten
% every run so it never holds stale rows. Close the workbook in Excel first.

if nargin < 2 || isempty(excelFile), excelFile = "SizingParams.xlsx"; end
if nargin < 3 || isempty(sheetName), sheetName = "Outputs";           end

MAX_INLINE_ELEMENTS = 10;   % arrays with more elements than this get a size summary instead

%% Inputs as read from Sheet1 (same reader Main.m uses)
inputs = readParams(excelFile);

%% Flatten final params into rows: {subclass, name, value}
rows = cell(0, 3);
subclasses = fieldnames(params);
for i = 1:numel(subclasses)
    sc = subclasses{i};
    if isstruct(params.(sc)) && isscalar(params.(sc))
        rows = [rows; local_flatten(params.(sc), sc, '')]; %#ok<AGROW>
    else
        rows(end+1, :) = {'', sc, params.(sc)}; %#ok<AGROW>
    end
end

%% Units / descriptions: code comments first, then user edits from the existing sheet
meta = local_metaFromCode(fileparts(which(char(excelFile))));
meta = local_metaFromSheet(excelFile, sheetName, meta);

%% Keep only calculated values and build the output table
header = {'Parameter', 'Subclass', 'Value', 'Units', 'Description', 'Source'};
body   = cell(0, numel(header));
for k = 1:size(rows, 1)
    sc   = rows{k, 1};
    name = rows{k, 2};
    val  = rows{k, 3};

    % Is this an original Sheet1 input? (only top-level names can be)
    isInput = ~isempty(sc) && isfield(inputs, sc) && ~contains(name, '.') ...
              && isfield(inputs.(sc), name);
    if isInput
        inVal = inputs.(sc).(name);
        if isequaln(inVal, val), continue; end            % untouched input -> not an output
        source = sprintf('Overwritten input (Sheet1 value: %s)', local_toText(inVal, MAX_INLINE_ELEMENTS));
    else
        source = 'Calculated';
    end

    key = [sc '|' name];
    units = ''; desc = '';
    if isKey(meta, key)
        m = meta(key); units = m{1}; desc = m{2};
    end

    body(end+1, :) = {name, sc, local_toCellValue(val, MAX_INLINE_ELEMENTS), units, desc, source}; %#ok<AGROW>
end

outTable = [header; body];
outTable{1, 8} = sprintf('Written by writeOutputs.m on %s', ...
                         char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm')));
outTable(cellfun(@isempty, outTable)) = {''};   % writecell can't write [] cells

%% Write (clears only this sheet first, other sheets untouched)
try
    writecell(outTable, excelFile, 'Sheet', sheetName, 'WriteMode', 'overwritesheet');
catch err
    error('writeOutputs:writeFailed', ...
        'Could not write sheet "%s" in %s -- is the workbook open in Excel?\n%s', ...
        sheetName, excelFile, err.message);
end

fprintf('writeOutputs: wrote %d calculated values to %s (sheet "%s").\n', ...
        size(body, 1), excelFile, sheetName);
end

%% ======================================================================
function rows = local_flatten(s, subclass, prefix)
% Recursively turn a (possibly nested) struct into {subclass, dottedName, value} rows.
rows = cell(0, 3);
f = fieldnames(s);
for i = 1:numel(f)
    name = f{i};
    if ~isempty(prefix), name = [prefix '.' name]; end
    v = s.(f{i});
    if isstruct(v) && isscalar(v)
        rows = [rows; local_flatten(v, subclass, name)]; %#ok<AGROW>
    elseif isstruct(v)                                   % struct array -> name(k).field
        for k = 1:numel(v)
            rows = [rows; local_flatten(v(k), subclass, sprintf('%s(%d)', name, k))]; %#ok<AGROW>
        end
    else
        rows(end+1, :) = {subclass, name, v}; %#ok<AGROW>
    end
end
end

%% ----------------------------------------------------------------------
function c = local_toCellValue(v, maxN)
% Value as Excel will receive it: real number/bool where possible, text otherwise.
if (isnumeric(v) || islogical(v)) && isscalar(v) && isreal(v)
    c = double(v);
    if islogical(v), c = logical(v); end
    if ~isfinite(c), c = num2str(c); end                % Inf/NaN -> 'Inf'/'NaN' text
elseif ischar(v) || (isstring(v) && isscalar(v))
    c = char(v);
else
    c = local_toText(v, maxN);
end
end

%% ----------------------------------------------------------------------
function t = local_toText(v, maxN)
% Readable one-cell text for any MATLAB value.
sz = strjoin(arrayfun(@num2str, size(v), 'UniformOutput', false), 'x');
if ismissing_safe(v)
    t = '(blank)';
elseif ischar(v)
    t = v;
elseif isstring(v)
    if isscalar(v), t = char(v); else, t = char(strjoin(v, ', ')); end
elseif isa(v, 'function_handle')
    t = func2str(v);
elseif (isnumeric(v) || islogical(v)) && isempty(v)
    t = '[]';
elseif (isnumeric(v) || islogical(v)) && numel(v) <= maxN && ismatrix(v)
    if islogical(v), t = mat2str(v); else, t = mat2str(double(v), 6); end
elseif isnumeric(v) && isreal(v)
    t = sprintf('[%s %s] min %.4g, max %.4g', sz, class(v), min(v(:)), max(v(:)));
elseif iscellstr(v) && numel(v) <= maxN %#ok<ISCLSTR>
    t = strjoin(v(:)', ', ');
elseif isnumeric(v) || islogical(v)
    t = sprintf('[%s %s]', sz, class(v));
elseif istable(v)
    t = sprintf('[%dx%d table: %s]', height(v), width(v), strjoin(v.Properties.VariableNames, ', '));
else
    t = sprintf('[%s %s]', sz, class(v));
end
end

function tf = ismissing_safe(v)
tf = false;
try
    tf = isscalar(v) && ~isstruct(v) && ~isa(v, 'function_handle') && ismissing(v);
catch
end
end

%% ----------------------------------------------------------------------
function meta = local_metaFromCode(rootDir)
% Scan the repo's .m files for lines like
%   params.geometry.S_wing = ...; % Wing area from design wing loading [m^2]
% and return a map 'geometry|S_wing' -> {units, description}.
meta = containers.Map('KeyType', 'char', 'ValueType', 'any');
if isempty(rootDir), rootDir = pwd; end
files = dir(fullfile(rootDir, '**', '*.m'));
pat = '^\s*params\.(\w+)\.(\w+)\s*=[^=].*?%\s*(.*?)\s*\[([^\]]*)\]\s*$';
for i = 1:numel(files)
    try
        txt = fileread(fullfile(files(i).folder, files(i).name));
    catch
        continue
    end
    lines = regexp(txt, '\r?\n', 'split');
    for j = 1:numel(lines)
        tok = regexp(lines{j}, pat, 'tokens', 'once');
        if isempty(tok), continue; end
        key = [tok{1} '|' tok{2}];
        if ~isKey(meta, key)                 % first definition found wins
            meta(key) = {strtrim(tok{4}), strtrim(tok{3})};
        end
    end
end
end

%% ----------------------------------------------------------------------
function meta = local_metaFromSheet(excelFile, sheetName, meta)
% Keep any Units/Description the user typed into a previous Outputs sheet.
try
    if ~any(strcmp(sheetnames(excelFile), sheetName)), return; end
    old = readcell(excelFile, 'Sheet', sheetName);
catch
    return
end
for r = 2:size(old, 1)
    if size(old, 2) < 5 || ~ischar(old{r,1}) || ~ischar(old{r,2}), continue; end
    key = [old{r,2} '|' old{r,1}];
    units = old{r,4}; desc = old{r,5};
    if ~ischar(units), units = ''; end
    if ~ischar(desc),  desc  = ''; end
    if isKey(meta, key)
        m = meta(key);
        if isempty(units), units = m{1}; end
        if isempty(desc),  desc  = m{2}; end
    end
    if ~isempty(units) || ~isempty(desc)
        meta(key) = {units, desc};
    end
end
end