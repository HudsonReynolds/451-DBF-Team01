% WRITTEN BY CLAUDE
function P = loadPropData(filename)
%LOADPROPDATA  Load an APC PER3 propeller data file and build a 2-D interpolant.
%
%   P = loadPropData('PER3_8x6.txt') parses the file (which contains one
%   table per propeller RPM) and returns a struct P that can interpolate
%   every tabulated quantity as a function of RPM and airspeed V (mph).
%
%   Once loaded, query it with the handle P.query:
%
%       P = loadPropData('PER3_8x6.txt');
%       r = P.query(12500, 45)      % RPM = 12500, V = 45 mph
%
%   r is a struct with a field for every output column, e.g.
%       r.J          advance ratio (V/nD)
%       r.Pe         efficiency
%       r.Ct         thrust coefficient
%       r.Cp         power coefficient
%       r.PWR_Hp     power (hp)          r.PWR_W     power (W)
%       r.Torque_inlbf                   r.Torque_Nm
%       r.Thrust_lbf                     r.Thrust_N
%       r.ThrPwr_g_per_W                 r.Mach   r.Reyn   r.FOM
%       r.inRange    true if (RPM,V) is inside the tabulated data envelope
%
%   RPM and V may be scalars or arrays (same size, or scalars broadcast),
%   so you can query many operating points at once.
%
%   How it works: the file tabulates each quantity on an irregular grid --
%   RPM is regular (1000:1000:25000) but each RPM block covers a different
%   span of speeds. The values are therefore interpolated with a scattered
%   linear interpolant over the (RPM, V) plane. Points that fall OUTSIDE the
%   tabulated envelope (e.g. a speed higher than any tabulated at that RPM)
%   return NaN and inRange = false rather than an unreliable extrapolation.
%
%   See also queryProp, scatteredInterpolant.

    if nargin < 1 || isempty(filename)
        filename = 'PER3_8x6.txt';
    end

    % ---- Read file --------------------------------------------------------
    txt   = fileread(filename);
    lines = regexp(txt, '\r\n|\r|\n', 'split');

    % ---- Column definitions (order as they appear in the file) -----------
    % Column 1 is V (an input); columns 2..15 are the interpolated outputs.
    fieldNames = {'J','Pe','Ct','Cp','PWR_Hp','Torque_inlbf','Thrust_lbf', ...
                  'PWR_W','Torque_Nm','Thrust_N','ThrPwr_g_per_W', ...
                  'Mach','Reyn','FOM'};
    fieldUnits = {'V/nD','-','-','-','hp','in-lbf','lbf', ...
                  'W','N-m','N','g/W','-','-','-'};
    nOut = numel(fieldNames);

    % ---- Parse RPM blocks into a single scattered point cloud ------------
    rpmCol = [];            % Nx1 : RPM at each point
    vCol   = [];            % Nx1 : V (mph) at each point
    outCol = [];            % Nx14: the 14 output quantities at each point
    rpmList = [];
    curRPM  = NaN;

    for i = 1:numel(lines)
        ln = lines{i};

        tok = regexp(ln, 'PROP\s+RPM\s*=\s*(\d+)', 'tokens', 'once');
        if ~isempty(tok)
            curRPM  = str2double(tok{1});
            rpmList = [rpmList; curRPM]; %#ok<AGROW>
            continue
        end

        if isnan(curRPM), continue, end

        % A data row parses to exactly 15 floats; header/units/blank lines
        % do not (they contain text such as "(mph)" or "-").
        nums = sscanf(ln, '%f');
        if numel(nums) == 15
            rpmCol = [rpmCol; curRPM];   %#ok<AGROW>
            vCol   = [vCol;   nums(1)];  %#ok<AGROW>
            outCol = [outCol; nums(2:15).']; %#ok<AGROW>
        end
    end

    if isempty(rpmCol)
        error('loadPropData:noData', ...
              'No data rows were parsed from "%s". Check the file format.', filename);
    end

    % ---- Normalise coordinates so the two axes have comparable scale ------
    % (RPM ~ 1e3-2.5e4, V ~ 0-175: without scaling the triangulation used by
    %  scatteredInterpolant would be badly conditioned.)
    rpmRange = [min(rpmCol) max(rpmCol)];
    vRange   = [min(vCol)   max(vCol)];
    nrm = @(rpm,V) deal( (rpm - rpmRange(1)) ./ diff(rpmRange), ...
                         (V   - vRange(1))   ./ diff(vRange) );

    [xn, yn] = nrm(rpmCol, vCol);

    % ---- Build one interpolant per output column -------------------------
    % All columns share the same (xn,yn) sites, so build the triangulation
    % once and just swap the Values for the remaining columns.
    F0 = scatteredInterpolant(xn, yn, outCol(:,1), 'linear', 'none');
    Fs = cell(1, nOut);
    Fs{1} = F0;
    for k = 2:nOut
        Fk = F0;
        Fk.Values = outCol(:,k);
        Fs{k} = Fk;
    end

    % ---- Package result ---------------------------------------------------
    P = struct();
    P.file        = filename;
    P.rpms        = unique(rpmList(:)).';
    P.fields      = fieldNames;
    P.units       = fieldUnits;
    P.rpmRange    = rpmRange;
    P.vRange      = vRange;
    P.nPoints     = numel(rpmCol);
    % keep the raw table around in case it is useful
    P.raw.RPM     = rpmCol;
    P.raw.V       = vCol;
    P.raw.values  = outCol;

    % Query handle: closes over the interpolants + metadata only.
    P.query = @(rpm, V) queryProp(rpm, V, Fs, fieldNames, fieldUnits, nrm);

    fprintf(['Loaded "%s": %d RPM tables (%g to %g), %d data points.\n' ...
             'Query with:  r = P.query(rpm, V)   (V in mph)\n'], ...
             filename, numel(P.rpms), P.rpms(1), P.rpms(end), P.nPoints);
end
