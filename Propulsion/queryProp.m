% WRITTEN BY CLAUDE
function r = queryProp(rpm, V, Fs, fieldNames, fieldUnits, nrm)
%QUERYPROP  Evaluate all propeller quantities at a given RPM and speed.
%
%   This is the function invoked by the handle P.query returned by
%   loadPropData. You normally do not call it directly -- use:
%
%       P = loadPropData('PER3_8x6.txt');
%       r = P.query(rpm, V);
%
%   rpm, V may be scalars or arrays of the same size (scalars are
%   broadcast). Every output field has the same size as the query.
%   Points outside the tabulated (RPM,V) envelope return NaN, and
%   r.inRange is false there.

    % ---- Broadcast scalar inputs to a common size ------------------------
    if isscalar(rpm) && ~isscalar(V), rpm = rpm + zeros(size(V)); end
    if isscalar(V) && ~isscalar(rpm), V   = V   + zeros(size(rpm)); end
    if ~isequal(size(rpm), size(V))
        error('queryProp:sizeMismatch', ...
              'RPM and V must be the same size (or one of them scalar).');
    end
    sz = size(rpm);

    % ---- Normalise the query points the same way the sites were ----------
    [xq, yq] = nrm(rpm(:), V(:));

    % ---- Evaluate every interpolant --------------------------------------
    r = struct();
    r.RPM = rpm;
    r.V   = V;
    firstVals = [];
    for k = 1:numel(fieldNames)
        vals = reshape(Fs{k}(xq, yq), sz);
        r.(fieldNames{k}) = vals;
        if k == 1, firstVals = vals; end
    end

    % A NaN in the (first) result means the point was outside the data hull.
    r.inRange = ~isnan(firstVals);
    r.units   = cell2struct(fieldUnits(:), fieldNames(:), 1);

    if ~all(r.inRange(:))
        warning('queryProp:outOfRange', ...
            ['%d of %d query point(s) fall outside the tabulated (RPM,V) ' ...
             'envelope and were set to NaN.'], ...
            nnz(~r.inRange), numel(r.inRange));
    end
end
