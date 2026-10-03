function ExportAllFigures(outDir)
%EXPORTALLFIGURES  Save every currently open figure as a PNG in outDir.
%
%   ExportAllFigures('Plots') saves every figure window open at the time
%   it's called (i.e. everything Main.m has generated so far) into
%   outDir, one PNG per figure, named after that figure's own 'Name'.
%   Any *.png already in outDir is deleted first, so the folder always
%   reflects exactly this run's figures -- nothing stale lingers if a
%   figure is renamed, removed, or added later.

    if ~exist(outDir, 'dir')
        mkdir(outDir);
    else
        old = dir(fullfile(outDir, '*.png'));
        for i = 1:numel(old)
            delete(fullfile(outDir, old(i).name));
        end
    end

    figs = findall(groot, 'Type', 'figure');
    saved = 0;
    for i = 1:numel(figs)
        f = figs(i);
        if ~isvalid(f)
            % Can happen when running interactively rather than in
            % batch: a figure window got closed (by hand, or by another
            % script) after findall collected its handle but before this
            % loop reached it. Skip it rather than letting one closed
            % window crash the export of everything else.
            continue
        end
        try
            name = get(f, 'Name');
        catch
            continue % closed between the isvalid check above and this call
        end
        if isempty(name)
            name = sprintf('Figure_%d', f.Number);
        end
        safeName = regexprep(name, '[^a-zA-Z0-9]+', '_');
        try
            exportgraphics(f, fullfile(outDir, [safeName '.png']), 'Resolution', 200);
            saved = saved + 1;
        catch ME
            warning('ExportAllFigures:skipped', 'Could not export figure "%s": %s', name, ME.message);
        end
    end

    fprintf('\nSaved %d of %d figure(s) to ''%s''.\n', saved, numel(figs), outDir);
end
