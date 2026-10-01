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
    for i = 1:numel(figs)
        f = figs(i);
        name = get(f, 'Name');
        if isempty(name)
            name = sprintf('Figure_%d', f.Number);
        end
        safeName = regexprep(name, '[^a-zA-Z0-9]+', '_');
        exportgraphics(f, fullfile(outDir, [safeName '.png']), 'Resolution', 200);
    end

    fprintf('\nSaved %d figure(s) to ''%s''.\n', numel(figs), outDir);
end
