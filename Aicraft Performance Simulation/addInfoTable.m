function addInfoTable(ax, rows)
%ADDINFOTABLE  Draw a compact label/value table into an axes.
%
%   addInfoTable(ax, rows) fills the (otherwise blank) axes ax with a
%   plain two-column table: rows{:,1} left-aligned labels, rows{:,2}
%   right-aligned values, one row per pair, a thin rule above the first
%   row and below the last, no header -- the same compact key-value panel
%   style the AAE 451 Aircraft Performance lecture notes place beside
%   each example figure (e.g. "Ground roll / 16.53 m", "RFP limit / 25 m",
%   ... stacked top to bottom next to the plot).
%
%   Typical use: put the real plot in a wider tile of a tiledlayout and
%   call this on the narrow tile beside it.
%
%       t = tiledlayout(1, 3);
%       nexttile(t, [1 2]); plot(...);
%       addInfoTable(nexttile(t), {'Ground roll', '16.53 m'; 'RFP limit', '25 m'});

axes(ax); %#ok<LAXES>
cla(ax);
xlim(ax, [0 1]); ylim(ax, [0 1]);
axis(ax, 'off');
hold(ax, 'on');

n = size(rows, 1);
topY = 0.94;
botY = 0.04;
if n > 1
    rowY = linspace(topY, botY, n);
    rowPitch = (topY - botY) / (n - 1);
else
    rowY = (topY + botY) / 2;
    rowPitch = topY - botY;
end
ruleY_top = min(topY + rowPitch*0.45, 0.99);
ruleY_bot = max(botY - rowPitch*0.45, 0.01);

ruleColor = [0.35 0.35 0.35];
plot(ax, [0 1], [ruleY_top ruleY_top], '-', 'Color', ruleColor, 'LineWidth', 1.0);
plot(ax, [0 1], [ruleY_bot ruleY_bot], '-', 'Color', ruleColor, 'LineWidth', 1.0);

for i = 1:n
    % Interpreter explicitly 'none': labels/values are plain text (not
    % LaTeX), and some MATLAB/figure configurations otherwise truncate a
    % literal '%' as a comment character or print '\' escapes raw instead
    % of rendering them -- 'none' sidesteps that regardless of cause.
    text(ax, 0.00, rowY(i), rows{i,1}, 'FontSize', 11, 'Interpreter', 'none', ...
        'HorizontalAlignment', 'left', 'VerticalAlignment', 'middle', 'Color', [0.15 0.15 0.15]);
    text(ax, 1.00, rowY(i), rows{i,2}, 'FontSize', 11, 'FontWeight', 'bold', 'Interpreter', 'none', ...
        'HorizontalAlignment', 'right', 'VerticalAlignment', 'middle', 'Color', [0 0.2 0.45]);
end

end
