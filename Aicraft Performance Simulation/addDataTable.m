function addDataTable(ax, colNames, data, varargin)
%ADDDATATABLE  Draw a multi-column data table into an axes.
%
%   addDataTable(ax, colNames, data) fills the (otherwise blank) axes ax
%   with a plain table: a bold header row (colNames, 1xM cell of
%   strings), a rule, then one row per entry of data (NxM cell array of
%   already-formatted strings), the first column left-aligned and every
%   other column right-aligned, with a closing rule -- same plain,
%   no-gridlines style as the AAE 451 Aircraft Performance lecture's own
%   table slides.
%
%   addDataTable(..., 'CellColor', colorFn) additionally colours each data
%   cell's text by calling colorFn(r, c, data{r,c}), which must return an
%   RGB triple -- used e.g. to colour a Requirements table's Outcome
%   column by Verified/Failed/Open.
%
%   addDataTable(..., 'ColAlign', align) overrides the default alignment;
%   align is a 1xM cell of 'left'/'right'/'center'.
%
%   Drawn with axes text/line objects, not a uitable, so it exports
%   cleanly with exportgraphics/saveas like any other figure content.

p = inputParser;
addParameter(p, 'CellColor', []);
addParameter(p, 'ColAlign', {});
parse(p, varargin{:});
colorFn = p.Results.CellColor;
colAlign = p.Results.ColAlign;

[nRows, nCols] = size(data);
if isempty(colAlign)
    colAlign = [{'left'}, repmat({'right'}, 1, nCols-1)];
end

axes(ax); %#ok<LAXES>
cla(ax);
xlim(ax, [0 1]); ylim(ax, [0 1]);
axis(ax, 'off');
hold(ax, 'on');

% Column x-positions: first column starts at the left edge, the rest are
% evenly spaced and right/left/center-anchored within their own slot.
colX = linspace(0, 1, nCols+1);
colLeft = colX(1:end-1);
colRight = colX(2:end) - 0.01;
colCenter = (colLeft+colRight)/2;

nDataRows = nRows;
topY = 0.95;           % header baseline
headerRuleY = 0.88;    % rule under the header
botY = 0.03;            % bottom rule
if nDataRows > 0
    rowY = linspace(headerRuleY - 0.07, botY + 0.03, nDataRows);
else
    rowY = [];
end

ruleColor = [0.25 0.25 0.25];
plot(ax, [0 1], [topY+0.045 topY+0.045], '-', 'Color', ruleColor, 'LineWidth', 1.2);
plot(ax, [0 1], [headerRuleY headerRuleY], '-', 'Color', ruleColor, 'LineWidth', 0.8);
plot(ax, [0 1], [botY-0.02 botY-0.02], '-', 'Color', ruleColor, 'LineWidth', 1.2);

local_colX = @(align, c) local_pick(align, colLeft(c), colRight(c), colCenter(c));

for c = 1:nCols
    text(ax, local_colX(colAlign{c}, c), topY, colNames{c}, 'FontSize', 10.5, 'FontWeight', 'bold', ...
        'Interpreter', 'none', 'HorizontalAlignment', colAlign{c}, 'VerticalAlignment', 'middle', 'Color', [0.05 0.05 0.05]);
end

for r = 1:nDataRows
    for c = 1:nCols
        clr = [0.10 0.10 0.10];
        if ~isempty(colorFn)
            clr = colorFn(r, c, data{r,c});
        end
        text(ax, local_colX(colAlign{c}, c), rowY(r), data{r,c}, 'FontSize', 10, ...
            'Interpreter', 'none', 'HorizontalAlignment', colAlign{c}, 'VerticalAlignment', 'middle', 'Color', clr);
    end
end

end

function x = local_pick(align, leftX, rightX, centerX)
    switch align
        case 'left', x = leftX;
        case 'right', x = rightX;
        otherwise, x = centerX;
    end
end
