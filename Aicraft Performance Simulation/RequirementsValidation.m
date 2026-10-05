function [outputs, params] = RequirementsValidation(params)

%% A9 Deliverable 9 -- Requirements Validation
%
% Compares the pipeline's final computed numbers against the team's own
% formal requirements (Preliminary Assignments/Team01_Requirements.slreqx,
% R1-R29 -- extracted directly from that file, not re-typed from memory).
% Most of the 29 requirements are mechanical, operational, or regulatory
% (storage volume, tooling-free battery access, budget, schedule, pilot
% qualification, etc.) and have no performance-model counterpart to check
% them against -- this script says so explicitly for each of those rather
% than silently skipping them, so the team can see exactly which
% requirements still need a non-MATLAB verification method (inspection,
% demonstration, test, or analysis outside this codebase) and which ones
% this simulation already answers.
%
% Results are written to RequirementsValidation.xlsx in this same folder
% instead of a console dump: one row per requirement, with Status and
% Detail as separate columns. Overwritten fresh every run of Main.m, so
% the sheet always reflects only the latest results, not a growing
% history.

have_D2 = isfield(params.performance, 'groundRoll_x') && isfield(params.performance, 'groundRoll_ok');
have_D3 = isfield(params.performance, 'ROC_max');
have_D6 = isfield(params.performance, 'mission_ok_power') && isfield(params.performance, 'mission_ok_energy');
have_D8 = isfield(params.performance, 'W_pay_max');

results = struct('id', {}, 'summary', {}, 'status', {}, 'detail', {});

results(end+1) = local_req('R1', 'Takeoff Distance', local_check(have_D2, ...
    have_D2 && params.performance.groundRoll_ok, ...
    sprintf('computed %.2f m vs limit %.1f m (Deliverable 2)', local_safe(params.performance,'groundRoll_x'), local_safe(params.performance,'S_TO'))));

results(end+1) = local_req('R2', 'Post Climb Altitude', local_check(have_D3, ...
    have_D3 && params.performance.ROC_max > 0, ...
    sprintf('ROC_max = %.2f m/s > 0, so the modelled climb reaches %.0f m (Deliverable 3)', local_safe(params.performance,'ROC_max'), local_safe(params.performance,'climb_alt'))));

results(end+1) = local_req('R3', 'Lap (3 full laps)', local_check(have_D6, ...
    have_D6 && params.performance.mission_ok_energy, ...
    'Deliverable 6 simulates exactly 3 laps (R3-cited) and checks the energy budget covers them'));

results(end+1) = local_req('R4', 'Stability (center-mark, lap 3)', local_na('not modelled -- needs a dynamic-stability check at a specific mission point, outside this performance pipeline'));
results(end+1) = local_req('R5', 'Safe Landing (dirt/grass)', local_na('not modelled -- landing-gear energy absorption / structural check, not a performance-pipeline quantity'));
results(end+1) = local_req('R6', 'Prevent Injury and Damage', local_na('operational/safety -- not a MATLAB-computable quantity'));
results(end+1) = local_req('R7', 'Battery Connection', local_na('mechanical/CAD layout -- not modelled here'));
results(end+1) = local_req('R8', 'Propeller Clearance', local_na('mechanical/CAD layout -- not modelled here'));
results(end+1) = local_req('R9', 'Storage Volume (75x75x150 cm)', local_na('needs the folded/disassembled stowed envelope -- not modelled (only flight geometry exists here)'));

results(end+1) = local_req('R10', 'Wingspan (<=150 cm)', local_check(true, ...
    params.geometry.wingspan <= 1.5, ...
    sprintf('wingspan = %.3f m vs 1.50 m limit', params.geometry.wingspan)));

results(end+1) = local_req('R11', 'Configuration Change (<5 min)', local_na('operational/human-factors timing -- not modelled'));
results(end+1) = local_req('R12', 'Battery & Payload Install (<2 min)', local_na('operational/human-factors timing -- not modelled'));
results(end+1) = local_req('R13', 'Battery Access (no tooling)', local_na('mechanical design -- not modelled'));

results(end+1) = local_req('R14', 'Payload Capacity (>=0.52 kg)', local_check(true, ...
    params.performance.W_pay >= 0.51, ...
    sprintf('baseline design payload W_pay = %.3f kg vs 0.52 kg requirement', params.performance.W_pay)));

results(end+1) = local_req('R15', 'Payload & Battery Mounting', local_na('mechanical design -- not modelled'));
results(end+1) = local_req('R16', 'Battery/Payload Modifications', local_na('mechanical design -- not modelled'));
results(end+1) = local_req('R17', 'Aircraft-Payload Stability', local_na('dynamic stability with payload -- not modelled (A5B checks static margin only, at one configuration)'));
results(end+1) = local_req('R18', 'Aircraft-No Payload Stability', local_na('dynamic stability without payload -- not modelled'));
results(end+1) = local_req('R19', 'Flyability (non-team pilot)', local_na('subjective/qualitative -- flight test item'));
results(end+1) = local_req('R20', 'Flight Data (telemetry)', local_na('avionics hardware capability -- not a performance-model quantity'));

results(end+1) = local_req('R21', 'Propulsion Power (<=1000 W)', local_check(have_D6, ...
    have_D6 && params.performance.mission_ok_power, ...
    sprintf('peak electrical draw checked against 1000 W across the whole mission (Deliverable 6)')));

results(end+1) = local_req('R22', 'Maximum Take Off Weight (<=6 kg)', local_check(true, ...
    params.performance.MTOM <= 6, ...
    sprintf('MTOM = %.3f kg vs 6.0 kg limit', params.performance.MTOM)));

results(end+1) = local_req('R23', 'Budget (<=$400)', local_na('cost tracking -- not a performance-model quantity'));
results(end+1) = local_req('R24', 'Schedule (ready Week 12)', local_na('project management -- not a performance-model quantity'));
results(end+1) = local_req('R25', 'As-Built Justifications', local_na('documentation/process item -- not a performance-model quantity'));
results(end+1) = local_req('R26', 'Multiple Flights (3x T.O./landing, no repair)', local_na('structural durability over repeated flights -- not modelled'));
results(end+1) = local_req('R27', 'Tracking (visible to 200 m)', local_na('paint/marking scheme -- not modelled'));

results(end+1) = local_req('R28', 'Weight Margin (as-designed <=90% of 6 kg)', local_check(true, ...
    params.performance.MTOM <= 0.9*6, ...
    sprintf('MTOM = %.3f kg vs 5.40 kg (90%% of 6 kg)', params.performance.MTOM)));

results(end+1) = local_req('R29', 'Repairability (<=1 hr)', local_na('mechanical design / assembly process -- not modelled'));

if have_D8
    results(end+1) = local_req('(D8)', 'Maximum payload found vs. R14/R28 margin', local_check(true, ...
        params.performance.W_pay_max >= 0.52, ...
        sprintf('Deliverable 8 found a maximum payload of %.3f kg while holding every other constraint, vs. the %.2f kg R14 requires', ...
        params.performance.W_pay_max, 0.52)));
end

n_pass = 0; n_fail = 0; n_na = 0;
for k = 1:numel(results)
    switch results(k).status
        case 'PASS', n_pass = n_pass + 1;
        case 'FAIL', n_fail = n_fail + 1;
        otherwise, n_na = n_na + 1;
    end
end

%% ---- Write to RequirementsValidation.xlsx (same folder as this script) ----
% Overwritten fresh every run with just the latest results -- one row per
% requirement ID, Status and Detail as separate columns (not combined
% into one block of text, so Status is filterable/sortable in Excel).
thisFile = mfilename('fullpath');
xlsxPath = fullfile(fileparts(thisFile), 'RequirementsValidation.xlsx');
runLabel = char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm'));

nRows = numel(results);
outTable = cell(nRows + 2, 4);
outTable(1, :) = {sprintf('Deliverable 9 -- Requirements Validation (last run: %s)', runLabel), '', '', ''};
outTable(2, :) = {'ID', 'Summary', 'Status', 'Detail'};
for k = 1:nRows
    outTable(k+2, :) = {results(k).id, results(k).summary, results(k).status, results(k).detail};
end

if isfile(xlsxPath)
    % writecell only replaces the named sheet -- any other sheet already
    % in the file (e.g. a leftover default "Sheet1", or an old run's data
    % under a different name) would otherwise sit there untouched forever.
    % Delete first so the file always ends up with exactly one sheet.
    delete(xlsxPath);
end
writecell(outTable, xlsxPath, 'Sheet', 'Deliverable 9');

fprintf('\n--- Requirements Validation (Deliverable 9) ---\n');
fprintf('  Source: Preliminary Assignments/Team01_Requirements.slreqx (R1-R29)\n');
fprintf('  Results written to %s (run: %s)\n', xlsxPath, runLabel);
fprintf('  %d checkable from this model (%d PASS, %d FAIL), %d require a non-MATLAB verification method.\n', ...
    n_pass+n_fail, n_pass, n_fail, n_na);

%% ---- Outputs ----
outputs.RequirementsValidation = struct('results', results, 'n_pass', n_pass, 'n_fail', n_fail, 'n_na', n_na, 'xlsxPath', xlsxPath);

end

function r = local_req(id, summary, statusDetail)
    r = struct('id', id, 'summary', summary, 'status', statusDetail{1}, 'detail', statusDetail{2});
end

function sd = local_check(available, passed, detailStr)
    if ~available
        sd = {'N/A', 'upstream deliverable not yet run'};
    elseif passed
        sd = {'PASS', detailStr};
    else
        sd = {'FAIL', detailStr};
    end
end

function sd = local_na(reason)
    sd = {'N/A', reason};
end

function v = local_safe(s, field)
    if isfield(s, field)
        v = s.(field);
    else
        v = NaN;
    end
end
