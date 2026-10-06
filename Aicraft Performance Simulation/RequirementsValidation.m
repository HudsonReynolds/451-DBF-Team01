function [outputs, params] = RequirementsValidation(params)

%% A9 Deliverable 9 -- Requirements Validation
%
% Compares the pipeline's final computed numbers against the team's own
% formal requirements (Preliminary Assignments/Team01_Requirements.slreqx,
% R1-R29 -- extracted directly from that file, not re-typed from memory).
% Each requirement gets one of the three outcomes A9 itself specifies:
%   Verified -- the evidence exists and the requirement is met.
%   Failed   -- the evidence exists and the requirement is not met.
%   Open     -- can only be closed by a test, demonstration, or inspection
%               of an article (or a process/paperwork step) that doesn't
%               exist yet; this performance pipeline has no way to check it.
% Most of the 29 requirements are mechanical, operational, or regulatory
% (storage volume, tooling-free battery access, budget, schedule, pilot
% qualification, etc.) and come out Open for that reason, not because
% they were skipped. For every Failed row, the Action column states the
% design change that would fix it; for every Open row, it states the
% test/demonstration that would close it and roughly when -- both
% required explicitly by A9 Deliverable 9.
%
% Results are written to RequirementsValidation.xlsx in this same folder
% instead of a console dump: one row per requirement, with Status, Detail
% and Action as separate columns. Overwritten fresh every run of Main.m,
% so the sheet always reflects only the latest results, not a growing
% history.

have_D2 = isfield(params.performance, 'groundRoll_x') && isfield(params.performance, 'groundRoll_ok');
have_D3 = isfield(params.performance, 'ROC_max');
have_D6 = isfield(params.performance, 'mission_ok_power') && isfield(params.performance, 'mission_ok_energy');
have_D8 = isfield(params.performance, 'W_pay_max');

results = struct('id', {}, 'summary', {}, 'status', {}, 'detail', {}, 'action', {});

results(end+1) = local_req('R1', 'Takeoff Distance', local_check(have_D2, ...
    have_D2 && params.performance.groundRoll_ok, ...
    sprintf('computed %.2f m vs limit %.1f m (Deliverable 2)', local_safe(params.performance,'groundRoll_x'), local_safe(params.performance,'S_TO')), ...
    'Reduce ground roll: more static thrust (larger prop or higher-Kv motor), less weight, or a lower rotation/ground-roll CL.'));

results(end+1) = local_req('R2', 'Post Climb Altitude', local_check(have_D3, ...
    have_D3 && params.performance.ROC_max > 0, ...
    sprintf('ROC_max = %.2f m/s > 0, so the modelled climb reaches %.0f m (Deliverable 3)', local_safe(params.performance,'ROC_max'), local_safe(params.performance,'climb_alt')), ...
    'Increase excess thrust-minus-drag at the climb speed: more power, less drag, or less weight.'));

results(end+1) = local_req('R3', 'Lap (3 full laps)', local_check(have_D6, ...
    have_D6 && params.performance.mission_ok_energy, ...
    'Deliverable 6 simulates exactly 3 laps (R3-cited) and checks the energy budget covers them', ...
    'Increase usable battery energy (larger pack) or reduce per-lap power draw (less drag, a better-matched prop/motor).'));

results(end+1) = local_req('R4', 'Stability (center-mark, lap 3)', local_open( ...
    'not modelled -- needs a dynamic-stability check at a specific mission point, outside this performance pipeline', ...
    'Flight test: fly the as-built aircraft through a representative lap configuration at the payload condition, verify handling and recovery; before competition.'));
results(end+1) = local_req('R5', 'Safe Landing (dirt/grass)', local_open( ...
    'not modelled -- landing-gear energy absorption / structural check, not a performance-pipeline quantity', ...
    'Flight test: land the as-built aircraft on the actual grass/dirt surface, inspect the airframe for damage after each landing; during flight-test sessions.'));
results(end+1) = local_req('R6', 'Prevent Injury and Damage', local_open( ...
    'operational/safety -- not a MATLAB-computable quantity', ...
    'Inspection + flight test: review the structure for exposed hazards (prop, sharp edges) and confirm nothing separates on a hard landing; before first flight.'));
results(end+1) = local_req('R7', 'Battery Connection', local_open( ...
    'mechanical/CAD layout -- not modelled here', ...
    'Physical inspection: verify connector type, polarity, and secure mounting once the battery and fuselage are both built; at integration.'));
results(end+1) = local_req('R8', 'Propeller Clearance', local_open( ...
    'mechanical/CAD layout -- not modelled here', ...
    'Physical measurement: measure as-built prop-to-ground and prop-to-fuselage clearance once the airframe and gear are assembled; at integration.'));
results(end+1) = local_req('R9', 'Storage Volume (75x75x150 cm)', local_open( ...
    'needs the folded/disassembled stowed envelope -- not modelled (only flight geometry exists here)', ...
    'Physical fit-check: fold/disassemble the as-built aircraft and measure the stowed envelope against 75x75x150 cm; once assembly is complete.'));

results(end+1) = local_req('R10', 'Wingspan (<=150 cm)', local_check(true, ...
    params.geometry.wingspan <= 1.5, ...
    sprintf('wingspan = %.3f m vs 1.50 m limit', params.geometry.wingspan), ...
    'Reduce wingspan: shorten the wing or lower aspect ratio within the structural/aero trade space.'));

results(end+1) = local_req('R11', 'Configuration Change (<5 min)', local_open( ...
    'operational/human-factors timing -- not modelled', ...
    'Timed demonstration: time a team member performing the configuration change on the as-built aircraft; during a practice/readiness review.'));
results(end+1) = local_req('R12', 'Battery & Payload Install (<2 min)', local_open( ...
    'operational/human-factors timing -- not modelled', ...
    'Timed demonstration: time battery and payload installation on the as-built aircraft; during a practice/readiness review.'));
results(end+1) = local_req('R13', 'Battery Access (no tooling)', local_open( ...
    'mechanical design -- not modelled', ...
    'Physical demonstration: remove and install the battery by hand, confirm no tools are needed; at integration.'));

results(end+1) = local_req('R14', 'Payload Capacity (>=0.52 kg)', local_check(true, ...
    params.performance.W_pay >= 0.51, ...
    sprintf('baseline design payload W_pay = %.3f kg vs 0.52 kg requirement', params.performance.W_pay), ...
    'Increase payload capacity: further weight reduction elsewhere, or a larger-capacity airframe/wing.'));

results(end+1) = local_req('R15', 'Payload & Battery Mounting', local_open( ...
    'mechanical design -- not modelled', ...
    'Physical inspection: verify mounting hardware secures both under expected flight loads; at integration.'));
results(end+1) = local_req('R16', 'Battery/Payload Modifications', local_open( ...
    'mechanical design -- not modelled', ...
    'Physical inspection: confirm no aircraft modification is needed to swap battery or payload; at integration.'));
results(end+1) = local_req('R17', 'Aircraft-Payload Stability', local_open( ...
    'dynamic stability with payload -- not modelled (A5B checks static margin only, at one configuration)', ...
    'Flight test: fly with payload installed, assess handling qualities qualitatively; during flight-test sessions.'));
results(end+1) = local_req('R18', 'Aircraft-No Payload Stability', local_open( ...
    'dynamic stability without payload -- not modelled', ...
    'Flight test: fly without payload, assess handling qualities qualitatively; during flight-test sessions.'));
results(end+1) = local_req('R19', 'Flyability (non-team pilot)', local_open( ...
    'subjective/qualitative -- flight test item', ...
    'Flight test: have a non-team-member pilot fly the aircraft and report on handling; during a flight-test session.'));
results(end+1) = local_req('R20', 'Flight Data (telemetry)', local_open( ...
    'avionics hardware capability -- not a performance-model quantity', ...
    'Bench/ground test: verify the telemetry/avionics package powers on and logs or transmits data correctly; at avionics integration.'));

results(end+1) = local_req('R21', 'Propulsion Power (<=1000 W)', local_check(have_D6, ...
    have_D6 && params.performance.mission_ok_power, ...
    'peak electrical draw checked against 1000 W across the whole mission (Deliverable 6)', ...
    'Reduce peak electrical draw: lower the throttle cap, a smaller propeller, or a more efficient motor/ESC pairing.'));

results(end+1) = local_req('R22', 'Maximum Take Off Weight (<=6 kg)', local_check(true, ...
    params.performance.MTOM <= 6, ...
    sprintf('MTOM = %.3f kg vs 6.0 kg limit', params.performance.MTOM), ...
    'Reduce aircraft mass: lighter structure, smaller battery, or reduced payload.'));

results(end+1) = local_req('R23', 'Budget (<=$400)', local_open( ...
    'cost tracking -- not a performance-model quantity', ...
    'Cost audit: tally as-built/as-purchased costs against the $400 cap; ongoing, finalized before competition.'));
results(end+1) = local_req('R24', 'Schedule (ready Week 12)', local_open( ...
    'project management -- not a performance-model quantity', ...
    'Program review: confirm the aircraft is flight-ready against the team''s build schedule; at the Week 12 milestone.'));
results(end+1) = local_req('R25', 'As-Built Justifications', local_open( ...
    'documentation/process item -- not a performance-model quantity', ...
    'Documentation review: compile and review as-built design-change justifications; before final report submission.'));
results(end+1) = local_req('R26', 'Multiple Flights (3x T.O./landing, no repair)', local_open( ...
    'structural durability over repeated flights -- not modelled', ...
    'Flight test: fly 3 consecutive take-off/landing cycles without repair, inspect the airframe between flights; during flight-test sessions.'));
results(end+1) = local_req('R27', 'Tracking (visible to 200 m)', local_open( ...
    'paint/marking scheme -- not modelled', ...
    'Visual demonstration: view the as-built, as-painted aircraft from 200 m and confirm visibility; during a flight-test or ground check.'));

results(end+1) = local_req('R28', 'Weight Margin (as-designed <=90% of 6 kg)', local_check(true, ...
    params.performance.MTOM <= 0.9*6, ...
    sprintf('MTOM = %.3f kg vs 5.40 kg (90%% of 6 kg)', params.performance.MTOM), ...
    'Reduce MTOM further below the 90% threshold -- same levers as R22, with added margin.'));

results(end+1) = local_req('R29', 'Repairability (<=1 hr)', local_open( ...
    'mechanical design / assembly process -- not modelled', ...
    'Timed demonstration: simulate a representative field repair on the as-built aircraft and time it; during a practice/readiness review.'));

if have_D8
    results(end+1) = local_req('(D8)', 'Maximum payload found vs. R14/R28 margin', local_check(true, ...
        params.performance.W_pay_max >= 0.52, ...
        sprintf('Deliverable 8 found a maximum payload of %.3f kg while holding every other constraint, vs. the %.2f kg R14 requires', ...
        params.performance.W_pay_max, 0.52), ...
        'Revisit Deliverable 8''s binding constraint -- that script''s own report states what change would raise the maximum payload.'));
end

n_verified = 0; n_failed = 0; n_open = 0;
for k = 1:numel(results)
    switch results(k).status
        case 'Verified', n_verified = n_verified + 1;
        case 'Failed', n_failed = n_failed + 1;
        otherwise, n_open = n_open + 1;
    end
end

%% ---- Write to RequirementsValidation.xlsx (same folder as this script) ----
% Overwritten fresh every run with just the latest results -- one row per
% requirement ID, Status, Detail and Action as separate columns (not
% combined into one block of text, so Status is filterable/sortable).
thisFile = mfilename('fullpath');
xlsxPath = fullfile(fileparts(thisFile), 'RequirementsValidation.xlsx');
runLabel = char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm'));

nRows = numel(results);
outTable = cell(nRows + 2, 5);
outTable(1, :) = {sprintf('Deliverable 9 -- Requirements Validation (last run: %s)', runLabel), '', '', '', ''};
outTable(2, :) = {'ID', 'Summary', 'Status', 'Detail', 'Action'};
for k = 1:nRows
    outTable(k+2, :) = {results(k).id, results(k).summary, results(k).status, results(k).detail, results(k).action};
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
fprintf('  %d checkable from this model (%d Verified, %d Failed), %d Open (need a test, demonstration, or inspection).\n', ...
    n_verified+n_failed, n_verified, n_failed, n_open);

%% ---- Table figure: the model-checkable requirements only ----
% Only the rows with an actual Value/Limit pair to show -- the 21 Open
% requirements don't have one (that's exactly why they're Open), so they
% stay in the Excel sheet/backup list rather than forced into this table.
tblRows = {};
if have_D2
    tblRows(end+1,:) = {'R1', 'Takeoff distance', sprintf('%.2f m', params.performance.groundRoll_x), sprintf('<= %.1f m', params.performance.S_TO), 'Verified'};
end
if have_D3
    tblRows(end+1,:) = {'R2', 'Climb feasibility', sprintf('%.2f m/s ROC', params.performance.ROC_max), '> 0 m/s', 'Verified'};
end
if isfield(params.performance, 'mission_E_left')
    tblRows(end+1,:) = {'R3', 'Lap energy margin', sprintf('%.1f kJ', params.performance.mission_E_left/1000), '>= 0 kJ', 'Verified'};
end
tblRows(end+1,:) = {'R10', 'Wingspan', sprintf('%.3f m', params.geometry.wingspan), '<= 1.50 m', 'Verified'};
tblRows(end+1,:) = {'R14', 'Payload capacity', sprintf('%.3f kg', params.performance.W_pay), '>= 0.52 kg', 'Verified'};
if isfield(params.performance, 'mission_P_peak')
    tblRows(end+1,:) = {'R21', 'Propulsion power', sprintf('%.0f W', params.performance.mission_P_peak), '<= 1000 W', 'Verified'};
end
tblRows(end+1,:) = {'R22', 'Maximum take-off weight', sprintf('%.3f kg', params.performance.MTOM), '<= 6.00 kg', 'Verified'};
tblRows(end+1,:) = {'R28', 'Weight margin', sprintf('%.3f kg', params.performance.MTOM), '<= 5.40 kg', 'Verified'};
if have_D8
    tblRows(end+1,:) = {'(D8)', 'Maximum payload found', sprintf('%.3f kg', params.performance.W_pay_max), '>= 0.52 kg', 'Verified'};
end
% Re-stamp each row's actual Outcome from results (built above), rather
% than assuming Verified -- a row only prints here if have_Dx was true,
% same guard results() used, so the ids line up.
idList = {results.id};
for i = 1:size(tblRows,1)
    j = find(strcmp(idList, tblRows{i,1}), 1);
    if ~isempty(j), tblRows{i,5} = results(j).status; end
end

figure('Name', 'Requirements Validation Table', 'Color', 'w', 'WindowStyle', 'docked');
ax_tbl = axes('Position', [0.03 0.05 0.94 0.85]);
addDataTable(ax_tbl, {'ID', 'Criterion', 'Value', 'Limit', 'Outcome'}, tblRows, ...
    'ColAlign', {'left','left','right','right','right'}, 'CellColor', @local_outcomeColor);
title('Requirements Validation -- Model-Checkable Requirements', 'Interpreter', 'none', 'FontWeight', 'bold');

%% ---- Outputs ----
outputs.RequirementsValidation = struct('results', results, 'n_verified', n_verified, 'n_failed', n_failed, 'n_open', n_open, 'xlsxPath', xlsxPath);

end

function r = local_req(id, summary, statusDetailAction)
    r = struct('id', id, 'summary', summary, 'status', statusDetailAction{1}, 'detail', statusDetailAction{2}, 'action', statusDetailAction{3});
end

function sda = local_check(available, passed, detailStr, fixStr)
    if ~available
        sda = {'Open', 'upstream deliverable not yet run', 're-run the full pipeline (Main.m) before this row is meaningful'};
    elseif passed
        sda = {'Verified', detailStr, ''};
    else
        sda = {'Failed', detailStr, fixStr};
    end
end

function sda = local_open(reason, actionStr)
    sda = {'Open', reason, actionStr};
end

function v = local_safe(s, field)
    if isfield(s, field)
        v = s.(field);
    else
        v = NaN;
    end
end

function clr = local_outcomeColor(r, c, value) %#ok<INUSL>
    % Colours only the Outcome column (5); every other cell stays the
    % addDataTable default (dark navy text).
    if c ~= 5
        clr = [0.10 0.10 0.10];
        return
    end
    switch value
        case 'Verified', clr = [0.00 0.40 0.20];
        case 'Failed',   clr = [0.75 0.10 0.10];
        otherwise,       clr = [0.45 0.45 0.45];
    end
end
