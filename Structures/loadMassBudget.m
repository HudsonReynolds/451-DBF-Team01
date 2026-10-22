function massTable = loadMassBudget(params)
%LOADMASSBUDGET  MassBudget component stations, scaled to the as-built mass.
%
% The MassBudget sheet sets WHERE the mass is (component X/Z stations, so
% the CG), while params.performance.MTOM (the measured as-built mass from
% the Inputs sheet) sets HOW MUCH there is. Every component mass is scaled
% by the same factor so the table sums to MTOM without moving the CG.

massTable = readtable('SizingParams.xlsx', 'Sheet', 'MassBudget');
scale = 1000*params.performance.MTOM / sum(massTable.Mass_g);
massTable.Mass_g = scale * massTable.Mass_g;
end
