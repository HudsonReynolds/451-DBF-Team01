function Setup()

% system parameters are all pulled from an Excel spreadsheet. See
% 'SizingParams.xlsx' for more details:

% set plotting up:
set(groot,'defaultLineLineWidth',1.5)
set(groot,'defaultFunctionLineLineWidth',1.5)

% set the interpreter to latex
set(groot, 'defaultAxesTickLabelInterpreter','latex'); 
set(groot, 'defaultLegendInterpreter','latex');
set(groot, 'defaultTextInterpreter', 'latex');

%change font sizes
set(groot, 'defaultLegendFontSize', 12)
set(groot, 'defaultTextFontSize', 16)
set(groot, 'defaultAxesFontSize', 12)

% grid on and box off
set(groot, 'defaultLegendBox', 'off');
set(groot,'defaultAxesXGrid','on');
set(groot,'defaultAxesYGrid','on');

% setup the path to include all subfolders
addpath(genpath(pwd))


end