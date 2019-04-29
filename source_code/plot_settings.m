function plot_settings(f,l)
% Funtion to setup the plot setting
%
% Inputs:
%   f       font size
%   l       plot line width
%
%--------------------------------------------------------------------------

set(groot,'defaultLineLineWidth','remove');
set(0,'DefaultAxesFontSize','remove');
set(0,'defaultLegendInterpreter','remove');
set(0,'defaultTextInterpreter','remove');
set(0,'defaultAxesTickLabelInterpreter','remove');
fontSize = f;
lineWidth = l;
grayColor = [0.8 0.8 0.8];
set(0,'defaultLegendInterpreter','latex');
set(0,'defaultTextInterpreter','latex');
set(0,'DefaultAxesFontSize',fontSize);