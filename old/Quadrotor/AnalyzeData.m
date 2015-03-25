
% this script will read the data from a file
% and then plot it
% written by Aaron Derstine

% prompt the user to select a file
[FileName,PathName,FilterIndex] = uigetfile('.txt','Windows Explorer','..\Test Data');

% Analyze!!
PlotMultipleTests(strcat(PathName, FileName));

