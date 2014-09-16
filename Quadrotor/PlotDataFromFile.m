
% this script will read the data from a file
% and then plot it
% written by Aaron Derstine

% prompt the user to select a file
[FileName,PathName,FilterIndex] = uigetfile('.txt','Windows Explorer','..\Test Data');

% open the selected file
fileID = fopen(strcat(PathName, FileName));
if (fileID == -1)
    msgbox('Error opening file');
    return
end

% first line is the test name that got the data
testName = strtrim(fgets(fileID));


% second line is the format of the data - used in the sscanf function
format = fgets(fileID);

% third line is the PID constant values
pidSettings = strtrim(fgets(fileID));

% find how many rows of data there will be and create a vector of that size
% it will be used to append to as we read the file
k = strfind(format, '%f');
Data = ones(length(k),1);

% read the file data
line = fgets(fileID);
while (ischar(line))
   D = sscanf(line, format);
   Data = [Data, D];
   line = fgets(fileID);
end

% close the file
fclose(fileID);

% remove that first useless column that we originally made
Data(:,1) = [];

figure
% plot the data - use the names since that way we know how the data is formatted
if (strcmp(testName,'Pitch Axis Test'))
    plot(Data(1,:),Data(2,:),Data(1,:),Data(3,:))
    title(pidSettings);
    %legend(pidSettings,0);
    xlabel('Sample Number (15ms)');
    ylabel('Pitch Angle (degrees)');
elseif (strcmp(testName,'Pitch Axis Test With Motor Values'))
    % top plot is normal pitch data
    subplot(2,1,1)
    plot(Data(1,:),Data(2,:),Data(1,:),Data(3,:))
    title(pidSettings);
    xlabel('Sample Number (15ms)');
    ylabel('Pitch Angle (degrees)');
    % bottom plot is motor values
    subplot(2,1,2)
    plot(Data(1,:),Data(4,:),Data(1,:),Data(5,:),Data(1,:),Data(6,:))
    xlabel('Sample Number (15ms)');
    ylabel('Motor Speed');
    legend('Motor 1', 'Motor 3', 'PID Output', 0);
elseif (strcmp(testName,'Some Other Name'))
    % add additional tests
else
    msgbox('No title match');
end

