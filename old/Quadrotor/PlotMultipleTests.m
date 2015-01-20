% this script will read the data from a file
% and then plot it
% written by Aaron Derstine and Zachary Wontrop

function [] = PlotMultipleTests(filename)

% open the selected file
fileID = fopen(filename);
if (fileID == -1)
    msgbox('Error opening file');
    return
end

count = 1;
numTests = sscanf(fgets(fileID), 'Tests: %f');

fig = figure;
set(fig, 'units', 'normalized', 'outerposition', [0 0 1 1]);
% create an array of axes handles for each of the tests
% used to scroll through the tests
a_handle = [subplot(4,1,1); subplot(4,1,2); subplot(4,1,3); subplot(4,1,4)];
set(findobj(a_handle(1,1)),'Visible','off');
set(findobj(a_handle(2,1)),'Visible','off');
set(findobj(a_handle(3,1)),'Visible','off');
set(findobj(a_handle(4,1)),'Visible','off');
while( count <= numTests) 

    % first line is the test name that got the data
    testName = strtrim(fgets(fileID));

    % second line is the format of the data - used in the sscanf function
    format = fgets(fileID);

    % third line is the test settings values
    testSettings = strtrim(fgets(fileID));

    % find how many rows of data there will be and create a vector of that size
    % it will be used to append to as we read the file
    k = strfind(format, '%f');
    Data = ones(length(k),1);

    % read the file data
    line = strtrim(fgets(fileID));
    while (ischar(line) && (strcmp('Complete Test', line) == 0))
       D = sscanf(line, format);
       Data = [Data, D];
       line = strtrim(fgets(fileID));
    end

    % remove that first useless column that we originally made
    Data(:,1) = [];
    
    % plot the data - use the names since that way we know how the data is formatted
    if (strcmp(testName,'Pitch Axis Test'))
        % top plot is normal pitch data
        h1 = axes('position',get(a_handle(1,1),'position'));
        plot(h1, Data(1,:),Data(2,:),Data(1,:),Data(3,:));
        title(testSettings);
        xlabel('Sample Number (15ms)');
        ylabel('Pitch Angle (degrees)');
        
        % bottom plot is motor values
        h2 = axes('position',get(a_handle(2,1),'position'));
        plot(h2, Data(1,:),Data(4,:),Data(1,:),Data(5,:));
        xlabel('Sample Number (15ms)');
        ylabel('Motor Speed');
        a_handle = [a_handle, [h1; h2]];
    elseif (strcmp(testName,'Roll/Pitch Test'))
        % top plot is roll data
        h1 = axes('position',get(a_handle(1,1),'position'));
        plot(h1, Data(1,:),Data(2,:),Data(1,:),Data(4,:));
        title(testSettings);
        xlabel('Sample Number (15ms)');
        ylabel('Roll Angle (degrees)');
        
        % bottom plot is pitch data
        h2 = axes('position',get(a_handle(2,1),'position'));
        plot(h2, Data(1,:),Data(3,:),Data(1,:),Data(4,:));
        title('Pitch values');
        xlabel('Sample Number (15ms)');
        ylabel('Pitch Angle (degrees)');
        a_handle = [a_handle, [h1; h2]];
        
    elseif (strcmp(testName,'Altitude Test'))
        % top plot is normal pitch data
        h1 = axes('position',get(a_handle(1,1),'position'));
        plot(h1, Data(1,:),Data(2,:),Data(1,:),Data(3,:));
        title(testSettings);
        xlabel('Sample Number (15ms)');
        ylabel('Altitude (inches)');
        
    elseif (strcmp(testName,'Full Flight Test'))
        % top plot is roll data
        h1 = axes('position',get(a_handle(1,1),'position'));
        plot(h1, Data(1,:),Data(2,:),Data(1,:),Data(4,:));
        title(testSettings);
        xlabel('Sample Number (15ms)');
        ylabel('Roll Angle (degrees)');
        
        % 2nd plot is pitch data
        h2 = axes('position',get(a_handle(2,1),'position'));
        plot(h2, Data(1,:),Data(3,:),Data(1,:),Data(4,:));
        title('Pitch values');
        xlabel('Sample Number (15ms)');
        ylabel('Pitch Angle (degrees)');
        
        % 3rd plot is yaw data
        h3 = axes('position',get(a_handle(3,1),'position'));
        plot(h3, Data(1,:),Data(5,:),Data(1,:),Data(6,:));
        title('Yaw values');
        xlabel('Sample Number (15ms)');
        ylabel('Yaw Angle (degrees)');
        
        % 4th plot is altitude data
        h4 = axes('position',get(a_handle(4,1),'position'));
        plot(h4, Data(1,:),Data(7,:),Data(1,:),Data(8,:));
        title('Altitude values');
        xlabel('Sample Number (15ms)');
        ylabel('Altitude Height (inches)');
        a_handle = [a_handle, [h1; h2; h3; h4]];
    elseif (strcmp(testName,'Some Other Name'))
        % add additional tests
    else
        msgbox('No title match');
    end
    count = count + 1;
end % Loop for each data set. X number of times

% close the file
fclose(fileID);

% remove the first column that doesn't contain an actual plot
a_handle(:,1) = [];
% turn off all plots except 1st one
for i=2:numTests
    set(findobj(a_handle(1,i)),'Visible','off');
    set(findobj(a_handle(2,i)),'Visible','off');
    set(findobj(a_handle(3,i)),'Visible','off');
    set(findobj(a_handle(4,i)),'Visible','off');
end

% scroll through the different plots with the arrow keys
% based off code found at:
% http://www.mathworks.com/matlabcentral/answers/100024
currentAxis = 1;
set(fig,'WindowKeyPressFcn',@scrollaxes)
function scrollaxes(src,evt)
    if strcmp(evt.Key,'leftarrow')
        set(findobj(a_handle(1,currentAxis)),'Visible','off');
        set(findobj(a_handle(2,currentAxis)),'Visible','off');
        set(findobj(a_handle(3,currentAxis)),'Visible','off');
        set(findobj(a_handle(4,currentAxis)),'Visible','off');
        if (currentAxis > 1)
            currentAxis = currentAxis - 1;
        else %currentAxis == 1
            currentAxis = numTests;
        end     
        set(findobj(a_handle(1,currentAxis)),'Visible','on');
        set(findobj(a_handle(2,currentAxis)),'Visible','on');
        set(findobj(a_handle(3,currentAxis)),'Visible','on');
        set(findobj(a_handle(4,currentAxis)),'Visible','on');
        %legend('Motor 1', 'Motor 3', 'Location','Best');
    elseif strcmp(evt.Key,'rightarrow')
        set(findobj(a_handle(1,currentAxis)),'Visible','off');
        set(findobj(a_handle(2,currentAxis)),'Visible','off');
        set(findobj(a_handle(3,currentAxis)),'Visible','off');
        set(findobj(a_handle(4,currentAxis)),'Visible','off');
        if (currentAxis < numTests)
            currentAxis = currentAxis + 1;
        else %currentAxis == numTests
            currentAxis = 1;
        end     
        set(findobj(a_handle(1,currentAxis)),'Visible','on');
        set(findobj(a_handle(2,currentAxis)),'Visible','on');
        set(findobj(a_handle(3,currentAxis)),'Visible','on');
        set(findobj(a_handle(4,currentAxis)),'Visible','on');
        %legend('Motor 1', 'Motor 3', 'Location','Best');
    end

end % event handler
end