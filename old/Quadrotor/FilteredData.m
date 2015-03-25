
% this script will read from serial port COM7
% written by Aaron Derstine
try
    % create serial object on COM5
    s1 = serial('COM5');
    s1.BaudRate = 115200; % BaudRate of Arduino

    % open the serial port
    fopen(s1);

    disp('Opening connection on Port COM5');
    disp('Will read data until "StopTransmission" is sent');
    data = 0;
    stopCondition = 'StopTransmission';
    
    screenSize = get(0, 'ScreenSize');
    f = figure('OuterPosition', screenSize);
    
    %# Generate a blank line and return the line handle
    accelLineHandle = line(nan, nan);  
    gyroLineHandle = line(nan, nan); 
    filterLineHandle = line(nan, nan); 
    
    % number of data reads expected
    dataPoints = 1000;
    
    % preallocate that much space in the matricies
    AccelData = zeros(1,dataPoints);
    GyroData = zeros(1,dataPoints);
    FilterData = zeros(1,dataPoints);
    
    % current index for entering data
    index = 0;

    % will keep reading until stopCondition is sent
    while (isempty(strfind(data,stopCondition)))
        data = fscanf(s1);
        %disp(data);
        % Sensor Data
        if (strfind(data, 'Data'))
            index = index + 1;
            % pull out data from the stream
            % Data: ad, gd, fd
            D = sscanf(data, 'Data: %f, %f, %f');
            AccelData(:,index) = D(1);
            GyroData(:,index) = D(2);
            FilterData(:,index) = D(3);
            
            % plot the results
%             line('XData', 1:index, 'YData', AccelData(1:index), 'Color', 'b');  
%             line('XData', 1:index, 'YData', GyroData(1:index), 'Color', 'g');
%             line('XData', 1:index, 'YData', FilterData(1:index), 'Color', 'r');
%             drawnow
        end
    end

    disp('Finished Reading Data');
    
    % plot the data  
    x = 1:dataPoints;
    figure(1);
    plot(x, AccelData, 'b', x, GyroData, 'g', x, FilterData, 'r');
    ylabel('Degrees');
    title('Roll Data');
    legend('Accelerometer', 'Gyroscope', 'Filter', 'Location', 'Best');    

% catch any exceptions so program doesn't quit
% ensures that the port gets closed
catch err   
    disp(err);
end

%close port and delete object
fclose(s1);
delete(s1);

clear s1;