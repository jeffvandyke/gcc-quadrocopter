
% this script will read from serial port COM7
% written by Aaron Derstine
try
    % create serial object on COM7
    s1 = serial('COM5');
    s1.BaudRate = 9600; % BaudRate of Arduino

    % open the serial port
    fopen(s1);

    disp('Opening connection on Port COM5');
    disp('Will read data until "StopTransmission" is sent');
    data = 0;
    stopCondition = 'StopTransmission';
    
    screenSize = get(0, 'ScreenSize');
    f = figure('OuterPosition', screenSize);
    
%     % add 3 subplots and return the handle for plotting
%     hPitch = subplot(3,1,1);
%     ylabel('Degrees');
%     title('Pitch Data');
%     legend('Accelerometer', 'Gyroscope', 'Filter', 'Location', 'Best');
%     hRoll = subplot(3,1,2);
%     ylabel('Degrees');
%     title('Roll Data');
%     legend('Accelerometer', 'Gyroscope', 'Filter', 'Location', 'Best');
%     hYaw = subplot(3,1,3);
%     ylabel('Degrees');
%     title('Yaw Data');
%     legend('Magnetometer', 'Gyroscope', 'Filter', 'Location', 'Best');
    
    %# Generate a blank line and return the line handle
    accelRollHandle = line(nan, nan); 
    accelPitchHandle = line(nan, nan); 
    gyroRollHandle = line(nan, nan); 
    gyroPitchHandle = line(nan, nan);
    gyroYawHandle = line(nan, nan);
    magYawHandle = line(nan, nan);
    filterRollHandle = line(nan, nan); 
    filterPitchHandle = line(nan, nan);
    filterYawHandle = line(nan, nan);
    
    % number of data reads expected
    dataPoints = 200;
    
    % preallocate that much space in the matricies
    AccelRollData = zeros(1,dataPoints);
    AccelPitchData = zeros(1,dataPoints);
    GyroRollData = zeros(1,dataPoints);
    GyroPitchData = zeros(1,dataPoints);
    GyroYawData = zeros(1,dataPoints);
    MagYawData = zeros(1,dataPoints); 
    FilterRollData = zeros(1,dataPoints);
    FilterPitchData = zeros(1,dataPoints);
    FilterYawData = zeros(1,dataPoints);
    
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
            % Data: ar, ap, gr, gp, gy, my, fr, fp, fy
            D = sscanf(data, 'Data: %f, %f, %f, %f, %f, %f, %f, %f, %f');
            AccelRollData(:,index) = D(1);
            AccelPitchData(:,index) = D(2);
            GyroRollData(:,index) = D(3);
            GyroPitchData(:,index) = D(4);
            GyroYawData(:,index) = D(5);
            MagYawData(:,index) = D(6);
            FilterRollData(:,index) = D(7);
            FilterPitchData(:,index) = D(8);
            FilterYawData(:,index) = D(9); 
            
            % plot the results
%            axes(hRoll);
%             line('XData', 1:index, 'YData', AccelRollData(1:index), 'Color', 'b');  
%             line('XData', 1:index, 'YData', GyroRollData(1:index), 'Color', 'g');
%             line('XData', 1:index, 'YData', FilterRollData(1:index), 'Color', 'r');
%             axes(hPitch);
%             line('XData', 1:index, 'YData', AccelPitchData(1:index), 'Color', 'b');  
%             line('XData', 1:index, 'YData', GyroPitchData(1:index), 'Color', 'g');
%             line('XData', 1:index, 'YData', FilterPitchData(1:index), 'Color', 'r');
%             axes(hYaw);
%             line('XData', 1:index, 'YData', MagYawData(1:index), 'Color', 'b');  
%             line('XData', 1:index, 'YData', GyroYawData(1:index), 'Color', 'g');
%             line('XData', 1:index, 'YData', FilterYawData(1:index), 'Color', 'r');
        end
    end

    disp('Finished Reading Data');
    
    % plot the data  
    x = 1:dataPoints;
    figure(1);
    plot(x, AccelPitchData, 'b', x, GyroPitchData, 'g', x, FilterPitchData, 'r');
    ylabel('Degrees');
    title('Pitch Data');
    legend('Accelerometer', 'Gyroscope', 'Filter', 'Location', 'Best');
    figure(2);
    plot(x, AccelRollData, 'b', x, GyroRollData, 'g', x, FilterRollData, 'r');
    ylabel('Degrees');
    title('Roll Data');
    legend('Accelerometer', 'Gyroscope', 'Filter', 'Location', 'Best');
    figure(3);
    plot(x, MagYawData, 'b', x, GyroYawData, 'g', x, FilterYawData, 'r');
    ylabel('Degrees');
    title('Yaw Data');
    legend('Magnetometer', 'Gyroscope', 'Filter', 'Location', 'Best'); 
    
    DeltaTimeData
    mean(DeltaTimeData)
    

% catch any exceptions so program doesn't quit
% ensures that the port gets closed
catch err   
    disp(err);
end

%close port and delete object
fclose(s1);
delete(s1);

clear s1;