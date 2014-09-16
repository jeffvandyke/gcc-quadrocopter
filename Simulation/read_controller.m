
% this script will read from serial port COM4
% written by Aaron Derstine
try
    % create serial object on COM4
    s1 = serial('COM4');
    s1.BaudRate = 9600; % BaudRate of Arduino

    % open the serial port
    fopen(s1);

    disp('Opening connection on Port COM4');
%     disp('Will read data until "StopTransmission" is sent');
%     data = 0;
%     stopCondition = 'StopTransmission';
%     
%     % number of data reads expected
%     dataPoints = 100;
%     
%     % preallocate that much space in the matricies
%     AccelData = zeros(3,dataPoints);
%     MagData = zeros(3,dataPoints);
%     GyroData = zeros(3,dataPoints);
%     
%     AccelRollData = zeros(1,dataPoints);
%     AccelPitchData = zeros(1,dataPoints);
%     GyroRollData = zeros(1,dataPoints);
%     GyroPitchData = zeros(1,dataPoints);
%     GyroYawData = zeros(1,dataPoints);
%     HeadingData = zeros(1,dataPoints);
%     
%     % current index for entering data
%     Aindex = 0;
%     Mindex = 0;
%     Gindex = 0;
%     
%     ARindex = 0;
%     APindex = 0;
%     GRindex = 0;
%     GPindex = 0;
%     GYindex = 0;
%     Hindex = 0;
    
    % will keep reading until stopCondition is sent
    while (true)
        data = fscanf(s1)
        %disp(data);
        % Sensor Data
%         if (strfind(data, 'Accelerometer'))
%             Aindex = Aindex + 1;
%             % pull out the data from the stream
%             A = sscanf(data, 'Accelerometer: %f, %f, %f');
%             % add to the matrix of data
%             AccelData(:,Aindex) = A;
%         elseif (strfind(data, 'Magnetometer'))
%             Mindex = Mindex + 1;
%             M = sscanf(data, 'Magnetometer: %f, %f, %f');
%             MagData(:,Mindex) = M;
%         elseif (strfind(data, 'Gyroscope'))
%             Gindex = Gindex + 1;
%             G = sscanf(data, 'Gyroscope: %f, %f, %f');
%             GyroData(:,Gindex) = G;
%         % Accel RPY
%         elseif (strfind(data, 'AccelRoll'))
%             ARindex = ARindex + 1;
%             AR = sscanf(data, 'AccelRoll: %f');
%             AccelRollData(:,ARindex) = AR;
%         elseif (strfind(data, 'AccelPitch'))
%             APindex = APindex + 1;
%             AP = sscanf(data, 'AccelPitch: %f');
%             AccelPitchData(:,APindex) = AP;
%         % Gyro RPY
%         elseif (strfind(data, 'GyroRoll'))
%             GRindex = GRindex + 1;
%             GR = sscanf(data, 'GyroRoll: %f');
%             GyroRollData(:,GRindex) = GR;
%         elseif (strfind(data, 'GyroPitch'))
%             GPindex = GPindex + 1;
%             GP = sscanf(data, 'GyroPitch: %f');
%             GyroPitchData(:,GPindex) = GP;
%         elseif (strfind(data, 'GyroYaw'))
%             GYindex = GYindex + 1;
%             GY = sscanf(data, 'GyroYaw: %f');
%             GyroYawData(:,GYindex) = GY;
%         % Heading
%         elseif (strfind(data, 'Heading'))
%             Hindex = Hindex + 1;
%             H = sscanf(data, 'Heading: %f');
%             HeadingData(:,Hindex) = H;
%         % Other
%         elseif (strfind(data, 'Duration'))
%             duration = sscanf(data, 'Duration: %d');
%         elseif (strfind(data, 'Display'))
%             disp(data);
%         end
    end

    disp('Finished Reading Data');
    
    % plot the data
    screenSize = get(0, 'ScreenSize');

% catch any exceptions so program doesn't quit
% ensures that the port gets closed
catch err   
    disp(err);
end

%close port and delete object
fclose(s1);
delete(s1);

clear s1;

