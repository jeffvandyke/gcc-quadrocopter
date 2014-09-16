% this script will read from serial port COM7
% written by Aaron Derstine
try
    % create serial object on COM7
    s1 = serial('COM5');
    s1.BaudRate = 9600; % BaudRate of Arduino

    % open the serial port
    fopen(s1);

    disp('Opening connection on Port COM7');
    disp('Will read data until "StopTransmission" is sent');
    data = 0;
    stopCondition = 'StopTransmission';
    
    % will keep reading until stopCondition is sent
    while (isempty(strfind(data,stopCondition)))
        data = fscanf(s1);
        % Sensor Data
        % pull out the data from the stream
        % roll, pitch, yaw
        D = sscanf(data, 'Data: %f %f %f');
        if ~isempty(D)
            rotate_copter(0,0,0,D(1),D(2),D(3));
            pause(.01);
        end
    end

    disp('Finished Reading Data');

% catch any exceptions so program doesn't quit
% ensures that the port gets closed
catch err   
    disp(err);
end

%close port and delete object
fclose(s1);
delete(s1);

clear s1;