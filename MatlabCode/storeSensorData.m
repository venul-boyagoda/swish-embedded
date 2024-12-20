function storeSensorData(src, ~, maxDataPoints)
    % Read the ASCII data from the serialport object.
    data = readline(src);
    
    % Split the string by commas
    sensorCoords = split(data, ',');
    
    % Convert each part to numeric and store in an array
    numericData = str2double(sensorCoords);
    
    % Append the numeric data as a new row in the ImuData.Data matrix
    src.UserData.Data = [src.UserData.Data; numericData'];
    
    % Update the Count value
    src.UserData.Count = src.UserData.Count + 1;
    
    % Stop collecting data if the count exceeds maxDataPoints
    if src.UserData.Count > maxDataPoints
        configureCallback(src, "off");

        % Extract data for each sensor type
        accelData = src.UserData.Data(:, 1:3); % Ax, Ay, Az
        gyroData = src.UserData.Data(:, 4:6);  % Gx, Gy, Gz
        magData = src.UserData.Data(:, 7:9);   % Mx, My, Mz

        % Define filter parameters
        samplingRate = 50; % Set your sampling rate in Hz (e.g., 100 samples per second)
        cutoffFrequency = 2; % Set the cutoff frequency in Hz (adjust as necessary)
        
        % Retrieve the sensor data from UserData.Data matrix
        data = src.UserData.Data; % Rows are samples, columns are Ax, Ay, Az, etc.

        % Calculate time vector
        timeVector = (0:length(data)-1) / samplingRate; % in seconds
        
        % Apply low-pass filter to each column (Ax, Ay, Az, etc.)
        filteredAx = lowpass(data(:, 1), cutoffFrequency, samplingRate);
        filteredAy = lowpass(data(:, 2), cutoffFrequency, samplingRate);
        filteredAz = lowpass(data(:, 3), cutoffFrequency, samplingRate);
        
        filteredGx = lowpass(data(:, 4), cutoffFrequency, samplingRate);
        filteredGy = lowpass(data(:, 5), cutoffFrequency, samplingRate);
        filteredGz = lowpass(data(:, 6), cutoffFrequency, samplingRate);
        
        filteredMx = lowpass(data(:, 7), cutoffFrequency, samplingRate);
        filteredMy = lowpass(data(:, 8), cutoffFrequency, samplingRate);
        filteredMz = lowpass(data(:, 9), cutoffFrequency, samplingRate);
        
        % Store the filtered data for further processing or plotting
        filteredData = [filteredAx, filteredAy, filteredAz, ...
                        filteredGx, filteredGy, filteredGz, ...
                        filteredMx, filteredMy, filteredMz];
        Y = fft(gyroData(:, 1));
        Fs = samplingRate;            % Sampling frequency                    
        T = 1/Fs;             % Sampling period       
        L = 1000;             % Length of signal
        t = (0:L-1)*T;        % Time vector


        plot(Fs/L*(-L/2:L/2-1),abs(fftshift(Y)),"LineWidth",3)
        title("fft Spectrum in the Positive and Negative Frequencies")
        xlabel("f (Hz)")
        ylabel("|fft(X)|")
        

        % Plot filtered data for one of the sensor groups as an example
        figure;
        
        subplot(3,2,1);
        plot(timeVector, filteredAx, 'r'); hold on;
        plot(timeVector, filteredAy, 'g');
        plot(timeVector, filteredAz, 'b');
        legend('Ax', 'Ay', 'Az');
        xlabel('Time (s)');
        ylabel('Acceleration (m/s^2)');
        title('Filtered Accelerometer Data');
        
        subplot(3,2,3);
        plot(timeVector, filteredGx, 'r'); hold on;
        plot(timeVector, filteredGy, 'g');
        plot(timeVector, filteredGz, 'b');
        legend('Gx', 'Gy', 'Gz');
        xlabel('Time (s)');
        ylabel('Angular Velocity (rad/s)');
        title('Filtered Gyroscope Data');
        
        subplot(3,2,5);
        plot(timeVector, filteredMx, 'r'); hold on;
        plot(timeVector, filteredMy, 'g');
        plot(timeVector, filteredMz, 'b');
        legend('Mx', 'My', 'Mz');
        xlabel('Time (s)');
        ylabel('Magnetic Field (uT)');
        title('Filtered Magnetometer Data');

        hold off;


        % % Create a new figure with 3 subplots
        % figure;
        
        % Plot accelerometer data
        subplot(3, 2, 2);
        plot(timeVector, accelData(:, 1), 'r'); hold on;
        plot(timeVector, accelData(:, 2), 'g');
        plot(timeVector, accelData(:, 3), 'b');
        legend('Ax', 'Ay', 'Az');
        xlabel('Time (s)');
        ylabel('Acceleration (m/s^2)');
        title('Accelerometer Data');

        % Plot gyroscope data
        subplot(3, 2, 4);
        plot(timeVector, gyroData(:, 1), 'r'); hold on;
        plot(timeVector, gyroData(:, 2), 'g');
        plot(timeVector, gyroData(:, 3), 'b');
        legend('Gx', 'Gy', 'Gz');
        xlabel('Time (s)');
        ylabel('Angular Velocity (rad/s)');
        title('Gyroscope Data');

        % Plot magnetometer data
        subplot(3, 2, 6);
        plot(timeVector, magData(:, 1), 'r'); hold on;
        plot(timeVector, magData(:, 2), 'g');
        plot(timeVector, magData(:, 3), 'b');
        legend('Mx', 'My', 'Mz');
        xlabel('Time (s)');
        ylabel('Magnetic Field (uT)');
        title('Magnetometer Data');

        hold off;
    end
end

