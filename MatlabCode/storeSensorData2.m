function storeSensorData2(src, ~)
    % Read the ASCII data from the serialport object.
    data = readline(src);
    
    % Split the string by commas
    sensorCoords = split(data, ',');
    
    % Convert each part to numeric and store in an array
    numericData = str2double(sensorCoords);
    
    % Append the numeric data as a new row in the ImuData.Data matrix
    src.UserData.Data = [src.UserData.Data; numericData'];

    qPred = double(src.UserData.EkfModule.updateOrientation( ...
    py.numpy.array(src.UserData.Quaternion), ...
    py.numpy.array(numericData(1:3)), ...
    py.numpy.array(numericData(4:6)), ...
    py.numpy.array(numericData(7:9))));

    % Update the pose plot
    set(src.UserData.PosePlot, "Orientation", quaternion(qPred));
    drawnow limitrate;

    src.UserData.Quaternion = qPred;
   
end

