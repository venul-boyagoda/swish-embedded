%delete(serialObj);
serialObj = serialport("/dev/tty.usbserial-2130",115200);
configureTerminator(serialObj,"CR/LF");
flush(serialObj);

serialObj.UserData = struct("Data", [], "Count", 1);
maxDataPoints = 1000;
configureCallback(serialObj, "terminator", @(src, event) storeSensorData(src, event, maxDataPoints));
