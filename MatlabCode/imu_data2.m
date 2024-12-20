delete(serialObj);
serialObj = serialport("/dev/tty.usbserial-2130",115200);
configureTerminator(serialObj,"CR/LF");
flush(serialObj);

if count(py.sys.path, pwd) == 0
    insert(py.sys.path, int32(0), pwd);
end

ekf_module = py.importlib.import_module('extendedkalmanfilter');
py.importlib.reload(ekf_module);

posePlot = poseplot();
title('Real-Time IMU Orientation');

q = [1.0, 0.0, 0.0, 0.0];

serialObj.UserData = struct("Data", [], "Count", 1, "PosePlot", posePlot, "Quaternion", q, "EkfModule", ekf_module);

configureCallback(serialObj, "terminator", @(src, event) storeSensorData2(src, event));