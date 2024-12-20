#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define BNO055_SAMPLERATE_DELAY_MS (10)

float ax, ay, az;
float gx, gy, gz;
float mx, my, mz; 

Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);

void setup(void)
{
  Serial.begin(115200);

  while (!Serial) delay(10);

  // Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");

  if(!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);

  bno.setExtCrystalUse(true);
}

void loop(void)
{
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

  // Serial.print("ACCEL X: ");
  // Serial.print(accel.x());
  // Serial.print(" Y: ");
  // Serial.print(accel.y());
  // Serial.print(" Z: ");
  // Serial.print(accel.z());
  // Serial.print("\t\t");

  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  // Serial.print("GYRO X: ");
  // Serial.print(gyro.x());
  // Serial.print(" Y: ");
  // Serial.print(gyro.y());
  // Serial.print(" Z: ");
  // Serial.print(gyro.z());
  // Serial.print("\t\t");

  imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

  // Serial.print("MAG X: ");
  // Serial.print(mag.x());
  // Serial.print(" Y: ");
  // Serial.print(mag.y());
  // Serial.print(" Z: ");
  // Serial.print(mag.z());
  // Serial.print("\t\t");

  // uint8_t system, gyrocal, accelcal, magcal = 0;
  // bno.getCalibration(&system, &gyrocal, &accelcal, &magcal);
  // Serial.print("CALIBRATION: Sys=");
  // Serial.print(system, DEC);
  // Serial.print(" Gyro=");
  // Serial.print(gyrocal, DEC);
  // Serial.print(" Accel=");
  // Serial.print(accelcal, DEC);
  // Serial.print(" Mag=");
  // Serial.println(magcal, DEC);
  ax = accel.x();
  ay = accel.y();
  az = accel.z();
  gx = gyro.x();
  gy = gyro.y();
  gz = gyro.z();
  mx = mag.x();
  my = mag.y();
  mz = mag.z();

  Serial.print(ax);
  Serial.print(",");
  Serial.print(ay);
  Serial.print(",");
  Serial.print(az);
  Serial.print(",");
  Serial.print(gx);
  Serial.print(",");
  Serial.print(gy);
  Serial.print(",");
  Serial.print(gz);
  Serial.print(",");
  Serial.print(mx);
  Serial.print(",");
  Serial.print(my);
  Serial.print(",");
  Serial.println(mz);

  delay(BNO055_SAMPLERATE_DELAY_MS);
}