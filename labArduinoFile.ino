#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <MAXDAC.h>

#define BNO055_SAMPLERATE_DELAY_MS (0)

Adafruit_BNO055 bnoA = Adafruit_BNO055(-1, BNO055_ADDRESS_A);
Adafruit_BNO055 bnoB = Adafruit_BNO055(-1, BNO055_ADDRESS_B);
maxdac dac(0x2A);

int readingIndex = 0;
bool resetOnce = false;

void dacReset(void)
{
  dac.shutDown(false);
  for (int i=0; i<8; i++) {
    dac.write(i, 255); //writes all the registers to full power
  }
  delay(1000);
  for (int i=0; i<8; i++) {
    dac.write(i, 0); //writes all the registers to full power
  }
  delay(1000);
  for (int i=0; i<8; i++) {
    dac.write(i, 255); //writes all the registers to full power
  }
  delay(1000);
  for (int i=0; i<8; i++) {
    dac.write(i, 0); //writes all the registers to full power
  }
  delay(1000);
  dac.reset();
}
void displaySensorDetails(void)
{
  sensor_t sensorA;
  sensor_t sensorB;
  bnoA.getSensor(&sensorA);
  bnoB.getSensor(&sensorB);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor (A/B):       "); Serial.print(sensorA.name); Serial.print(", "); Serial.println(sensorB.name);
  Serial.print  ("Driver Ver (A/B):   "); Serial.print(sensorA.version); Serial.print(", "); Serial.println(sensorB.version);
  Serial.print  ("Unique ID (A/B):    "); Serial.print(sensorA.sensor_id); Serial.print(", "); Serial.println(sensorB.sensor_id);
  Serial.print  ("Max Value (A/B):    "); Serial.print(sensorA.max_value); Serial.print(" xxx"); Serial.print(", "); Serial.print(sensorB.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value (A/B):    "); Serial.print(sensorA.min_value); Serial.print(" xxx"); Serial.print(", "); Serial.print(sensorB.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution (A/B):   "); Serial.print(sensorA.resolution); Serial.print(" xxx"); Serial.print(", "); Serial.print(sensorB.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void setup() {
  Serial.begin(250000);
  if (analogRead(0) < 500) {
      /* Initialise the sensor */
      if (!bnoA.begin()) {
        Serial.print("Ooops, BNO055(A) not detected");
        while (1);
      }
    
      bnoA.setExtCrystalUse(true);
    
      if (!bnoB.begin()) {
        Serial.print("Ooops, BNO055(B) not detected");
        while (1);
      }
    
      bnoB.setExtCrystalUse(true);
    
      delay(1000);
    
      /* Display some basic information on this sensor */
      displaySensorDetails();

  }

  dacReset();
}

void loop(void) {
  /* Get a new sensor event */
  sensors_event_t eventA;
  sensors_event_t eventB;
  bnoA.getEvent(&eventA);
  bnoB.getEvent(&eventB);

  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2

  //imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Quaternion quatA = bnoA.getQuat();
  imu::Quaternion quatB = bnoB.getQuat();
  //imu::Vector<3> accl = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  //imu::Vector<3> magn = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  //imu::Vector<3> gyros = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  int checkResetPin = analogRead(1);
  
  if (checkResetPin > 500 && resetOnce == false ) {
    dacReset();
    resetOnce = true;
  } else if (checkResetPin < 500 && resetOnce == true) {
    resetOnce = false;
  }
  
  if (readingIndex < 256) {
    Serial.print(++readingIndex);
  } else {
    readingIndex = 0;
    Serial.print(++readingIndex);
  }
  Serial.print(",");

  int a = readingIndex % 2;     // calculate LSB
  int b = readingIndex / 2 % 2;
  int c = readingIndex / 4 % 2;
  int d = readingIndex / 8 % 2;
  int e = readingIndex / 16 % 2;
  int f = readingIndex / 32 % 2;
  int g = readingIndex / 64 % 2;
  int h = readingIndex / 128 % 2;
  
  dac.write(0, a * 127);
  dac.write(1, b * 127);
  dac.write(2, c * 127);
  dac.write(3, d * 127);
  dac.write(4, e * 127);
  dac.write(5, f * 127);
  dac.write(6, g * 127);
  dac.write(7, h * 127);

  /* Display the floating point data */
  /* Display calibration status for each sensor. */
  uint8_t systemA, gyroA, accelA, magA = 0;
  bnoA.getCalibration(&systemA, &gyroA, &accelA, &magA);
  Serial.print("A,");
  Serial.print(systemA, DEC);
  Serial.print(",");
  Serial.print(gyroA, DEC);
  Serial.print(",");
  Serial.print(accelA, DEC);
  Serial.print(",");
  Serial.print(magA, DEC);

  uint8_t systemB, gyroB, accelB, magB = 0;
  bnoB.getCalibration(&systemB, &gyroB, &accelB, &magB);
  Serial.print(",B,");
  Serial.print(systemB, DEC);
  Serial.print(",");
  Serial.print(gyroB, DEC);
  Serial.print(",");
  Serial.print(accelB, DEC);
  Serial.print(",");
  Serial.print(magB, DEC);

  Serial.print(",A,");
  Serial.print(quatA.w(), 14);
  Serial.print(",");
  Serial.print(quatA.x(), 14);
  Serial.print(",");
  Serial.print(quatA.y(), 14);
  Serial.print(",");
  Serial.print(quatA.z(), 14);

  Serial.print(",B,");
  Serial.print(quatB.w(), 14);
  Serial.print(",");
  Serial.print(quatB.x(), 14);
  Serial.print(",");
  Serial.print(quatB.y(), 14);
  Serial.print(",");
  Serial.println(quatB.z(), 14);
  /*
    Serial.print(",");
    Serial.print(accl.x(), 23);
    Serial.print(",");
    Serial.print(accl.y(), 23);
    Serial.print(",");
    Serial.print(accl.z(), 23);

    Serial.print(",");
    Serial.print(magn.x(), 4);
    Serial.print(",");
    Serial.print(magn.y(), 4);
    Serial.print(",");
    Serial.print(magn.z(), 4);

    Serial.print(",");
    Serial.print(gyros.x(), 26);
    Serial.print(",");
    Serial.print(gyros.y(), 26);
    Serial.print(",");
    Serial.print(gyros.z(), 26);
    Serial.print(",");
    sensorValue = analogRead(A0)*(5.0/1023.0)*(1.0/32.81155203);
    Serial.print(analogRead(A0),6);
    Serial.print(",");
    Serial.println(sensorValue,6);

    delay(0.001); */
}