#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>  // required!
#include <Adafruit_LSM9DS1.h>

#define PRINT_SPEED 500 // 500 ms between prints

// Define pin connections
const int flexSensorPin = A0;  // Analog pin for the flex sensor
const int redPin = 1;          // PWM pin for red LED (D1)
const int greenPin = 2;        // PWM pin for green LED (D2)
const int bluePin = 3;         // PWM pin for blue LED (D3)
const int chipSelect = 6;      // CS pin for microSD (D6)

Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(); // i2c constructor (we used SDA/SCL to connect IMU)

// Dynamic calibration
int neutralFlex = 0;  // Neutral (unbent) sensor reading
const int flexBuffer = 50;  // Dead zone buffer to prevent false detections

const unsigned long loggingInterval = 100; // in ms

void setup() {
    pinMode(redPin, OUTPUT);
    pinMode(greenPin, OUTPUT);
    pinMode(bluePin, OUTPUT);
    Serial.begin(115200);

    while (!Serial) {
      delay(1);
    } // while

    Serial.println("LSM9DS1 data read demo");

    // IMU code
     // initialization (from demo code)
    if (!lsm.begin()) {
      Serial.println("Failed to init LSM9DS1. Check your wiring!");
      while (1);
    } // if
    Serial.println("LSM9DS1 IMU initialized");
    setupSensor();

    // SD card code
    // initialization
    if (!SD.begin(chipSelect)) {
      Serial.println("SD card init failed");
    } else {
      Serial.println("SD card initialized");
      // csv file header row
      File dataFile = SD.open("DATA.csv", FILE_WRITE);
      if (dataFile) {
        dataFile.println("Time(ms), Flex Value, Deviation, Stage"); // stage dictating color
        dataFile.close();
      } else {
        Serial.println("Error, could not create DATA.csv");
      } // else
    } // else

    // IMU code
    // initialization
    if (!lsm.begin()) {
      Serial.println("IMU init failed");
      while(1);
    } // if
    Serial.println("IMU initialized");

    // Flex sensor code
    // Read neutral position at startup
    long sum = 0;
    for (int i = 0; i < 100; i++) {  // Take multiple readings to average
        sum += analogRead(flexSensorPin);
        delay(10);
    }
    neutralFlex = sum / 100;  // Set as baseline
    Serial.print("Neutral Position: ");
    Serial.println(neutralFlex);
} // setup

void loop() {
    int flexValue = analogRead(flexSensorPin);  // Read flex sensor value
    int deviation = flexValue - neutralFlex;  // Difference from neutral

    // print flex sensor values to serial monitor
    Serial.print("Flex Value: ");
    Serial.print(flexValue);
    Serial.print(" | Deviation: ");
    Serial.println(deviation);

    int stage; // dictates RGB LED color

    if (abs(deviation) < flexBuffer) {
        stage = 3;  // Close to neutral
    } else if (deviation > 0) {  // Bending in one direction
        stage = map(deviation, flexBuffer, 300, 4, 6);  // Scale up
    } else {  // Bending in the opposite direction
        stage = map(deviation, -300, -flexBuffer, 0, 2);  // Scale down
    }
    stage = constrain(stage, 0, 6);

    // Define 7 color states for bidirectional bending
    switch (stage) {
        case 0: setColor(255, 0, 0); break;   // Red - Max bend (left)
        case 1: setColor(255, 128, 0); break; // Orange
        case 2: setColor(255, 255, 0); break; // Yellow
        case 3: setColor(0, 255, 0); break;   // Green - Neutral position
        case 4: setColor(0, 255, 255); break; // Cyan
        case 5: setColor(0, 0, 255); break;   // Blue
        case 6: setColor(128, 0, 255); break; // Purple - Max bend (right)
    }

    // read IMU data 
    lsm.read();
    sensors_event_t a, m, g, temp;
    lsm.getEvent(&a, &m, &g, &temp);

    // print IMU values to serial monitor
    Serial.print("Accel X: "); Serial.print(a.acceleration.x); Serial.print(" m/s^2");
    Serial.print("\tY: "); Serial.print(a.acceleration.y);     Serial.print(" m/s^2 ");
    Serial.print("\tZ: "); Serial.print(a.acceleration.z);     Serial.println(" m/s^2 ");

    Serial.print("Mag X: "); Serial.print(m.magnetic.x);   Serial.print(" uT");
    Serial.print("\tY: "); Serial.print(m.magnetic.y);     Serial.print(" uT");
    Serial.print("\tZ: "); Serial.print(m.magnetic.z);     Serial.println(" uT");

    Serial.print("Gyro X: "); Serial.print(g.gyro.x);   Serial.print(" rad/s");
    Serial.print("\tY: "); Serial.print(g.gyro.y);      Serial.print(" rad/s");
    Serial.print("\tZ: "); Serial.print(g.gyro.z);      Serial.println(" rad/s");
    
    // initialize variables to be logged in sd card
    float accX = a.acceleration.x;
    float accY = a.acceleration.y;
    float accZ = a.acceleration.z;
    float magX = m.magnetic.x;
    float magY = m.magnetic.y;
    float magZ = m.magnetic.z;
    float gyroX = g.gyro.x;
    float gyroY = g.gyro.y;
    float gyroZ = g.gyro.z;

    // log sensor data into SD card
    logData(flexValue, deviation, stage,  // flex sensor values
            accX, accY, accZ,             // IMU acceleration values
            magX, magY, magZ,             // IMU magnetic values
            gyroX, gyroY, gyroZ);         // IMU gyro values

    // changed from 100
    delay(loggingInterval);  // Small delay for stability
} // loop

// ============== HELPER FUNCTIONS ================
// Function to set RGB LED color
void setColor(int red, int green, int blue) {
    analogWrite(redPin, 255 - red);   // Common cathode correction
    analogWrite(greenPin, 255 - green);
    analogWrite(bluePin, 255 - blue);
} // setColor

void logData(int flexValue, int deviation, int stage, 
             float ax, float ay, float az,
             float mx, float my, float mz,
             float gx, float gy, float gz) {
  File dataFile = SD.open("DATA.csv", FILE_WRITE);
  if (dataFile) {
    dataFile.print(millis());
    dataFile.print(",");
    dataFile.print(flexValue);
    dataFile.print(",");
    dataFile.print(deviation);
    dataFile.print(",");
    dataFile.print(stage);
    dataFile.print(",");
    dataFile.print(ax);
    dataFile.print(",");
    dataFile.print(ay); 
    dataFile.print(",");
    dataFile.print(az);
    dataFile.print(",");
    dataFile.print(mx);
    dataFile.print(",");
    dataFile.print(my);
    dataFile.print(",");
    dataFile.print(mz);
    dataFile.print(",");
    dataFile.print(gx);
    dataFile.print(",");
    dataFile.print(gy);
    dataFile.print(",");
    dataFile.print(gz);
    dataFile.close();
    Serial.println("Flex Sensor data logged to SD");
  } else {
    Serial.println("Error in opening DATA.csv for writing");
  } // if
} // logData

void setupSensor() {
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G, lsm.LSM9DS1_ACCELDATARATE_10HZ);
  
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
} // setupSensor