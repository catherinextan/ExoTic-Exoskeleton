#include <SD.h>
#include <SPI.h>
#include <LSM6DS3.h>
#include <Wire.h>

const int flexSensorPin = PD4;  // Analog pin for the flex sensor
const int redPin = 1;          // PWM pin for red LED (D1)
const int greenPin = 2;        // PWM pin for green LED (D2)
const int bluePin = 3;         // PWM pin for blue LED (D3)
const int chipSelect = 6;      // CS pin for microSD (D6)
LSM6DS3 myIMU(I2C_MODE, 0x6A);

// Calibration variables
float initial_aX = 0, initial_aY = 0, initial_aZ = 0;

// Threshold parameters
const float DIRECTION_THRESHOLD = 25.0;    // Degrees/s
const float ACCEL_THRESH = 0.5;           // m/s²
const float DRIFT_COMP = 0.15;             // 25% velocity decay per second
const float ZERO_THRESH = 0.1;             // Speed threshold
const float STATIONARY_ACCEL = 0.2;        // m/s² for stationary detection

// Motion tracking
float velX = 0, velY = 0, velZ = 0;
unsigned long lastUpdate = 0;
float maxDirectionChange = 0;
bool isStationary = true;

// Timing control
unsigned long lastPrintTime = 0;
const unsigned long PRINT_INTERVAL = 2000;


void setup() {
  Serial.begin(115200);
  while(!Serial);

  pinMode(PD5, OUTPUT);
  digitalWrite(PD5, HIGH);

  // SD card initialization
if (!SD.begin(chipSelect)) {
      Serial.println("SD card init failed");
    } else {
      Serial.println("SD card initialized");
      // csv file header row
      File dataFile = SD.open("DATA.csv", FILE_WRITE);
      if (dataFile) {
        dataFile.println("Ax, Ay, Az, Gx, Gy, Gz"); // acceleratometer and gyro measurements
        dataFile.close();
      } else {
        Serial.println("Error, could not create DATA.csv");
      } // else
    } // else

  // IMU initialization 
  if(myIMU.begin() != 0) {
    Serial.println("IMU initialization failed");
  } else {
    calibrateIMU();
    Serial.println("Time(s)\tSpeed\tVelX\tVelY\tVelZ\tAccelerated\tAccX\tAccY\tAccZ\tDirection");
    lastUpdate = micros();
    lastPrintTime = millis();
  } // if
} // setup

void loop() {
  unsigned long nowMicros = micros();
  float dt = (nowMicros - lastUpdate)/1e6f;
  lastUpdate = nowMicros;

  // Read and calibrate sensor data
  float accX = (myIMU.readFloatAccelX() - initial_aX) * 9.81;
  float accY = (myIMU.readFloatAccelY() - initial_aY) * 9.81;
  float accZ = (myIMU.readFloatAccelZ() - initial_aZ) * 9.81;
  float gZ = myIMU.readFloatGyroZ();

  // Stationary detection
  float accMag = sqrt(accX*accX + accY*accY + accZ*accZ);
  if(accMag < STATIONARY_ACCEL) {
    if(!isStationary) {
      // Reset velocities if newly stationary
      velX = velY = velZ = 0;
      isStationary = true;
    } // if
  } else {
    isStationary = false;
    // Update velocity with strong drift compensation
    velX += accX * dt - velX * DRIFT_COMP * dt;
    velY += accY * dt - velY * DRIFT_COMP * dt;
    velZ += accZ * dt - velZ * DRIFT_COMP * dt;
  } // if

  // Track direction changes
  if(fabs(gZ) > maxDirectionChange) {
    maxDirectionChange = fabs(gZ);
  } // if

  // Periodic reporting
  if(millis() - lastPrintTime >= PRINT_INTERVAL) {
    float speed = sqrt(velX*velX + velY*velY + velZ*velZ);
    if(speed < ZERO_THRESH) speed = 0;
    
    String accelerated = (accMag > ACCEL_THRESH) ? "YES" : "NO";
    String direction = (maxDirectionChange > DIRECTION_THRESHOLD) ? 
                      String(maxDirectionChange, 1)+"°/s" : "-";

    Serial.print(millis()/1000.0, 1); Serial.print("\t");
    Serial.print(speed, 2);          Serial.print("\t");
    Serial.print(velX, 2);           Serial.print("\t");
    Serial.print(velY, 2);           Serial.print("\t");
    Serial.print(velZ, 2);           Serial.print("\t");
    Serial.print(accelerated);       Serial.print("\t\t");
    Serial.print(accX, 2);           Serial.print("\t");
    Serial.print(accY, 2);           Serial.print("\t");
    Serial.print(accZ, 2);           Serial.print("\t");
    Serial.println(direction);

    // Reset tracking variables
    maxDirectionChange = 0;
    lastPrintTime = millis();
  } // if
} // loop


// ============ SD card data logging function =============
void logData(float ax, float ay, float az,
             float gx, float gy, float gz) {
  File dataFile = SD.open("DATA.csv", FILE_WRITE);
  if (dataFile) {
    dataFile.print(ax);
    dataFile.print(",");
    dataFile.print(ay);
    dataFile.print(",");
    dataFile.print(az);
    dataFile.print(",");
    dataFile.print(gx);
    dataFile.print(",");
    dataFile.print(gy);
    dataFile.print(",");
    dataFile.println(gz);   // added newline to fix formatting
    dataFile.close();
    Serial.println("Flex Sensor data logged to SD");
  } else {
    Serial.println("Error opening DATA.csv for writing");
  } // if
} // logData

// =========== IMU calibration function =============
void calibrateIMU() {
  const int NUM_CAL = 500;  // Increased calibration samples
  float sum_aX = 0, sum_aY = 0, sum_aZ = 0;
  
  Serial.println("Calibrating... Keep sensor stationary!");
  delay(2000);  // Allow time to position sensor
  
  for(int i=0; i<NUM_CAL; i++) {
    sum_aX += myIMU.readFloatAccelX();
    sum_aY += myIMU.readFloatAccelY();
    sum_aZ += myIMU.readFloatAccelZ();
    delay(10);
  }
  initial_aX = sum_aX/NUM_CAL;
  initial_aY = sum_aY/NUM_CAL;
  initial_aZ = sum_aZ/NUM_CAL;
  
  Serial.print("Calibration offsets: ");
  Serial.print(initial_aX, 4); Serial.print(", ");
  Serial.print(initial_aY, 4); Serial.print(", ");
  Serial.println(initial_aZ, 4);
} // calibrateIMU