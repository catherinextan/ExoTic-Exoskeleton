#include <SPI.h>
#include <Wire.h>
#include <SFE_LSM9DS0.h>

#define LSM9DS0_XM 0x1D
#define LSM9DS0_G 0x6B
LSM9DS0 dof(MODE_I2C, LSM9DS0_G, LSM9DS0_XM);

#define PRINT_SPEED 500 // 500 ms between prints

// Velocity variables
float velocityX = 0, velocityY = 0, velocityZ = 0;
float biasX = 0, biasY = 0, biasZ = 0;
bool calibrated = false;
const int CALIBRATION_SAMPLES = 50;
int calibrationCount = 0;

// Moving average filter
const int FILTER_SIZE = 5;
float accXHistory[FILTER_SIZE] = {0};
float accYHistory[FILTER_SIZE] = {0};
float accZHistory[FILTER_SIZE] = {0};
int filterIndex = 0;

// Dynamic zero velocity detection
const float ACCEL_THRESHOLD = 0.03;  // Threshold for detecting motion
const float VELOCITY_DECAY = 0.85;   // Velocity decay factor when near-stationary
const int STATIONARY_COUNTER_THRESHOLD = 10; // Number of readings below threshold to consider stationary
int stationaryCounter = 0;

unsigned long lastTime = 0;

void setup() {
  Serial.begin(115200);
  
  uint16_t status = dof.begin();
  Serial.print("LSM9DS0 WHO_AM_I's returned: 0x");
  Serial.println(status, HEX);
  Serial.println("Should be 0x49D4\n");
  
  // Configure sensor settings
  dof.setAccelScale(dof.A_SCALE_2G);
  dof.setGyroScale(dof.G_SCALE_245DPS);
  dof.setAccelODR(dof.A_ODR_100);
  dof.setGyroODR(dof.G_ODR_95_BW_25);
  
  // Initialize time
  lastTime = millis();
  
  Serial.println("Calibrating accelerometer, please keep device still...");
  delay(1000); // Give user time to place device
}

float movingAverage(float newValue, float* history, int size) {
  // Update history
  history[filterIndex] = newValue;
  filterIndex = (filterIndex + 1) % size;
  
  // Calculate average
  float sum = 0;
  for (int i = 0; i < size; i++) {
    sum += history[i];
  }
  return sum / size;
}

void calibrateAccelerometer() {
  // We'll calibrate by averaging a number of readings to determine bias
  dof.readAccel();
  float ax = dof.calcAccel(dof.ax);
  float ay = dof.calcAccel(dof.ay);
  float az = dof.calcAccel(dof.az);
  
  // Accumulate readings
  biasX += ax;
  biasY += ay;
  biasZ += (az - 1.0); // Subtract 1g from Z-axis (assuming Z is up)
  
  calibrationCount++;
  
  if (calibrationCount >= CALIBRATION_SAMPLES) {
    // Calculate average bias
    biasX /= CALIBRATION_SAMPLES;
    biasY /= CALIBRATION_SAMPLES;
    biasZ /= CALIBRATION_SAMPLES;
    
    Serial.println("Calibration complete!");
    Serial.print("Bias X: "); Serial.println(biasX, 4);
    Serial.print("Bias Y: "); Serial.println(biasY, 4);
    Serial.print("Bias Z: "); Serial.println(biasZ, 4);
    
    calibrated = true;
  }
}

void loop() {
  if (!calibrated) {
    calibrateAccelerometer();
    return;
  }
  
  printGyro();
  printAccel();
  printMag();
  printHeading((float)dof.mx, (float)dof.my);
  printOrientation(dof.calcAccel(dof.ax), dof.calcAccel(dof.ay), dof.calcAccel(dof.az));
  Serial.println();
  delay(PRINT_SPEED);
}

void printGyro() {
  dof.readGyro();
  Serial.print("G: ");
  Serial.print(dof.calcGyro(dof.gx), 2);
  Serial.print(", ");
  Serial.print(dof.calcGyro(dof.gy), 2);
  Serial.print(", ");
  Serial.println(dof.calcGyro(dof.gz), 2);
}

void printAccel() {
  dof.readAccel();
  
  // Read raw acceleration values
  float ax = dof.calcAccel(dof.ax);
  float ay = dof.calcAccel(dof.ay);
  float az = dof.calcAccel(dof.az);
  
  // Correct for bias (from calibration)
  float correctedAx = ax - biasX;
  float correctedAy = ay - biasY;
  float correctedAz = az - biasZ - 1.0; // Remove gravity component (assuming Z is up)
  
  // Apply moving average filter
  float filteredAx = movingAverage(correctedAx, accXHistory, FILTER_SIZE);
  float filteredAy = movingAverage(correctedAy, accYHistory, FILTER_SIZE);
  float filteredAz = movingAverage(correctedAz, accZHistory, FILTER_SIZE);
  
  // Apply dead zone to filtered values
  if (abs(filteredAx) < ACCEL_THRESHOLD) filteredAx = 0;
  if (abs(filteredAy) < ACCEL_THRESHOLD) filteredAy = 0;
  if (abs(filteredAz) < ACCEL_THRESHOLD) filteredAz = 0;
  
  // Calculate time delta
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;
  
  // Check for stationary state
  if (abs(filteredAx) < ACCEL_THRESHOLD && 
      abs(filteredAy) < ACCEL_THRESHOLD && 
      abs(filteredAz) < ACCEL_THRESHOLD) {
    stationaryCounter++;
    if (stationaryCounter > STATIONARY_COUNTER_THRESHOLD) {
      // Device is likely stationary, gradually decrease velocity
      velocityX *= VELOCITY_DECAY;
      velocityY *= VELOCITY_DECAY;
      velocityZ *= VELOCITY_DECAY;
      
      // Reset to zero if very small
      if (abs(velocityX) < 0.1) velocityX = 0;
      if (abs(velocityY) < 0.1) velocityY = 0;
      if (abs(velocityZ) < 0.1) velocityZ = 0;
    }
  } else {
    stationaryCounter = 0; // Reset counter if there's motion
    
    // Integrate acceleration to get velocity
    velocityX += filteredAx * dt * 9.81;
    velocityY += filteredAy * dt * 9.81;
    velocityZ += filteredAz * dt * 9.81;
  }
  
  // Don't use hard constraints - they cause the "stuck at -20" problem
  // Instead use adaptive drift compensation based on confidence
  float confidence = max(0.0, 1.0 - (stationaryCounter / (float)STATIONARY_COUNTER_THRESHOLD));
  velocityX *= (0.99 + 0.01 * confidence);
  velocityY *= (0.99 + 0.01 * confidence);
  velocityZ *= (0.99 + 0.01 * confidence);
  
  Serial.print("Acceleration (g): X = ");
  Serial.print(ax, 2);
  Serial.print(", Y = ");
  Serial.print(ay, 2);
  Serial.print(", Z = ");
  Serial.println(az, 2);
  
  Serial.print("Velocity (m/s): X = ");
  Serial.print(velocityX, 2);
  Serial.print(", Y = ");
  Serial.print(velocityY, 2);
  Serial.print(", Z = ");
  Serial.println(velocityZ, 2);
}

void printMag() {
  dof.readMag();
  Serial.print("M: ");
  Serial.print(dof.calcMag(dof.mx), 2);
  Serial.print(", ");
  Serial.print(dof.calcMag(dof.my), 2);
  Serial.print(", ");
  Serial.println(dof.calcMag(dof.mz), 2);
}

void printHeading(float hx, float hy) {
  float heading = (hy > 0) ? (90 - atan(hx / hy) * (180 / PI)) : (-atan(hx / hy) * (180 / PI));
  if (hy == 0) heading = (hx < 0) ? 180 : 0;
  Serial.print("Heading: ");
  Serial.println(heading, 2);
}

void printOrientation(float x, float y, float z) {
  float pitch = atan2(x, sqrt(y * y + z * z)) * 180.0 / PI;
  float roll = atan2(y, sqrt(x * x + z * z)) * 180.0 / PI;
  Serial.print("Pitch, Roll: ");
  Serial.print(pitch, 2);
  Serial.print(", ");
  Serial.println(roll, 2);
}