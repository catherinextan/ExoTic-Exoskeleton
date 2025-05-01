// Define pin connections
const int flexSensorPin = A0;  // Analog pin for the flex sensor
const int redPin = 1;          // PWM pin for red LED (D1)
const int greenPin = 2;        // PWM pin for green LED (D2)
const int bluePin = 3;         // PWM pin for blue LED (D3)

// Dynamic calibration
int neutralFlex = 0;  // Neutral (unbent) sensor reading
const int flexBuffer = 50;  // Dead zone buffer to prevent false detections

void setup() {
    pinMode(redPin, OUTPUT);
    pinMode(greenPin, OUTPUT);
    pinMode(bluePin, OUTPUT);
    Serial.begin(115200);

    // Read neutral position at startup
    long sum = 0;
    for (int i = 0; i < 100; i++) {  // Take multiple readings to average
        sum += analogRead(flexSensorPin);
        delay(10);
    }
    neutralFlex = sum / 100;  // Set as baseline
    Serial.print("Neutral Position: ");
    Serial.println(neutralFlex);
}

void loop() {
    int flexValue = analogRead(flexSensorPin);  // Read flex sensor value
    int deviation = flexValue - neutralFlex;  // Difference from neutral

    Serial.print("Flex Value: ");
    Serial.print(flexValue);
    Serial.print(" | Deviation: ");
    Serial.println(deviation);

    int stage;

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

    delay(100);  // Small delay for stability
}

// Function to set RGB LED color
void setColor(int red, int green, int blue) {
    analogWrite(redPin, 255 - red);   // Common cathode correction
    analogWrite(greenPin, 255 - green);
    analogWrite(bluePin, 255 - blue);
} 
