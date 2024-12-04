#include <Stepper.h>  // Include the Stepper motor library

// Motor Configuration
const int stepsPerRevolution = 2048;  // 28BYJ-48 motor (used for 90-degree rotation)
Stepper myStepper(stepsPerRevolution, 3, 4, 5, 6);  // Motor connected to pins 3, 4, 5, 6

// Ultrasonic Sensor Pins
#define TRIG_PIN 9
#define ECHO_PIN 10

// Threshold for obstacle detection
#define OBSTACLE_THRESHOLD 20  // Distance threshold in cm

// Interval for taking sensor readings (every 5 degrees)
#define DEGREE_STEP 5
#define STEPS_PER_DEGREE (stepsPerRevolution / 360)

// Signal Pin for continue and ignore command (pulse signal)
#define SIGNAL_PIN 7  // Pin to receive the signal to continue and ignore obstacle

// Variables to track obstacle status
bool obstacleDetected = false;
bool obstacleIgnored = false;  // Tracks whether the current obstacle is being ignored
bool skipObstacle = false;    // Tracks whether we should skip the obstacle

void setup() {
  Serial.begin(9600);
  myStepper.setSpeed(15);  // Set motor speed (RPM)

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(SIGNAL_PIN, INPUT);  // Set the signal pin as input
}

void loop() {
  // Rotate motor 90 degrees clockwise, checking for obstacles every 5 degrees
  rotateMotorClockwise();

  // Rotate motor 90 degrees counterclockwise
  rotateMotorCounterClockwise();
}

// Function to rotate the motor clockwise by 90 degrees
void rotateMotorClockwise() {
  Serial.println("Rotating 90 degrees clockwise...");

  for (int i = 0; i < 18; i++) {  // 90 degrees = 18 steps (each step = 5 degrees)
    moveMotor(1);  // Move 5 degrees clockwise
    long distance = measureDistance();  // Measure distance

    Serial.print("Distance at ");
    Serial.print((i + 1) * DEGREE_STEP);
    Serial.print(" degrees: ");
    Serial.println(distance);

    // Check if obstacle is detected
    if (distance <= OBSTACLE_THRESHOLD) {
      if (!obstacleDetected) {
        obstacleDetected = true;  // Set obstacle detected status
        obstacleIgnored = false;  // Reset ignore flag
        Serial.println("Obstacle detected! Checking if obstacle is removed...");
      }

      // Continuously check if the obstacle is removed
      while (obstacleDetected) {
        long distance = measureDistance();  // Keep measuring distance

        if (distance > OBSTACLE_THRESHOLD) {
          // If the obstacle is removed
          Serial.println("Obstacle removed. Continuing movement...");
          obstacleDetected = false;  // Reset obstacle detected status
          break;  // Exit while loop, continue moving
        }

        // Check if a pulse signal is received to ignore the obstacle
        if (digitalRead(SIGNAL_PIN) == HIGH && !obstacleIgnored) {
          Serial.println("Pulse received: Obstacle ignored. Continuing movement.");
          obstacleDetected = false;  // Ignore the obstacle and continue moving
          obstacleIgnored = true;  // Mark that the obstacle is ignored
          skipObstacle = true;  // Flag that we are skipping the current obstacle
          break;  // Exit while loop
        }

        delay(100);  // Wait a little before checking again
      }
    } else {
      // If no obstacle, continue moving as usual
      delay(10);  // Small delay to allow for smooth motor movement
    }

    // If skipping obstacle, continue ignoring the obstacle
    if (skipObstacle) {
      Serial.println("Skipping the obstacle...");
      while (distance <= OBSTACLE_THRESHOLD) {  // Keep skipping as long as obstacle is present
        distance = measureDistance();  // Keep measuring distance
        delay(100);  // Continue checking
      }
      // Once the obstacle is skipped (removed), stop skipping and continue
      skipObstacle = false;
    }
  }

  Serial.println("Reached 90 degrees clockwise.");
}

// Function to rotate the motor counterclockwise by 90 degrees
void rotateMotorCounterClockwise() {
  Serial.println("Rotating 90 degrees counterclockwise...");

  for (int i = 0; i < 18; i++) {  // 90 degrees = 18 steps (each step = 5 degrees)
    moveMotor(-1);  // Move 5 degrees counterclockwise
    long distance = measureDistance();  // Measure distance

    Serial.print("Distance at ");
    Serial.print((i + 1) * DEGREE_STEP);
    Serial.print(" degrees: ");
    Serial.println(distance);

    // Check if obstacle is detected
    if (distance <= OBSTACLE_THRESHOLD) {
      if (!obstacleDetected) {
        obstacleDetected = true;  // Set obstacle detected status
        obstacleIgnored = false;  // Reset ignore flag
        Serial.println("Obstacle detected! Checking if obstacle is removed...");
      }

      // Continuously check if the obstacle is removed
      while (obstacleDetected) {
        long distance = measureDistance();  // Keep measuring distance

        if (distance > OBSTACLE_THRESHOLD) {
          // If the obstacle is removed
          Serial.println("Obstacle removed. Continuing movement...");
          obstacleDetected = false;  // Reset obstacle detected status
          break;  // Exit while loop, continue moving
        }

        // Check if a pulse signal is received to ignore the obstacle
        if (digitalRead(SIGNAL_PIN) == HIGH && !obstacleIgnored) {
          Serial.println("Pulse received: Obstacle ignored. Continuing movement.");
          obstacleDetected = false;  // Ignore the obstacle and continue moving
          obstacleIgnored = true;  // Mark that the obstacle is ignored
          skipObstacle = true;  // Flag that we are skipping the current obstacle
          break;  // Exit while loop
        }

        delay(100);  // Wait a little before checking again
      }
    } else {
      // If no obstacle, continue moving as usual
      delay(10);  // Small delay to allow for smooth motor movement
    }

    // If skipping obstacle, continue ignoring the obstacle
    if (skipObstacle) {
      Serial.println("Skipping the obstacle...");
      while (distance <= OBSTACLE_THRESHOLD) {  // Keep skipping as long as obstacle is present
        distance = measureDistance();  // Keep measuring distance
        delay(100);  // Continue checking
      }
      // Once the obstacle is skipped (removed), stop skipping and continue
      skipObstacle = false;
    }
  }

  Serial.println("Reached starting position.");
}

// Function to move the motor a certain number of steps (positive for clockwise, negative for counterclockwise)
void moveMotor(int direction) {
  int steps = direction * STEPS_PER_DEGREE * DEGREE_STEP;  // Move 5 degrees
  myStepper.step(steps);
}

// Function to measure distance using ultrasonic sensor
long measureDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);  // wait for the pulse to settle
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10); // trigger pulse
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH);  // measure duration of echo
  long distance = duration * 0.034 / 2;    // calculate distance in cm
  return distance;
}
