/*
====================================================================
                 AUTONOMOUS RACING CAR
                     ESP32-S3 + RPLIDAR A1
====================================================================
*/

#include <Arduino.h>
#include <ESP32Servo.h>

// MOTOR PINS
const int LEFT_MOTOR_IN1 = 5;
const int LEFT_MOTOR_IN2 = 6;
const int LEFT_MOTOR_PWM = 7;
const int RIGHT_MOTOR_IN1 = 9;
const int RIGHT_MOTOR_IN2 = 10;
const int RIGHT_MOTOR_PWM = 8;

// STEERING
const int STEERING_SERVO_PIN = 4;
Servo steeringServo;
const int STEERING_CENTER = 90;
const int STEERING_LEFT = 55;
const int STEERING_RIGHT = 125;

// BUTTONS
const int START_BUTTON = 11;
const int POWER_BUTTON = 12;
const int ESTOP_BUTTON = 13;

// ENCODERS
const int LEFT_ENCODER_PIN = 14;
const int RIGHT_ENCODER_PIN = 15;
volatile long leftEncoderTicks = 0;
volatile long rightEncoderTicks = 0;

// RPLIDAR A1 UART
const int LIDAR_RX_PIN = 16;
const int LIDAR_TX_PIN = 17;
HardwareSerial LidarSerial(1);
const uint32_t LIDAR_BAUD = 115200;

// RPLIDAR COMMANDS
const uint8_t RPLIDAR_CMD_STOP = 0x25;
const uint8_t RPLIDAR_CMD_SCAN = 0x20;
const uint8_t RPLIDAR_CMD_RESET = 0x40;

// ENCODER SETTINGS
const float TICKS_PER_REVOLUTION = 20.0;
const float WHEEL_DIAMETER_MM = 65.0;
const float WHEEL_CIRCUMFERENCE_MM = PI * WHEEL_DIAMETER_MM;

// SPEED SETTINGS
int currentSpeed = 0;
const int MIN_SPEED = 80;
const int MAX_SPEED = 255;
const int ACCELERATION_STEP = 2;
const int DECELERATION_STEP = 8;

// LiDAR SETTINGS
const float SAFE_DISTANCE_CM = 50.0;
const float SLOW_DISTANCE_CM = 100.0;
const float TURN_DISTANCE_CM = 150.0;

// LiDAR DATA
float lidarDistance[360];
unsigned long lidarLastUpdate[360];

float frontDistance = 999.0;
float leftDistance = 999.0;
float rightDistance = 999.0;
float rearDistance = 999.0;

// RACING DATA
float totalDistance = 0.0;
float currentSpeedKmh = 0.0;

// CAR STATE
bool powerOn = false;
bool carRunning = false;
bool emergencyStop = false;
bool obstacleDetected = false;
bool turnAhead = false;

// TIMERS
unsigned long lastSpeedCalculation = 0;
unsigned long lastSensorAnalysis = 0;
unsigned long lastPrint = 0;

void IRAM_ATTR leftEncoderISR() {
    leftEncoderTicks++;
}

void IRAM_ATTR rightEncoderISR() {
    rightEncoderTicks++;
}

void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println();
    Serial.println("========================================");
    Serial.println("       AUTONOMOUS RACING CAR");
    Serial.println("       ESP32-S3 + RPLIDAR A1");
    Serial.println("========================================");

    initializePins();
    initializeMotors();
    initializeSteering();
    initializeEncoders();
    initializeButtons();
    initializeLidar();

    stopCar();
    centerSteering();

    Serial.println();
    Serial.println("SYSTEM READY");
    Serial.println("Press POWER button.");
}

void loop() {
    checkEmergencyStop();

    if (emergencyStop) {
        emergencyBrake();
        return;
    }

    checkPowerButton();

    if (!powerOn) {
        stopCar();
        return;
    }

    checkStartButton();

    if (!carRunning) {
        stopCar();
        return;
    }

    processLidar();
    updateEncoderDistance();
    calculateSpeed();

    if (millis() - lastSensorAnalysis >= 50) {
        lastSensorAnalysis = millis();
        updateDirectionDistances();
        detectObstacles();
        detectTurn();
    }

    autonomousDrive();
    printSensorData();

    delay(2);
}

void initializePins() {
    pinMode(LEFT_MOTOR_IN1, OUTPUT);
    pinMode(LEFT_MOTOR_IN2, OUTPUT);
    pinMode(RIGHT_MOTOR_IN1, OUTPUT);
    pinMode(RIGHT_MOTOR_IN2, OUTPUT);
    pinMode(LEFT_MOTOR_PWM, OUTPUT);
    pinMode(RIGHT_MOTOR_PWM, OUTPUT);
    Serial.println("Pins initialized.");
}

void initializeMotors() {
    ledcAttach(LEFT_MOTOR_PWM, 1000, 8);
    ledcAttach(RIGHT_MOTOR_PWM, 1000, 8);
    stopCar();
    Serial.println("Motors initialized.");
}

void initializeSteering() {
    steeringServo.attach(STEERING_SERVO_PIN);
    steeringServo.write(STEERING_CENTER);
    Serial.println("Steering initialized.");
}

void initializeEncoders() {
    pinMode(LEFT_ENCODER_PIN, INPUT_PULLUP);
    pinMode(RIGHT_ENCODER_PIN, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN), leftEncoderISR, RISING);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN), rightEncoderISR, RISING);

    Serial.println("Encoders initialized.");
}

void initializeButtons() {
    pinMode(START_BUTTON, INPUT_PULLUP);
    pinMode(POWER_BUTTON, INPUT_PULLUP);
    pinMode(ESTOP_BUTTON, INPUT_PULLUP);
    Serial.println("Buttons initialized.");
}

void initializeLidar() {
    LidarSerial.begin(
        LIDAR_BAUD,
        SERIAL_8N1,
        LIDAR_RX_PIN,
        LIDAR_TX_PIN
    );

    delay(100);

    while (LidarSerial.available()) {
        LidarSerial.read();
    }

    rplidarStop();
    delay(100);
    rplidarStart();

    Serial.println("RPLIDAR A1 initialized.");
}

void rplidarStop() {
    uint8_t command[] = {0xA5, RPLIDAR_CMD_STOP};
    LidarSerial.write(command, sizeof(command));
    LidarSerial.flush();
}

void rplidarReset() {
    uint8_t command[] = {0xA5, RPLIDAR_CMD_RESET};
    LidarSerial.write(command, sizeof(command));
    LidarSerial.flush();
    delay(100);
}

void rplidarStart() {
    uint8_t command[] = {0xA5, RPLIDAR_CMD_SCAN};
    LidarSerial.write(command, sizeof(command));
    LidarSerial.flush();
    delay(100);
}

void processLidar() {
    static uint8_t packet[5];
    static uint8_t packetIndex = 0;

    while (LidarSerial.available()) {
        uint8_t b = LidarSerial.read();

        if (packetIndex == 0) {
            if ((b & 0x01) == 0) {
                continue;
            }

            packet[0] = b;
            packetIndex = 1;
            continue;
        }

        packet[packetIndex] = b;
        packetIndex++;

        if (packetIndex >= 5) {
            packetIndex = 0;

            bool startBit = packet[0] & 0x01;
            bool inverseStartBit = packet[0] & 0x02;

            if (startBit == inverseStartBit) {
                continue;
            }

            uint8_t quality = packet[0] >> 2;

            if (quality == 0) {
                continue;
            }

            uint16_t angleRaw =
                packet[1] |
                (packet[2] << 8);

            float angle = (angleRaw >> 1) / 64.0;

            uint16_t distanceRaw =
                packet[3] |
                (packet[4] << 8);

            float distanceMM = distanceRaw / 4.0;
            float distanceCM = distanceMM / 10.0;

            if (angle < 0 || angle >= 360) {
                continue;
            }

            if (distanceCM <= 0 || distanceCM > 1200) {
                continue;
            }

            int index = (int)angle;

            if (index >= 0 && index < 360) {
                lidarDistance[index] = distanceCM;
                lidarLastUpdate[index] = millis();
            }
        }
    }
}

float getDistanceAtAngle(int angle) {
    angle = angle % 360;

    if (angle < 0) {
        angle += 360;
    }

    if (millis() - lidarLastUpdate[angle] > 500) {
        return 999.0;
    }

    return lidarDistance[angle];
}

float getMinimumDistance(int startAngle, int endAngle) {
    float minimumDistance = 999.0;

    if (startAngle <= endAngle) {
        for (int angle = startAngle; angle <= endAngle; angle++) {
            float distance = getDistanceAtAngle(angle);

            if (distance < minimumDistance) {
                minimumDistance = distance;
            }
        }
    } else {
        for (int angle = startAngle; angle < 360; angle++) {
            float distance = getDistanceAtAngle(angle);

            if (distance < minimumDistance) {
                minimumDistance = distance;
            }
        }

        for (int angle = 0; angle <= endAngle; angle++) {
            float distance = getDistanceAtAngle(angle);

            if (distance < minimumDistance) {
                minimumDistance = distance;
            }
        }
    }

    return minimumDistance;
}

void updateDirectionDistances() {
    frontDistance = getMinimumDistance(345, 15);
    rightDistance = getMinimumDistance(75, 105);
    rearDistance = getMinimumDistance(165, 195);
    leftDistance = getMinimumDistance(255, 285);
}

void detectObstacles() {
    obstacleDetected = frontDistance <= SAFE_DISTANCE_CM;
}

void detectTurn() {
    if (frontDistance < TURN_DISTANCE_CM) {
        turnAhead =
            leftDistance > frontDistance + 20 ||
            rightDistance > frontDistance + 20;
    } else {
        turnAhead = false;
    }
}

void setMotorPWM(int leftPWM, int rightPWM) {
    leftPWM = constrain(leftPWM, 0, 255);
    rightPWM = constrain(rightPWM, 0, 255);

    ledcWrite(LEFT_MOTOR_PWM, leftPWM);
    ledcWrite(RIGHT_MOTOR_PWM, rightPWM);
}

void moveForward(int speed) {
    speed = constrain(speed, 0, 255);

    digitalWrite(LEFT_MOTOR_IN1, HIGH);
    digitalWrite(LEFT_MOTOR_IN2, LOW);

    digitalWrite(RIGHT_MOTOR_IN1, HIGH);
    digitalWrite(RIGHT_MOTOR_IN2, LOW);

    setMotorPWM(speed, speed);
}

void moveReverse(int speed) {
    speed = constrain(speed, 0, 255);

    digitalWrite(LEFT_MOTOR_IN1, LOW);
    digitalWrite(LEFT_MOTOR_IN2, HIGH);

    digitalWrite(RIGHT_MOTOR_IN1, LOW);
    digitalWrite(RIGHT_MOTOR_IN2, HIGH);

    setMotorPWM(speed, speed);
}

void stopCar() {
    setMotorPWM(0, 0);

    digitalWrite(LEFT_MOTOR_IN1, LOW);
    digitalWrite(LEFT_MOTOR_IN2, LOW);

    digitalWrite(RIGHT_MOTOR_IN1, LOW);
    digitalWrite(RIGHT_MOTOR_IN2, LOW);
}

void steerLeft() {
    steeringServo.write(STEERING_LEFT);
}

void steerRight() {
    steeringServo.write(STEERING_RIGHT);
}

void centerSteering() {
    steeringServo.write(STEERING_CENTER);
}

void turnLeft(int speed) {
    steerLeft();
    moveForward(speed);
}

void turnRight(int speed) {
    steerRight();
    moveForward(speed);
}

void increaseSpeed() {
    currentSpeed += ACCELERATION_STEP;

    if (currentSpeed > MAX_SPEED) {
        currentSpeed = MAX_SPEED;
    }
}

void decreaseSpeed() {
    currentSpeed -= DECELERATION_STEP;

    if (currentSpeed < MIN_SPEED) {
        currentSpeed = MIN_SPEED;
    }
}

void emergencyBrake() {
    currentSpeed = 0;
    stopCar();
    centerSteering();
}

void controlSpeed() {
    if (frontDistance <= SAFE_DISTANCE_CM) {
        currentSpeed = 0;
        return;
    }

    if (frontDistance <= SLOW_DISTANCE_CM) {
        decreaseSpeed();
        return;
    }

    if (turnAhead) {
        decreaseSpeed();
        return;
    }

    increaseSpeed();
}

void updateEncoderDistance() {
    static long previousLeftTicks = 0;
    static long previousRightTicks = 0;

    long leftTicks;
    long rightTicks;

    noInterrupts();
    leftTicks = leftEncoderTicks;
    rightTicks = rightEncoderTicks;
    interrupts();

    long leftDifference = leftTicks - previousLeftTicks;
    long rightDifference = rightTicks - previousRightTicks;

    previousLeftTicks = leftTicks;
    previousRightTicks = rightTicks;

    float averageTicks =
        (leftDifference + rightDifference) / 2.0;

    float revolutions =
        averageTicks / TICKS_PER_REVOLUTION;

    float distanceMM =
        revolutions * WHEEL_CIRCUMFERENCE_MM;

    totalDistance += distanceMM / 1000.0;
}

void calculateSpeed() {
    unsigned long now = millis();

    if (now - lastSpeedCalculation < 100) {
        return;
    }

    static float previousDistance = 0;

    float distanceDifference =
        totalDistance - previousDistance;

    previousDistance = totalDistance;

    currentSpeedKmh =
        distanceDifference * 10.0 * 3.6;

    lastSpeedCalculation = now;
}

void resetDistance() {
    totalDistance = 0;

    noInterrupts();
    leftEncoderTicks = 0;
    rightEncoderTicks = 0;
    interrupts();

    Serial.println("Distance reset.");
}

int chooseAvoidanceDirection() {
    if (
        leftDistance > SAFE_DISTANCE_CM &&
        leftDistance > rightDistance
    ) {
        return -1;
    }

    if (
        rightDistance > SAFE_DISTANCE_CM &&
        rightDistance > leftDistance
    ) {
        return 1;
    }

    if (leftDistance > SAFE_DISTANCE_CM) {
        return -1;
    }

    if (rightDistance > SAFE_DISTANCE_CM) {
        return 1;
    }

    return 0;
}

void avoidObstacle() {
    currentSpeed = MIN_SPEED;

    int direction = chooseAvoidanceDirection();

    if (direction == -1) {
        Serial.println("OBSTACLE -> LEFT");
        steerLeft();
        moveForward(MIN_SPEED);
    } else if (direction == 1) {
        Serial.println("OBSTACLE -> RIGHT");
        steerRight();
        moveForward(MIN_SPEED);
    } else {
        Serial.println("OBSTACLE -> STOP");
        emergencyBrake();
    }
}

void handleTurn() {
    if (leftDistance > rightDistance) {
        turnLeft(currentSpeed);
    } else {
        turnRight(currentSpeed);
    }
}

void autonomousDrive() {
    if (emergencyStop) {
        emergencyBrake();
        return;
    }

    if (frontDistance <= SAFE_DISTANCE_CM) {
        emergencyBrake();
        avoidObstacle();
        return;
    }

    if (turnAhead) {
        controlSpeed();
        handleTurn();
        return;
    }

    controlSpeed();

    centerSteering();
    moveForward(currentSpeed);
}

void checkStartButton() {
    static bool lastState = HIGH;

    bool state = digitalRead(START_BUTTON);

    if (lastState == HIGH && state == LOW) {
        if (powerOn && !emergencyStop) {
            carRunning = !carRunning;

            if (carRunning) {
                currentSpeed = MIN_SPEED;
                Serial.println("RACING STARTED");
            } else {
                currentSpeed = 0;
                stopCar();
                centerSteering();
                Serial.println("RACING STOPPED");
            }
        }

        delay(50);
    }

    lastState = state;
}

void checkPowerButton() {
    static bool lastState = HIGH;

    bool state = digitalRead(POWER_BUTTON);

    if (lastState == HIGH && state == LOW) {
        powerOn = !powerOn;

        if (powerOn) {
            Serial.println("POWER ON");
            currentSpeed = 0;
        } else {
            Serial.println("POWER OFF");
            carRunning = false;
            currentSpeed = 0;
            stopCar();
            centerSteering();
        }

        delay(50);
    }

    lastState = state;
}

void checkEmergencyStop() {
    if (digitalRead(ESTOP_BUTTON) == LOW) {
        if (!emergencyStop) {
            emergencyStop = true;
            carRunning = false;
            currentSpeed = 0;
            stopCar();
            centerSteering();

            Serial.println();
            Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!");
            Serial.println("!!! EMERGENCY STOP !!!");
            Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!");
        }
    }
}

void resetEmergencyStop() {
    if (digitalRead(ESTOP_BUTTON) == HIGH) {
        emergencyStop = false;
        currentSpeed = 0;
        centerSteering();
        Serial.println("Emergency stop reset.");
    }
}

void printSensorData() {
    unsigned long now = millis();

    if (now - lastPrint < 500) {
        return;
    }

    lastPrint = now;

    Serial.println();
    Serial.println("================================");
    Serial.println("          CAR STATUS");
    Serial.println("================================");

    Serial.print("Power: ");
    Serial.println(powerOn ? "ON" : "OFF");

    Serial.print("Running: ");
    Serial.println(carRunning ? "YES" : "NO");

    Serial.print("Emergency: ");
    Serial.println(emergencyStop ? "YES" : "NO");

    Serial.println("--------------------------------");

    Serial.print("Front: ");
    Serial.print(frontDistance, 1);
    Serial.println(" cm");

    Serial.print("Left: ");
    Serial.print(leftDistance, 1);
    Serial.println(" cm");

    Serial.print("Right: ");
    Serial.print(rightDistance, 1);
    Serial.println(" cm");

    Serial.print("Rear: ");
    Serial.print(rearDistance, 1);
    Serial.println(" cm");

    Serial.println("--------------------------------");

    Serial.print("Distance: ");
    Serial.print(totalDistance, 3);
    Serial.println(" m");

    Serial.print("Speed: ");
    Serial.print(currentSpeedKmh, 2);
    Serial.println(" km/h");

    Serial.print("Motor PWM: ");
    Serial.println(currentSpeed);

    Serial.print("Obstacle: ");
    Serial.println(obstacleDetected ? "YES" : "NO");

    Serial.print("Turn: ");
    Serial.println(turnAhead ? "YES" : "NO");

    Serial.println("================================");
}
