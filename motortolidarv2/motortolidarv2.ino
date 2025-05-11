#include <WiFi.h>
#include <WiFiUdp.h>

const char* ssid = "Arjun";
const char* password = "qwerty123";
const char* udpAddress = "192.168.121.158";  // Ubuntu IP
const int odomPort = 12346;                  // Odometry port
const int cmdPort = 12347;                   // Cmd_vel port
const int resetPort = 12348;                 // Reset port

#define IN1 12
#define IN2 14
#define ENA 13
#define IN3 27
#define IN4 26
#define ENB 25
#define ENCODER_LEFT_A 33
#define ENCODER_LEFT_B 32
#define ENCODER_RIGHT_A 35
#define ENCODER_RIGHT_B 34

const float WHEELBASE = 0.13;          // 13 cm
const float WHEEL_CIRCUMFERENCE = 0.314;
const float COUNTS_PER_REV = 4680.0;
const float DISTANCE_PER_COUNT = WHEEL_CIRCUMFERENCE / COUNTS_PER_REV;

WiFiUDP udpOdom;
WiFiUDP udpCmd;
WiFiUDP udpReset;  // New reset UDP

volatile long leftEncoderCount = 0, rightEncoderCount = 0;
float x = 0.0, y = 0.0, yaw = 0.0;

void IRAM_ATTR leftEncoderISR() {
    if (digitalRead(ENCODER_LEFT_B)) leftEncoderCount--; else leftEncoderCount++;
}

void IRAM_ATTR rightEncoderISR() {
    if (digitalRead(ENCODER_RIGHT_B)) rightEncoderCount--; else rightEncoderCount++;
}

void setup() {
    Serial.begin(115200);
    pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(ENA, OUTPUT);
    pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT); pinMode(ENB, OUTPUT);
    pinMode(ENCODER_LEFT_A, INPUT); pinMode(ENCODER_LEFT_B, INPUT);
    pinMode(ENCODER_RIGHT_A, INPUT); pinMode(ENCODER_RIGHT_B, INPUT);

    attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), leftEncoderISR, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), rightEncoderISR, RISING);

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500); Serial.print(".");
    }
    Serial.println("\nConnected to WiFi");
    Serial.print("Motor ESP IP: "); Serial.println(WiFi.localIP());

    udpOdom.begin(odomPort);
    udpCmd.begin(cmdPort);
    udpReset.begin(resetPort);  // Start reset port
}

void resetOdometry() {
    leftEncoderCount = 0;
    rightEncoderCount = 0;
    x = 0.0;
    y = 0.0;
    yaw = 0.0;
    Serial.println("Odometry reset: x=0, y=0, yaw=0");
}

void loop() {
    static long lastLeftCount = 0, lastRightCount = 0;
    static unsigned long lastTime = millis();

    // Check for reset command
    if (udpReset.parsePacket()) {
        char packet[16];
        int len = udpReset.read(packet, 15);
        packet[len] = '\0';
        if (strcmp(packet, "reset") == 0) {
            resetOdometry();
        }
        Serial.print("Reset packet: "); Serial.println(packet);
    }

    // Check for cmd_vel packets
    if (udpCmd.parsePacket()) {
        char packet[32];
        int len = udpCmd.read(packet, 31);
        packet[len] = '\0';

        float linear = atof(strtok(packet, ","));
        float angular = atof(strtok(NULL, ","));

        float leftSpeed = linear - (angular * WHEELBASE / 2);
        float rightSpeed = linear + (angular * WHEELBASE / 2);
        float maxSpeed = 0.4;
        leftSpeed = constrain(leftSpeed, -maxSpeed, maxSpeed);
        rightSpeed = constrain(rightSpeed, -maxSpeed, maxSpeed);

        int leftPWM = abs(leftSpeed) * 255 / maxSpeed;
        int rightPWM = abs(rightSpeed) * 255 / maxSpeed;

        if (leftSpeed > 0) {
            digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
        } else if (leftSpeed < 0) {
            digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
        } else {
            digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
        }
        if (rightSpeed > 0) {
            digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
        } else if (rightSpeed < 0) {
            digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
        } else {
            digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
        }
        analogWrite(ENA, leftPWM);
        analogWrite(ENB, rightPWM);

        Serial.print("Received cmd_vel: "); Serial.print(linear); Serial.print(","); Serial.println(angular);
    }

    // Send odometry every 100ms
    unsigned long currentTime = millis();
    if (currentTime - lastTime >= 100) {
        long deltaLeft = leftEncoderCount - lastLeftCount;
        long deltaRight = rightEncoderCount - lastRightCount;
        lastLeftCount = leftEncoderCount;
        lastRightCount = rightEncoderCount;
        float dt = (currentTime - lastTime) / 1000.0;
        lastTime = currentTime;

        float leftDistance = deltaLeft * DISTANCE_PER_COUNT;
        float rightDistance = deltaRight * DISTANCE_PER_COUNT;
        float distance = (leftDistance + rightDistance) / 1;
        float deltaYaw = (rightDistance - leftDistance) / WHEELBASE;

        yaw += deltaYaw;
        x += distance * cos(yaw);
        y += distance * sin(yaw);
        if (yaw > PI) yaw -= 2 * PI;
        if (yaw < -PI) yaw += 2 * PI;

        char packet[64];
        snprintf(packet, sizeof(packet), "%.3f,%.3f,%.3f", x, y, yaw);
        udpOdom.beginPacket(udpAddress, odomPort);
        udpOdom.write((uint8_t*)packet, strlen(packet));
        udpOdom.endPacket();
    }
}