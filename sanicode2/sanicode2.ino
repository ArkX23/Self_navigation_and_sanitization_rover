#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <Servo.h>

// WiFi credentials (replace with your own)
const char* ssid = "Arjun";      // Your WiFi network name
const char* password = "qwerty123";  // Your WiFi password

// Define relay pins
const int RELAY2_PIN = 0;   // GPIO0 (D3 on NodeMCU)
const int RELAY3_PIN = 2;   // GPIO2 (D4 on NodeMCU)
const int RELAY4_PIN = 14;  // GPIO14 (D5 on NodeMCU)

// Define servo pin
const int SERVO_PIN = 12;   // GPIO12 (D6 on NodeMCU)

// Create servo object
Servo servo;

bool relay2Loop = false;    // Flag to control Relay 2 loop
bool servoLoop = false;     // Flag to control Servo loop

// Create a web server object on port 80
ESP8266WebServer server(80);

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(115200);

  // Set relay pins as outputs
  pinMode(RELAY2_PIN, OUTPUT);
  pinMode(RELAY3_PIN, OUTPUT);
  pinMode(RELAY4_PIN, OUTPUT);

  // Initially turn off all relays (HIGH for low-level trigger)
  digitalWrite(RELAY2_PIN, HIGH);
  digitalWrite(RELAY3_PIN, HIGH);
  digitalWrite(RELAY4_PIN, HIGH);

  // Attach servo to its pin
  servo.attach(SERVO_PIN);
  servo.write(0);  // Set initial position to 0 degrees

  // Connect to WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Connected! IP address: ");
  Serial.println(WiFi.localIP());

  // Define web server routes
  server.on("/", handleRoot);          // Root page
  server.on("/1", handleCommand1);     // Command 1: Relay 3 ON, then Relay 4
  server.on("/2", handleCommand2);     // Command 2: Start Relay 2 loop
  server.on("/3", handleCommand3);     // Command 3: Stop loop and turn off all
  server.on("/4", handleCommand4);     // Command 4: Start Servo cleaning motion

  // Start the server
  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  // Handle incoming client requests
  server.handleClient();

  // Relay 2 loop logic (runs only if relay2Loop is true)
  if (relay2Loop) {
    digitalWrite(RELAY2_PIN, LOW);   // Turn ON Relay 2
    Serial.println("Relay 2 is ON");
    delay(5000);                     // Wait 5 seconds
    digitalWrite(RELAY2_PIN, HIGH);  // Turn OFF Relay 2
    Serial.println("Relay 2 is OFF");
    delay(5000);                     // Wait 5 seconds
  }

  // Servo loop logic (runs only if servoLoop is true)
  if (servoLoop) {
    unsigned long startTime = millis();
    while (millis() - startTime < 10000 && servoLoop) {  // Run for 10 seconds
      servo.write(180);  // Move to 90 degrees
      Serial.println("Servo at 90 degrees");
      delay(500);      // Wait 0.5 second
      servo.write(0);   // Move back to 0 degrees
      Serial.println("Servo at 0 degrees");
      delay(500);      // Wait 0.5 second
      // Check for new client requests to allow interruption
      server.handleClient();
    }
    if (servoLoop) {
      servo.write(0);   // Ensure servo returns to 0 when loop ends
      servoLoop = false; // Stop servo loop after 10 seconds
      Serial.println("Servo cleaning stopped");
    }
  }
}

// Root page handler
void handleRoot() {
  String html = "<h1>ESP8266 Relay and Servo Control</h1>";
  html += "<p><a href=\"/1\">Command 1</a>: Relay 3 ON, then Relay 4 ON after 1 sec</p>";
  html += "<p><a href=\"/2\">Command 2</a>: Start Relay 2 loop (5 sec ON/OFF cycle)</p>";
  html += "<p><a href=\"/3\">Command 3</a>: Stop loop and turn off all relays</p>";
  html += "<p><a href=\"/4\">Command 4</a>: Start Servo cleaning motion (10 sec)</p>";
  server.send(200, "text/html", html);
}

// Command 1: Relay 3 ON, then Relay 4 ON after 1 sec
void handleCommand1() {
  servoLoop = false;  // Stop servo loop if running
  servo.write(0);    // Reset servo to 0 degrees
  digitalWrite(RELAY3_PIN, LOW);
  Serial.println("Relay 3 is ON");
  delay(5000);
  digitalWrite(RELAY4_PIN, LOW);
  Serial.println("Relay 4 is ON");
  delay(2000);
  digitalWrite(RELAY2_PIN, LOW);
  Serial.println("Relay 2 is ON");
  server.send(200, "text/plain", "Command 1 executed");
}

// Command 2: Start Relay 2 loop
void handleCommand2() {
  servoLoop = false;  // Stop servo loop if running
  servo.write(0);    // Reset servo to 0 degrees
  relay2Loop = true;
  Serial.println("Relay 2 loop started");
  server.send(200, "text/plain", "Command 2 executed");
}

// Command 3: Stop Relay 2 loop and turn off all relays
void handleCommand3() {
  relay2Loop = false;
  servoLoop = false;  // Stop servo loop if running
  servo.write(0);    // Reset servo to 0 degrees
  digitalWrite(RELAY2_PIN, HIGH);
  Serial.println("Relay 2 is OFF");
  delay(1000);
  digitalWrite(RELAY4_PIN, HIGH);
  Serial.println("Relay 4 is OFF");
  delay(25000);
  digitalWrite(RELAY3_PIN, HIGH);
  Serial.println("Relay 3 is OFF");
  server.send(200, "text/plain", "Command 3 executed");
}

// Command 4: Start Servo cleaning motion
void handleCommand4() {
  servoLoop = true;    // Start servo loop
  Serial.println("Servo cleaning started");
  server.send(200, "text/plain", "Command 4 executed");
}