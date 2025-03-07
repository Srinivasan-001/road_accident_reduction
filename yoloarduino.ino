#include <ESP8266WiFi.h>
#include <LiquidCrystal_I2C.h>
#include <WiFiUdp.h>

// WiFi Configuration - Replace with your actual credentials
const char* ssid = "Srini";
const char* password = "1234567890";

// Server Configuration
WiFiUDP udpServer;
const int UDP_PORT = 4210;  // Choose a port that's not commonly used

// LCD Configuration
LiquidCrystal_I2C lcd(0x27, 16, 2);  // I2C address 0x27, 16 column and 2 rows

// Buzzer Pin Configuration
const int BUZZER_PIN = D7;  // GPIO pin for buzzer

// Motor Driver Pins
const int MOTOR_PIN1 = D1;  // Motor driver input 1
const int MOTOR_PIN2 = D2;  // Motor driver input 2
const int MOTOR_ENABLE = D3;  // Motor PWM pin

// Alert Duration and Frequency
const int ALERT_DURATION = 2000;  // 2 seconds
const int BUZZER_FREQUENCY_DROWSY = 1000;  // 1kHz for drowsiness
const int BUZZER_FREQUENCY_PHONE = 2000;   // 2kHz for phone use
const int BUZZER_FREQUENCY_SPEED = 1500;   // 1.5kHz for speed limit alert

// Speed Limit Variables
int currentSpeedLimit = 0;
int lastSpeedLimit = 0;

// Timer for returning to welcome message
unsigned long alertDisplayTime = 0;
const unsigned long DISPLAY_TIMEOUT = 3000;  // 3 seconds
bool alertActive = false;

void setup() {
  // Initialize Serial Communication
  Serial.begin(115200);

  // Initialize Motor Driver Pins
  pinMode(MOTOR_PIN1, OUTPUT);
  pinMode(MOTOR_PIN2, OUTPUT);
  pinMode(MOTOR_ENABLE, OUTPUT);

  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Driver Monitor");
  lcd.setCursor(0, 1);
  lcd.print("Initializing...");

  // Initialize Buzzer
  pinMode(BUZZER_PIN, OUTPUT);

  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  // Start UDP Listener
  udpServer.begin(UDP_PORT);

  // Clear LCD and show IP
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("IP:");
  lcd.setCursor(0, 1);
  lcd.print(WiFi.localIP().toString());

  delay(2000);  // Show IP for 2 seconds
  
  // Display welcome message after IP
  displayWelcomeMessage();
}

void displayWelcomeMessage() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Welcome Driver!");
  lcd.setCursor(0, 1);
  lcd.print("Drive Safely");
  alertActive = false;
}

void triggerBuzzer(int frequency, int duration) {
  tone(BUZZER_PIN, frequency, duration);
  delay(duration);  // Ensure buzzer sounds for full duration
  noTone(BUZZER_PIN);
}

void displayAlert(const String& alertType) {
  // Clear LCD
  lcd.clear();
  alertActive = true;
  alertDisplayTime = millis();

  // Display Alert
  if (alertType == "DROWSY") {
    lcd.setCursor(0, 0);
    lcd.print("DROWSINESS ALERT!");
    lcd.setCursor(0, 1);
    lcd.print("Stay Awake!");

    // Specific buzzer for drowsiness
    triggerBuzzer(BUZZER_FREQUENCY_DROWSY, ALERT_DURATION);
  }
  else if (alertType == "PHONE") {
    lcd.setCursor(0, 0);
    lcd.print("PHONE USE");
    lcd.setCursor(0, 1);
    lcd.print("DETECTED!");

    // Specific buzzer for phone use
    triggerBuzzer(BUZZER_FREQUENCY_PHONE, ALERT_DURATION);
  }
  else if (alertType == "SPEED") {
    lcd.setCursor(0, 0);
    lcd.print("SPEED LIMIT");
    lcd.setCursor(0, 1);
    lcd.print(String(currentSpeedLimit) + " km/h");

    // Buzzer for speed limit
    triggerBuzzer(BUZZER_FREQUENCY_SPEED, ALERT_DURATION);
  }
}

void controlMotorSpeed(int speedLimit) {
  // Reduce motor speed proportionally
  int pwmValue = map(speedLimit, 0, 100, 0, 255);
  analogWrite(MOTOR_ENABLE, pwmValue);
  
  // Ensure motor continues running
  digitalWrite(MOTOR_PIN1, HIGH);
  digitalWrite(MOTOR_PIN2, LOW);
}

void loop() {
  // Check if it's time to return to the welcome message
  if (alertActive && (millis() - alertDisplayTime > DISPLAY_TIMEOUT)) {
    displayWelcomeMessage();
  }

  // Prepare UDP packet buffer
  char packetBuffer[255];
  int packetSize = udpServer.parsePacket();

  // Check UDP messages
  if (packetSize) {
    // Read the packet
    int len = udpServer.read(packetBuffer, 255);
    if (len > 0) {
      packetBuffer[len] = 0;  // Null-terminate the string
      
      String message = String(packetBuffer);

      // Check for specific alert types
      if (message.startsWith("ALERT:")) {
        String alertType = message.substring(6);
        displayAlert(alertType);
      }
    }
  }

  // Check for speed limit message from Serial
  if (Serial.available() > 0) {
    String message = Serial.readStringUntil('\n');
    
    if (message.startsWith("SPEED:")) {
      // Extract speed limit
      int speedLimit = message.substring(6).toInt();
      
      // Update current speed limit
      currentSpeedLimit = speedLimit;
      
      // Display speed limit alert if changed
      if (speedLimit != lastSpeedLimit) {
        displayAlert("SPEED");
        controlMotorSpeed(speedLimit);
        lastSpeedLimit = speedLimit;
      }
    }
  }

  delay(100);  // Small delay to prevent overwhelming the system
}