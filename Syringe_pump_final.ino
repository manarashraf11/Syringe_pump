#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

// Set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Motor control pins
int in1 = 2;
int in2 = 3;
int enA = 9;  // Enable A, PWM pin

// Buzzer control pin
int buzzerPin = 4;

// Door sensor pin
int doorSensorPin = 6;

// Water flow sensor pin
int flowSensorPin = 5; 

// Ultrasonic sensor pins
const int trigPin = 7;  
const int echoPin = 8; 

// Battery voltage pin
const int batteryPin = A0;  // Adjust this as needed

// Desired flow rate in L/min
float desiredFlowRate = 1;  // Adjust this value as needed

// Variables to store the previous values
float previousDesiredFlowRate = 0.0;
float previousCurrentFlowRate = 0.0;
int previousDoorState = LOW;

void setup() {
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enA, OUTPUT);

  pinMode(buzzerPin, OUTPUT);

  pinMode(doorSensorPin, INPUT_PULLUP);

  pinMode(flowSensorPin, INPUT);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  lcd.init(); // initialize the lcd 
  lcd.backlight(); // open the backlight
  lcd.clear();
  Serial.begin(9600);
}

void loop() {
  // Read the current flow rate
  int flow = pulseIn(flowSensorPin, HIGH);  // Measure pulse width
  float currentFlowRate = 1000.0 / (flow * 2.5);  // Convert pulse width to flow rate

  // Calculate the error in flow rate
  float error = desiredFlowRate - currentFlowRate;

  // Adjust the motor speed based on the error
  int pwmValue = 1500 + (int)(1000 * error);  // Adjust the scaling factor as needed
  pwmValue = constrain(pwmValue, 0, 255);  // Ensure PWM value is between 0 and 255

  // Move the motor forward
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(enA, pwmValue);  // Adjust PWM value for desired speed
  delay(2000);  // Run for 2 seconds

 // Stop the motor
 digitalWrite(in1, LOW);
 digitalWrite(in2, LOW);
 analogWrite(enA, 0);
 delay(1000);  // Pause for 1 second

  // Door sensor
  int doorState = digitalRead(doorSensorPin);
  if (doorState != previousDoorState) {
    if (doorState == HIGH) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Door is OPEN");
    } else {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Door is CLOSED");
    }
    previousDoorState = doorState;
  }

  // Display the pre-adjusted flow rate value and the sensed value if they have changed and the door is closed
  if (doorState == LOW && (currentFlowRate != previousCurrentFlowRate || desiredFlowRate != previousDesiredFlowRate)) {
    lcd.clear();
    lcd.print("Set Rate " + String((int)desiredFlowRate) + "L/m");
    lcd.setCursor(0, 1);
    lcd.print("Curr Rate " + String((int)currentFlowRate) + "L/m");

    previousDesiredFlowRate = desiredFlowRate;
    previousCurrentFlowRate = currentFlowRate;
  }

  // Check for occlusion
  if (currentFlowRate < 0.9 * desiredFlowRate) {  // Adjust the threshold as needed
    Serial.println("Occlusion detected!");
    digitalWrite(buzzerPin, HIGH);  // Turn the buzzer on
    delay(200);  // Wait for 200 milliseconds
    digitalWrite(buzzerPin, LOW);  // Turn the buzzer off
  }

  // Ultrasonic sensor
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  float distance = duration * 0.034 / 2;

  // Check for air bubbles
  if (distance < 10) {  // Adjust the threshold as needed
    Serial.println("Air bubbles detected!");
    digitalWrite(buzzerPin, HIGH);  // Turn the buzzer on
    delay(200);  // Wait for 200 milliseconds
    digitalWrite(buzzerPin, LOW);  // Turn the buzzer off
  }

  // Check battery voltage
  int batteryValue = analogRead(batteryPin);
  float batteryVoltage = (batteryValue / 1023.0) * 5.0;  // Convert ADC value to voltage

  // Check for low battery
  if (batteryVoltage < 5) {  // Adjust the threshold as needed
    Serial.println("Low battery!");
    digitalWrite(buzzerPin, HIGH);  // Turn the buzzer on
    delay(200);  // Wait for 200 milliseconds
    digitalWrite(buzzerPin, LOW);  // Turn the buzzer off
  }
  delay(1000);
}
