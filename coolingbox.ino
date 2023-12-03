#include <DHT.h>
#include <SoftwareSerial.h>
#include <ESP8266WiFi.h>
#include <ThingSpeak.h>

#define DHT_PIN 14       // D5
#define FAN_SPEED_PIN 2  // D4
#define OUT1 4           // D2
#define OUT2 0           // D3
#define GSM_TX 15        // D8
#define GSM_RX 13        // D7
#define PUMP_PIN 5      // D1

// thingSpeak connections
const char *ssid = "HOMENET";
const char *password = "Vacuum1234";
const char *thingSpeakApiKey = "O9KL7KSBX0OPFP69";
const long thingSpeakChannelID = 2364655;

// constants for PID parameters
const float kp = 60.0;  // Proportional gain
const float ki = 0.0001; // Integral gain
const float kd = 2.0;   // Derivative gain

const float setpointTemperature = 15.0;
float temperature, humidity, error, last_error, integral, derivative, fanspeed;

// Define variables for time-based control
unsigned long previousMillis = 0;
const long interval = 1000; // Time interval for control loop

// Client for ThingSpeak communication
WiFiClient client;

// initialize the DHT sensor
DHT dht(DHT_PIN, DHT11);

// initializa GSM module
SoftwareSerial mySerial(GSM_RX, GSM_TX);

void setup()
{
  // initializing the baud rate
  Serial.begin(9600);


  // initialize the DHT sensor
  dht.begin();

  // set pin to start the pump
  pinMode(PUMP_PIN, OUTPUT);

  // set fan pins as fanspeed
  pinMode(FAN_SPEED_PIN, OUTPUT);

  // set pins for fan direction
  pinMode(OUT1, OUTPUT);
  pinMode(OUT2, OUTPUT);

  // initialize the PID variables
  last_error = 0;
  integral = 0;

  // begin serial communication
  mySerial.begin(9600);

  Serial.println("Initializing..."); 
  delay(1000);
  mySerial.println("AT"); //Once the handshake test is successful, it will back to OK
  updateSerial();
}

void loop()
{
  // Read temperature and humidity from DHT sensor
  humidity = dht.readHumidity();
  temperature = dht.readTemperature();


  // Check if any reads failed and exit early (to try again).
  if (isnan(humidity) || isnan(temperature))
  {
    Serial.println("Failed to read from DHT sensor!");
    delay(2000);
    return;
  }

  // Calculate the error
  error = temperature - setpointTemperature;

  // Calculate the integral and derivative terms
  integral += error;
  derivative = error - last_error;

  // Calculate the control fanspeed using PID
  // fanspeed = abs(kp * error + ki * integral + kd * derivative);
  fanspeed = kp * error + ki * integral + kd * derivative;

  // Limit the control fanspeed to avoid full-speed fan
  if (fanspeed > 255)
  {
    fanspeed = 255;
  }
  else if (fanspeed < 0)
  {
    fanspeed = 0;
  }

  // Update the fan speed
  // set fan direction
  digitalWrite(OUT1, LOW);
  digitalWrite(OUT2, HIGH);
  analogWrite(FAN_SPEED_PIN, fanspeed);

  if (fanspeed > 125.0) {
    digitalWrite(PUMP_PIN, HIGH);
  }

  // Save the current error for the next iteration
  last_error = error;

  // if (temperature < setpointTemperature) {
  //   sendText();
  // }

  // Print temperature and control fanspeed
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.print(" °C, Setpoint: ");
  Serial.print(setpointTemperature);
  Serial.print(" Proportional: ");
  Serial.print(error);
  Serial.print(" Integral: ");
  Serial.print(integral);
  Serial.print(" Derivative: ");
  Serial.print(derivative);
  Serial.print(" °C, Fan Speed: ");
  Serial.println(fanspeed);
  delay(2000);

  // Add a delay to control the loop rate
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval)
  {
    previousMillis = currentMillis;
  }
}

void updateSerial()
{
  delay(500);
  while (Serial.available()) 
  {
    mySerial.write(Serial.read());//Forward what Serial received to Software Serial Port
  }
  while(mySerial.available()) 
  {
    Serial.write(mySerial.read());//Forward what Software Serial received to Serial Port
  }
}

void sendText() {
  mySerial.println("AT+CMGF=1"); // Configuring TEXT mode
  updateSerial();
  // mySerial.println("AT+CMGS=\"+254719286396\"");//change ZZ with country code and xxxxxxxxxxx with phone number to sms
  mySerial.println("AT+CMGS=\"+254708221455\"");
  updateSerial();
  mySerial.print("Temperature: "); mySerial.println(temperature);//text content
  mySerial.print("Humidity: "); mySerial.println(humidity);
  mySerial.print("Temperature is okay!");
  updateSerial();
  mySerial.write(26);
}
