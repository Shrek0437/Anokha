#include <dummy.h>

#include<SoftwareSerial.h> 
#include<TinyGPSPlus.h>
#include<TinyGPS++.h>


const int outputPin1 = D0;  // LED1 to glow by default always
const int outputPin2 = D4;   // LED2 to glow based on motion detection
const int inputPin = D5;
const int SensorPin = D3; 
#define GPS_TX_PIN D2 // GPS module TX pin connected to NodeMCU D2 pin
#define GPS_RX_PIN D1 // GPS module RX pin connected to NodeMCU D1 pin

TinyGPSPlus gps;
SoftwareSerial gpsSerial(D5, D6);
void setup() {
  Serial.print("streetlightfaultdetection");
  pinMode(outputPin1, OUTPUT);
  pinMode(outputPin2, OUTPUT);
  pinMode(inputPin, INPUT);
  pinMode(SensorPin, INPUT);
  Serial.begin(9600);
  digitalWrite(outputPin1, HIGH);
}

void loop() {
  
    int sensorValue = digitalRead(SensorPin); //SensorPin is from output of Proximity Sensor
    Serial.println(sensorValue);

  if (sensorValue == LOW) {
    digitalWrite(outputPin2, HIGH); // Turn on the LED when motion is detected
    Serial.println("Vehicle detected!");
  } else {
    digitalWrite(outputPin2, LOW); // Turn off the LED when no motion is detected
    Serial.println("Vehicle Not detected"); // PROXIMITY SENSOR
  }
  bool fault=digitalRead(inputPin); //inpurPin is from output of current sensor
  Serial.println(fault);
  
  delay(1000);
  
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      if (gps.location.isValid()) {
        // Print GPS data
        Serial.print("Latitude: ");
        Serial.println(gps.location.lat(), 6);
        Serial.print("Longitude: ");
        Serial.println(gps.location.lng(), 6);
        
        // Send GPS data to ThingSpeak
        //sendToThingSpeak(gps.location.lat(), gps.location.lng());
     }
}
}
}