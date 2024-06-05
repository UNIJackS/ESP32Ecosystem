/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp-now-esp32-arduino-ide/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/

#include <esp_now.h>
#include <WiFi.h>
#include <AccelStepper.h>

//------------------PINS--------------------
const int primaryYDirPin = 1;
const int primaryYStepPin = 2;

const int secondaryYDirPin = 3;
const int secondaryYStepPin = 4;

const int XDirPin = 5;
const int XStepPin = 6;

//------------------Driver Objects--------------------

AccelStepper primaryY(AccelStepper::FULL2WIRE, primaryYStepPin, primaryYDirPin);
AccelStepper secondaryY(AccelStepper::FULL2WIRE, secondaryYStepPin, secondaryYDirPin);
AccelStepper x(AccelStepper::FULL2WIRE, XStepPin, XDirPin);

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
  int deviceID; // 1,2,3,4 or 5 depending on device

  int momentarySL; //  -1,0,1 down,middle,up
  int momentarySR; //  -1,0,1 down,middle,up

  int toggleSL; //      0,1 down,up
  int toggleSR; //      0,1 down,up

  int joySL; //         0,1 down,up
  int joySR; //         0,1 down,up

  int potL; //          0-1023 analog value
  int potM; //          0-1023 analog value
  int potR; //          0-1023 analog value

  int joyXL; //         0-1023 analog value
  int joyYL; //         0-1023 analog value

  int joyXR; //         0-1023 analog value
  int joyYR; //         0-1023 analog value
} struct_message;

// Create a struct_message called myData
struct_message myData;

void dumpData(){
  Serial.print(myData.momentarySL);
  Serial.print(",");
  Serial.print(myData.momentarySR);
  Serial.print(",");

  Serial.print(myData.toggleSL);
  Serial.print(",");
  Serial.print(myData.toggleSR);
  Serial.print(",");

  Serial.print(myData.joySL);
  Serial.print(",");
  Serial.print(myData.joySR);
  Serial.print(",");

  Serial.print(myData.potL);
  Serial.print(",");
  Serial.print(myData.potM);
  Serial.print(",");
  Serial.print(myData.potR);
  Serial.print(",");

  Serial.print(myData.joyXL);
  Serial.print(",");
  Serial.print(myData.joyYL);
  Serial.print(",");

  Serial.print(myData.joyXR);
  Serial.print(",");
  Serial.print(myData.joyYR);
  Serial.print(",");
  Serial.println("");
}



// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));

  dumpData();

  //stops running if rover contorl is slected
  if(myData.toggleSL == 1){
    
    return;
  }


  //primary y axis driving
  if(myData.joyXL > 2000){
    Serial.println("left forward");
    primaryY.moveTo(primaryY.currentPosition() + 10);
  }
  if(myData.joyXL < 1800){
    Serial.println("left backward");
    primaryY.moveTo(primaryY.currentPosition() - 10);
  }
  
  //secondary y axis driving
  if(myData.joyXR > 2000){
    Serial.println("right forward");
    secondaryY.moveTo(secondaryY.currentPosition() + 10);
  }
  if(myData.joyXR < 1800){
    Serial.println("right backward");
    secondaryY.moveTo(secondaryY.currentPosition() - 10);
  }

  //secondary y axis driving
  if(myData.joyYR > 2000){
    Serial.println("right pointing right");
    x.moveTo(x.currentPosition() + 10);
  }
  if(myData.joyYR < 1800){
    Serial.println("right pointing left");
    x.moveTo(x.currentPosition() - 10);
  }

}
 
void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);

  primaryY.setMaxSpeed(1000);
  primaryY.setAcceleration(100);
  primaryY.setSpeed(200);

  secondaryY.setMaxSpeed(1000);
  secondaryY.setAcceleration(100);
  secondaryY.setSpeed(200);
  
  x.setMaxSpeed(1000);
  x.setAcceleration(100);
  x.setSpeed(200);
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);
}
 
void loop() {
 primaryY.run();
 secondaryY.run();
 x.run();

}
