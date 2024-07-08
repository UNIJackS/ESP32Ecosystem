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
#include <ESP32Servo.h>

//------------------PINS--------------------
const int primaryYDirPin = 27;    //purple
const int primaryYStepPin = 14;   //blue

const int secondaryYDirPin = 25;  //purple
const int secondaryYStepPin = 26; //blue

const int XDirPin = 12;           //white
const int XStepPin = 13;          // grey

const int gripperPin = 33; 
const int wristPin = 32; 

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

  int potL; //          0 to 4095 | left to right
  int potM; //          0 to 4095 | left to right
  int potR; //          0 to 4095 | left to right

  int joyXL; //         0 to 4095 | left to right
  int joyYL; //         0 to 4095 | bottom to top

  int joyXR; //         0 to 4095 | left to right
  int joyYR; //         0 to 4095 | bottom to top
} struct_message;

// Create a struct_message called myData
struct_message myData;

// Creates servo objects to contorl the wrist and gripper
Servo gripperServo;
Servo wristServo;

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

int primaryYDir = 0;    //-1,0,1 reverse,stop,forward
int secondaryYDir = 0;  //-1,0,1 reverse,stop,forward
int XDir = 0;           //-1,0,1 reverse,stop,forward

int gripperPos = 0; // 0,73   open to closed
int wristPos = 0;   // 0,180  rotation of wrist

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));

  //dumpData();

  //stops running if rover contorl is slected
  if(myData.toggleSL == 1){
    
    return;
  }

  //-------------Stepper Driving-------------
  //primary y axis driving
  if(myData.joyYL > 2000){
    Serial.println("left forward");
    primaryYDir = 1;
  }else if(myData.joyYL < 1700){
    Serial.println("left backward");
    primaryYDir = -1;
  }else{
    primaryYDir = 0;
  }
  
  //secondary y axis driving
  if(myData.joyYR > 2000){
    Serial.println("right forward");
    secondaryYDir = 1;
  }else if(myData.joyYR < 1700){
    Serial.println("right backward");
    secondaryYDir = -1;
  }else{
    secondaryYDir = 0;
  }

  //secondary y axis driving
  if(myData.joyXL > 2000){
    Serial.println("right pointing right");
    XDir = 1;
  }else if(myData.joyXL < 1700){
    Serial.println("right pointing left");
    XDir = -1;
  }else{
    XDir = 0;
  }
  //-------------Servo Driving-------------

  wristPos = map(myData.potR,0,4095,0,180);

  if(myData.joyXR > 2000 && (gripperPos + 1) <= 73){
    Serial.println("Gripper opening");
    gripperPos += 1;
  }else if(myData.joyXR < 1700 && (gripperPos - 1) >= 0){
    Serial.println("Gripper closing");
    gripperPos -= 1;
  }

}

void direction(int pin, int value) {
  if(value == 1){
    digitalWrite(pin, LOW);
  }else if(value == -1){
    digitalWrite(pin, HIGH);
  }
}
 
void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);

  //-------------Stepper Driving-------------
  pinMode(primaryYDirPin, OUTPUT);
  pinMode(primaryYStepPin, OUTPUT);

  pinMode(secondaryYDirPin, OUTPUT);
  pinMode(secondaryYStepPin, OUTPUT);

  pinMode(XDirPin, OUTPUT);
  pinMode(XStepPin, OUTPUT);

  //-------------Servo Driving-------------
  ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);

	gripperServo.setPeriodHertz(50);   
	gripperServo.attach(gripperPin, 500, 2400); 
  
  wristServo.setPeriodHertz(50);    
	wristServo.attach(wristPin, 500, 2400);

  //-------------ESP NOW------------- 
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
  //-------------Servo Driving-------------
 
  
  gripperServo.write(gripperPos);
  wristServo.write(wristPos);

  //-------------Stepper Driving-------------
  direction(primaryYDirPin, primaryYDir);
  direction(secondaryYDirPin, secondaryYDir);
  direction(XDirPin, XDir);
  if(primaryYDir != 0){
    digitalWrite(primaryYStepPin, HIGH);
  }
  if(secondaryYDir != 0){
    digitalWrite(secondaryYStepPin, HIGH);
  }
  if(XDir != 0){
    digitalWrite(XStepPin, HIGH);
  }
  delay(1);
  digitalWrite(primaryYStepPin, LOW);
  digitalWrite(secondaryYStepPin, LOW);
  digitalWrite(XStepPin, LOW);
  delay(1);
}
