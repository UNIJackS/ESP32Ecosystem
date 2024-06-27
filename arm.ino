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

//------------------PINS--------------------
const int primaryYDirPin = 27;    //purple
const int primaryYStepPin = 14;   //blue

const int secondaryYDirPin = 25;  //purple
const int secondaryYStepPin = 26; //blue

const int XDirPin = 12;           //white
const int XStepPin = 13;          // grey


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

long lastPolTime = millis();

int primaryYDir = 0;    //-1,0,1 reverse,stop,forward
int secondaryYDir = 0;  //-1,0,1 reverse,stop,forward
int XDir = 0;           //-1,0,1 reverse,stop,forward

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));

  dumpData();

  //stops running if rover contorl is slected
  if(myData.toggleSL == 1){
    
    return;
  }


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
  if(myData.joyXR > 2000){
    Serial.println("right pointing right");
    XDir = 1;
  }else if(myData.joyXR < 1700){
    Serial.println("right pointing left");
    XDir = -1;
  }else{
    XDir = 0;
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

  pinMode(primaryYDirPin, OUTPUT);
  pinMode(primaryYStepPin, OUTPUT);

  pinMode(secondaryYDirPin, OUTPUT);
  pinMode(secondaryYStepPin, OUTPUT);

  pinMode(XDirPin, OUTPUT);
  pinMode(XStepPin, OUTPUT);

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
