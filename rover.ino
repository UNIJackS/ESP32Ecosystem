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

const int motorAForwardPin = 4;//purple
const int motorAReversePin = 16;//white


const int motorBForwardPin = 2;//orange
const int motorBReversePin = 15;//yellow

//------------------PWM--------------------

const int freq = 2000;

const int resolution = 8;

const int motorAForwardChannel = 0;
const int motorAReverseChannel = 1;

const int motorBForwardChannel = 2;
const int motorBReverseChannel = 3;

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
  int deviceID; // 1,2,3,4 or 5 depending on device

  int momentarySL; //  -1,0,1 | down,middle,up
  int momentarySR; //  -1,0,1 | down,middle,up

  int toggleSL; //      0,1 | down,up
  int toggleSR; //      0,1 | down,up

  int joySL; //         0,1 | down,up
  int joySR; //         0,1 | down,up

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

  //stops running if arm contorl is slected
  if(myData.toggleSR == 1){
    ledcWrite(motorAForwardChannel, 0);
    ledcWrite(motorAReverseChannel, 0);

    ledcWrite(motorBForwardChannel, 0);
    ledcWrite(motorBReverseChannel, 0);
    return;
  }

  //checks if the left joy stick is forward then writes its positon to the motor driver
  if(myData.joyYL > 2000){
    Serial.println("left forward");
    int mappedJoyYL = map((myData.joyXL), 2057, 4095, 0, 255);

    Serial.println(mappedJoyYL);
    ledcWrite(motorAForwardChannel, mappedJoyYL);
  }else{
    ledcWrite(motorAForwardChannel, 0);
  }

  //checks if the left joy stick is back then writes its positon to the motor driver
  if(myData.joyYL < 1800){
    Serial.println("left backward");
    int mappedJoyYL = map((myData.joyXL), 0, 2037, 0, 255);

    mappedJoyYL = 255-mappedJoyYL;

    Serial.println(mappedJoyYL);
    ledcWrite(motorAReverseChannel, mappedJoyYL);
  }else{
    ledcWrite(motorAReverseChannel, 0);
  }

  //checks if the right joy stick is forward then writes its positon to the motor driver
  if(myData.joyYR > 2000){
    Serial.println("right forward");
    int mappedJoyYR = map((myData.joyXR), 2057, 4095, 0, 255);

    Serial.println(mappedJoyYR);
    ledcWrite(motorBForwardChannel, mappedJoyYR);
  }else{
    ledcWrite(motorBForwardChannel, 0);
  }
  //checks if the right joy stick is back then writes its positon to the motor driver
  if(myData.joyYR < 1800){
    Serial.println("right backward");
    int mappedJoyYR = map((myData.joyXR), 0, 2037, 0, 255);

    mappedJoyYR = 255-mappedJoyYR;

    Serial.println(mappedJoyYR);
    ledcWrite(motorBReverseChannel, mappedJoyYR);
  }else{
    ledcWrite(motorBReverseChannel, 0);
  }

}
 
void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);


  //sets motor contorl pins as outputs
  pinMode(motorAForwardPin, OUTPUT);
  pinMode(motorAReversePin, OUTPUT);

  pinMode(motorBForwardPin, OUTPUT);
  pinMode(motorBReversePin, OUTPUT);

  //sets the pwm channels
  ledcSetup(motorAForwardChannel, freq, resolution);
  ledcSetup(motorAReverseChannel, freq, resolution);

  ledcSetup(motorBForwardChannel, freq, resolution);
  ledcSetup(motorBReverseChannel, freq, resolution);

  //Attaches channel to the GPIO to be controlled
  ledcAttachPin(motorAForwardPin, motorAForwardChannel);
  ledcAttachPin(motorAReversePin, motorAReverseChannel);

  ledcAttachPin(motorBForwardPin, motorBForwardChannel);
  ledcAttachPin(motorBReversePin, motorBReverseChannel);
  
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

}
