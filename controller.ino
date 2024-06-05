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

// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t broadcastAddress[] = {0x10, 0x06, 0x1C, 0x82, 0xA3, 0xD4};

int momentarySLPinUp = 14;   //white
int momentarySLPinDown = 12; //yellow
int momentarySRPinUp = 27;   //dark green
int momentarySRPinDown = 26; //orange

int toggleSLPin = 13; //blue
int toggleSRPin = 16; //purple

int joySLPin = 2; //orange
int joySRPin = 4; //yellow

int potLPin = 25; //white
int potMPin = 33; //grey
int potRPin = 32; //purple

int joyXLPin = 35; //white
int joyYLPin = 34; //purple

int joyXRPin = 36; //green
int joyYRPin = 39; //blue




// Structure example to send data
// Must match the receiver structure
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

esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //Serial.print("\r\nLast Packet Send Status:\t");
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
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
 
void setup() {
  // Init Serial Monitor
  Serial.begin(115200);
 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  pinMode(momentarySLPinUp, INPUT_PULLUP);
  pinMode(momentarySLPinDown, INPUT_PULLUP);
  pinMode(momentarySRPinUp, INPUT_PULLUP);
  pinMode(momentarySRPinDown, INPUT_PULLUP);

  pinMode(joySLPin, INPUT_PULLUP);
  pinMode(joySRPin, INPUT_PULLUP);
}
 
void loop() {
  myData.deviceID = 1;
  // Set values to send
  Serial.println();
  //Serial.print(digitalRead(momentarySLPinUp));
  //Serial.print(digitalRead(momentarySLPinDown));
  if(digitalRead(momentarySLPinUp) == LOW){
    myData.momentarySL = 1;
  }else if(digitalRead(momentarySLPinDown) == LOW){
    myData.momentarySL = -1;
  }else{
    myData.momentarySL = 0;
  }

  if(digitalRead(momentarySRPinUp) == LOW){
    myData.momentarySR = 1;
  }else if(digitalRead(momentarySRPinDown) == LOW){
    myData.momentarySR = -1;
  }else{
    myData.momentarySR = 0;
  }

  myData.toggleSL = digitalRead(toggleSLPin);
  myData.toggleSR = digitalRead(toggleSRPin);

  myData.joySL = digitalRead(joySLPin);
  myData.joySR = digitalRead(joySRPin);

  myData.potL = analogRead(potLPin);
  myData.potM = analogRead(potMPin);
  myData.potR = analogRead(potRPin);

  myData.joyXL = analogRead(joyXLPin);
  myData.joyYL = analogRead(joyYLPin);

  myData.joyXR = analogRead(joyXRPin);
  myData.joyYR = analogRead(joyYRPin);

  Serial.print(sizeof(myData));
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
  delay(100);
}
