ESPnow ecosystem. 
=======
* This Repository contains the code and documentation for controlling my robotics projects. I currently have a controller, rover and arm however I plan to add to in the future. The idea of this project was to create an ecosystem in which all my projects use the same control scheme. This would allow me to use the same controller and program to control all my current and future projects. It would also speed up my development process as I wouldn't have to start from nothing every time. 
  
 
Controller
-----------
* This is the core of the ecosystem as it connects and controls the other projects. It currently has 2 joysticks, 3 potentiometers, 2 toggle switches and 2 momentary switches.

Rover
-----------
* This is the most mobile part of the ecosystem providing a way to move a sizeable payload from one spot to another. However, it could not load itself which led to the development of the arm. The rover is very simple it follows a differential steering model with 3 wheels and 2 motors. I chose this simple design as it minimised the chances of something breaking. The fewer parts there are the less thing that can go wrong. This was a very big lesson for me after building the controller which had so many things that could go wrong.

Arm
-----------
* This is the most versatile part of the ecosystem. When mounted to the rover it allows it to load payloads by itself. When mounted statically it can perform other actions like moving blocks or other simple manipulations. The arm is controlled by 5 motors. 3 of which are stepper motors that control the primary axis of the arm. The other 2 are servo motors which rotate and operate the gripper on the end of the arm.

MAC addresses:
-----------
* Controller (1) C8:2E:18:F7:F5:40    (0xC8, 0x2E, 0x18, 0xF7, 0xF5, 0x40)
* Arm        (2) C8:2E:18:F7:AA:80    (0xC8, 0x2E, 0x18, 0xF7, 0xAA, 0x80)
* Rover      (4) 10:06:1C:82:A3:D4    (0x10, 0x06, 0x1C, 0x82, 0xA3, 0xD4)

* unassigned (3) C8:2E:18:F8:0A:8C
* unassigned (5) C8:2E:18:F8:07:CC


