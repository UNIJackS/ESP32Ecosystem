This is a repository for my ESPnow ecosystem. 
=======
* I initially controlled my robotic projects using Arduinos and 433Mhz radios. This worked and was very reliable however I could not get the refresh rate as high as I would have liked. This led me to get 5 eps32 boards and use ESPnow which is what you are seeing now.
 
Controller
-----------
* This is the core of the ecosystem as it connects and controls the other projects. It currently has 2 joysticks, 3 pots, 2 toggle switches and 2 momentary switches. 

Rover
-----------
* This was the first part of the ecosystem that was finished. The Program provides a way to control a differential drive rover from at least 20 meters away (the furthest I've tested so far).

Arm
-----------
* This program provides a way to control a robotic arm. Currently, it has 3 stepper motors that control its major movement and 2 servos to move the gripper.


MAC addresses:
-----------
* Controller (1) C8:2E:18:F7:F5:40    (0xC8, 0x2E, 0x18, 0xF7, 0xF5, 0x40)
* Arm        (2) C8:2E:18:F7:AA:80    (0xC8, 0x2E, 0x18, 0xF7, 0xAA, 0x80)
* Rover      (4) 10:06:1C:82:A3:D4    (0x10, 0x06, 0x1C, 0x82, 0xA3, 0xD4)

* unassigned (3) C8:2E:18:F8:0A:8C
* unassigned (5) C8:2E:18:F8:07:CC


