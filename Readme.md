# Arduino Obstacle Avoiding Robot Car
Obstracle avoiding robot car using an HC-SR04 ultrasonic sensor to detect obstracle in from of the vehicle.
Build with FreeRtos and controlled by two tasks __distanceTask__ and __driveTask__. The __distanceTask__ checks for obstacles with the HC-SR04 Ultrasonic Sensor within OBSTACLE_LIMIT range and notify driveTask if obstacle is deteced.

The __driveTask__ runs as a state machine and go FORWARD until an obstacle is detected (blocks in FORWARD state until notified from distanceTask that obstacle is ahead) then change to REVERSE state and reverse for about 1 second and enter TURN state. Do a short turn in TURN state and return to FORWARD state. The TURN and FORWARD states are hold for a second +/- some random ms.

### Hardware list:

Arduino	UNO R3

4x DC Motor Tt 130Motor	DC3V-6V DC Gear Motor

Arduino sensor shield v5.0

L298 Dual full bridge driver

HC-SR04 Ultrasonic Sensor

Generic robot car frame

Powered	by 2x18650 4200 mAh 3.7V li-ion
