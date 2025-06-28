# 🤖   - WRO 2025 Future Engineers: KU STEAM Pinkies
## 📚 Table of Contents
- [Team Introduction](#team-introduction)
- [Robot Movement Control](#robot-movement-control)
- [Power and Sensor Management](#power-and-sensor-management)
- [Obstacle Avoidance](#obstacle-avoidance)
- [Encountered Problems and Solutions](#encountered-problems-and-solutions)
- [Robot Construction](#robot-construction)
- [Robot Control Architecture](#robot-control-architecture)
- [Electronics Wiring Diagram](#electronics-wiring-diagram)
- [Ideas and Principles](#ideas-and-principles)
- [Video](#video)
- [Code](#code)
- [3D models](#3d-models)

WRO Future Engineers documentation 
Team Name: KU STEAM TP-40 
Project Title:   
## Team Introduction
We are the KU STEAM TP-40 team 
 
participating in the WRO Future Engineers category. 
Our team is composed of enthusiastic young engineers, each contributing to different aspects of the project with a shared goal — to develop a smart and autonomous rover capable of navigating and making decisions independently. 
Team Members and Roles: 
Project Manager – All team members: Marius, Domas, Jonas 
Programmer – Marius 
Electronics Specialist – Jonas 
Mechanical Designer – Domas 
Tester – All team members: Marius, Domas, Jonas
 
By combining our individual strengths, responsibilities, and teamwork, we were able to design, build, and refine a reliable autonomous rover ready for competition. 
Our Goal: 
To create an autonomous, environment-recognizing robot capable of efficient movement under any conditions, avoiding obstacles without human intervention. 
Turinys 
 
 
## Robot Movement Control
A Raspberry Pi camera is used for image processing with OpenCV: for color recognition, obstacle analysis, and decision-making. 
The robot moves using a single-motor drive powering the rear wheels. The direction is controlled using a gear and rack mechanism that adjusts the angle of the front axle wheels. A servo motor, controlled via a PWM signal, allows precise steering adjustments. 
To ensure stable movement parallel to a wall, two TOF (Time-of-Flight) sensors are mounted on the same side: 
x₁ – front (beam AA₁) 
x₂ – rear (beam BB₁) 
Important: Beams AA₁ and BB₁ are parallel because both TOF sensors are mounted parallel to each other, perpendicular to the robot's chassis. This allows precise estimation of: 
Whether the robot is angled relative to the wall (Δx) Control is based on two differences: 
Δx = x₂ − x₁ (angular tilt) 
Δd = d₁ − d₂ (distance difference from the wall) 
Δx is used to correct the angle – if the front distance is smaller than the rear (or vice versa), the robot isn't parallel to the wall. The system adjusts the front axle angle until Δx ≈ 0. 
To avoid sudden and uncontrolled turns that could cause instability, the turning angle of the front axle is adjusted gradually using a simple PID (Proportional–Integral–Derivative) control algorithm. 
PID logic ensures that: 
Reactions to errors (Δx or Δd) are proportional to their size 
No sudden jumps – angle changes gradually depending on the rate of change 
Long-term deviations are corrected smoothly, even if sensor data briefly fluctuates 
As a result, the robot moves smoothly, without overloading the steering mechanism or jumping between trajectories. 
This control allows the robot to: 
Maintain a smooth, straight trajectory 
Avoid colliding with walls 
Move safely even in narrow paths 
 
 
 
## Power and Sensor Management
The robot is powered by a single Power Bank (5V), supplying energy to the entire system: 
Raspberry Pi – image processing and control 
Arduino – sensor data handling and low-level control 
Motor and servo motor – movement and steering mechanism 
Power-saving solutions: 
No unnecessary sensors – each component has a clear function 
Minimum required number of sensors to save power and reduce construction complexity 
Both Arduino and Raspberry Pi use the same power source 
Sensors used: 
4 TOF (Time-of-Flight) sensors – for precise navigation along walls 
1 ultrasonic sensor – for front obstacle detection 
Camera – used to recognize obstacles and help avoid them 
## Obstacle Avoidance
Obstacles are detected using a front-mounted ultrasonic sensor, continuously measuring the distance to objects. When the distance drops below a threshold, the robot initiates an avoidance maneuver without stopping. 
The avoidance algorithm is based on: 
Color recognition via the camera and OpenCV 
Predefined logic indicating which way to turn depending on the detected color 
When the ultrasonic sensor detects an obstacle: 
The camera captures the obstacle 
OpenCV filters the color and identifies the object’s color 
Based on the recognized color, the robot immediately decides which direction to turn – e.g., red = left, blue = right 
Without stopping, the servo adjusts the front axle angle via the gear and rack mechanism to maneuver around the obstacle. 
After avoidance, the robot returns to a parallel trajectory using TOF sensor data (Δx, Δd) to realign relative to the wall. 
This setup allows: 
Smooth obstacle avoidance without stopping 
Fast and efficient reactions 
Real-time adaptation to different situations 
 
## Encountered Problems and Solutions
Servo Sensitivity o Problem: Sudden servo angle changes caused instability o Solution: Smooth PID-based control applied 
Ultrasonic Measurement Inaccuracy o Problem: Wall reflections caused unstable readings o Solution: Replaced with more accurate TOF sensors 
Color Recognition Errors o Problem: Lighting conditions interfered with color detection o Solution: Adjusted HSV ranges for different lighting scenarios 
Power Supply Issues o Problem: Single Power Bank couldn't supply peak current 
Solution: Used a more powerful Power Bank (2.4A output) with a separate servo stabilizer 
Ultrasonic Inaccuracy for Side Navigation 
Problem: Angled reflection caused incorrect measurements, leading to wall drift 
Solution: Switched to TOF optical sensors for side distance measurement 
## Robot Construction
Structure Design Focus: 
Durability 
Lightweight 
Easy repair 
Chassis and Frame: 
Base made from plywood cut with CNC 
3D Printed Parts: 
PLA plastic parts for the gear and rack mechanism 
3D printed servo mounts, sensor holders, and steering rod joints 
Chosen for quick design adjustments and part replacements during testing 
Drive System: 
Rear wheels are drive wheels 
Front wheels steered by a servo motor 
Rubber wheels ensure good traction and reduced slipping, especially on smooth surfaces 
Sensor Placement: 
Two TOF sensors on each side (front and rear) for wall distance measurements 
Front-mounted ultrasonic sensor pointed forward for early obstacle detection 
Electronics Mounting: 
Raspberry Pi and Arduino mounted inside on a plywood panel 
Cables neatly grouped with clips to reduce disconnection risk during tests 
## Robot Control Architecture
The robot has two clearly separated control layers: 
Raspberry Pi – handles high-level tasks: OpenCV image processing, obstacle analysis, decision-making 
Arduino – manages low-level operations: reads sensors, controls motors and servos in real time 
Controller Communication: 
Raspberry Pi and Arduino communicate via serial console (UART) 
Allows fast data exchange 
Examples: 
Raspberry Pi ➔ Arduino: CMD:STEER=+15 – steer 15° to the right 
Arduino ➔ Raspberry Pi: SENS:TOF1=245 – sensor feedback Simple text format makes the messages easy to decode, log, and debug 
Why this approach? 
Clear separation: Raspberry Pi “thinks”, Arduino “acts” – clean code, each part does its job 
Stability: Arduino reliably performs fast actions, even if Raspberry is busy processing  Easy testing: each controller can be tested independently 
## Electronics Wiring Diagram
The robot’s electronic system is designed with clear separation between power, control, and signal transmission circuits. 
Controllers: 
Raspberry Pi – camera, OpenCV, behavior control 
Arduino Mega 2560 – low-level control (sensors, motors, servos)  Chosen for its large number of I/O pins needed for: 
TOF sensors via XSHUT o Multiple PWM outputs o UART communication 
Wiring Principles: 
Power (5V Power Bank): 
USB ➔ Raspberry Pi 
USB ➔ step-down converter ➔ Arduino Mega  Between Raspberry Pi and Arduino Mega: o Simple USB cable 
Communication via serial console (UART) 

Raspberry Pi computer and USB cable used for communication with Arduino Mega via console UART interface Sensor Shield (on Arduino Mega): 
Extends connection options 
Simplifies linking TOF sensors, ultrasonic sensor, servos 
Supports direct digital and I2C component connections 
Motor Shield: 
Installed on Mega for DC motor control using PWM and direction control 

Arduino Mega 2560 controller with two mounted modules: Sensor Shield and Motor Shield. 
Sensor Management: 
4 TOF Sensors (VL53L0X): 
Connected via shared I2C line 
Each uses a separate XSHUT pin for address assignment 
 
Four TOF (Time-of-Flight) sensors used for robot navigation along the wall. Connected via I2C, each controlled via XSHUT (marked in red). 
Ultrasonic Sensor (HC-SR04): 
Trig and Echo connected to digital pins on Mega 
 
Ultrasonic distance sensor used for obstacle detection in front of the robot. 
Camera: 
Connected to Raspberry Pi, used for OpenCV processing 
 
Raspberry Pi camera used for image processing with OpenCV: for color recognition, obstacle analysis, and decision-making. 
 
## Ideas and Principles
This was one of the most complex projects we’ve worked on. We followed several guiding rules and ideas that helped us make decisions quickly and efficiently. 
Core Principles: 
Reliability over speed – better to be a bit slower, but always working 
Simplicity is key – avoid overly complex solutions 
Iterative improvement – enhance step by step 
Testing over theory – everything is tested in practice 
Mistakes = data – failure means learning something new 
Effective Ideas: 
Switched from ultrasonic to TOF for more stable side detection 
Used PID control for smooth turning 
3D printed mounts for fast design changes 
Calibrated color ranges in the actual competition room 
Kept code structured: sensor reading, decision-making, and action execution separated 
Built the frame from plywood for strength without excess weight 
Tested under real conditions – recreated the map at home for accuracy 
 
Conclusion 
We believe these principles helped us build a functional, reliable robot. If our experience helps someone else – we’ll be even happier! 
## Video

## Code
All of the code with comments https://github.com/domas-cloud/TP-40/tree/main/code 
## 3D models
3d models stl and svg files are in github https://github.com/domas-cloud/TP-40/tree/main/models 
 
 
 
 
 
 
 