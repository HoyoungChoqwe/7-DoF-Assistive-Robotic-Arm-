7-DoF Assistive Robotic Manipulator
Project Overview:
This project is a 7-degree-of-freedom assistive robotic arm designed to help users with limited upper-limb mobility perform remote pick-and-place tasks. The system combines an accessible wireless controller, inverse kinematics, computer vision, and embedded servo control.
The project was developed using a distributed architecture consisting of an ESP32 remote controller, an NVIDIA Jetson Nano, and an STM32 microcontroller. Each processor was assigned a specific role based on its processing capabilities and real-time requirements.

# System Architecture
ESP32 Remote Controller
The ESP32 was used as the main user-input device. It collected commands from the joystick and control buttons and transmitted them wirelessly to the robotic-arm system.
The controller handled:
- Joystick input
- Button input
- Input dead-zone processing
- Button debouncing
- Wireless command transmission
- Communication packet generation
The controller was designed to provide a simple and accessible interface for users who may have difficulty operating traditional robotic-control systems.

# NVIDIA Jetson Nano
The Jetson Nano handled the high-level control and computation required by the system.
Its responsibilities included:
- Running the inverse-kinematics algorithm
- Receiving user commands from the remote controller
- Converting Cartesian XYZ targets into joint-angle commands
- Running the computer-vision subsystem
- Detecting AprilTags
- Generating target-position commands
- Sending calculated joint angles to the STM32
The inverse-kinematics code used the desired Cartesian position of the robotic arm to calculate the corresponding joint-angle targets. This allowed the arm to be controlled using XYZ position commands instead of requiring the user to control every joint individually.

# STM32 Motor Controller
The STM32 was responsible for low-level motor control and servo actuation.
Its responsibilities included:
- Receiving joint-angle commands from the Jetson Nano
- Parsing incoming command data
- Converting joint-angle targets into PWM values
- Controlling multiple servos simultaneously
- Applying servo calibration values
- Enforcing joint-angle limits
- Coordinating smooth robotic-arm movement
- Handling emergency-stop and safety behavior
Separating the servo-control logic from the high-level processing allowed the STM32 to focus on consistent and responsive motor commands.

# System Control Flow:
<p align="center">
  <strong>Joystick and Button Inputs</strong><br>
  ↓<br>
  <strong>ESP32 Remote Controller</strong><br>
  ↓<br>
  <strong>Wireless Communication</strong><br>
  ↓<br>
  <strong>NVIDIA Jetson Nano</strong><br>
  ↓<br>
  <strong>Inverse-Kinematics Solver</strong><br>
  ↓<br>
  <strong>Joint-Angle Commands</strong><br>
  ↓<br>
  <strong>STM32 Motor Controller</strong><br>
  ↓<br>
  <strong>PWM Servo Commands</strong><br>
  ↓<br>
  <strong>7-DoF Robotic Arm</strong>
</p>

# System Control Flow:

<p align="center">
  <strong>Camera Input</strong><br>
  ↓<br>
  <strong>AprilTag Detection</strong><br>
  ↓<br>
  <strong>Object or Target Position</strong><br>
  ↓<br>
  <strong>Inverse-Kinematics Solver</strong><br>
  ↓<br>
  <strong>Joint-Angle Commands</strong><br>
  ↓<br>
  <strong>STM32 Servo Control</strong>
</p>

# Communication System
A structured communication method was used to transfer commands between the different processors.
Controller packets included information such as:
- Packet header
- Joystick values
- Button states
- Command identifiers
- Payload data
- Error-checking information
The receiving program parsed the incoming data and rejected incomplete or invalid commands. This provided a more reliable method of communication than sending individual unstructured values.
The Jetson Nano also transmitted calculated joint-angle targets to the STM32, where they were converted into servo commands.

# Inverse Kinematics
The inverse-kinematics algorithm ran on the Jetson Nano and translated Cartesian XYZ commands into joint-angle targets for the robotic arm.
The IK control process included:
- Reading the current joint configuration
- Receiving a desired XYZ target
- Calculating the arm Jacobian
- Determining the required joint-angle changes
- Applying joint limits
- Updating the desired joint configuration
- Sending the calculated angles to the STM32
A damped least-squares approach was used to improve the behavior of the solver near singular configurations. The algorithm allowed several joints to move together rather than commanding each joint independently.

# Servo Control
The STM32 generated PWM commands for the robotic-arm servos based on the joint-angle values received from the Jetson Nano.
Servo-control development included:
- Individually calibrating each servo
- Mapping joint angles to PWM pulse widths
- Defining minimum and maximum joint limits
- Coordinating movement across multiple joints
- Debugging incorrect movement directions
- Adjusting offsets for the physical arm configuration
- Testing calculated IK angles before applying them to the arm
These calibration and safety limits were necessary because each servo and mechanical joint had a different usable motion range.

# Computer Vision
The Jetson Nano was also used for the computer-vision subsystem. AprilTags were used to identify and localize targets within the camera view.
The detected target position could be converted into a Cartesian command and passed to the inverse-kinematics solver. This supported assisted object targeting and reduced the amount of manual joint control required from the user.

# Accessible Remote Controller
The remote controller was designed with accessibility as a primary requirement.
The interface included:
- A large joystick
- Large physical buttons
- Wireless operation
- A custom enclosure
- Simple directional control
- Emergency-stop functionality
The controller enclosure was designed and adjusted based on the dimensions of the physical components and printed circuit board.

# Safety Features:
The system incorporated several software and hardware safety features:
- Joint-angle limits
- Servo command limits
- Emergency-stop input
- Idle-state behavior
- Input dead zones
- Invalid-packet rejection
- Controlled state transitions
- Servo calibration constraints
These features helped prevent the arm from moving beyond its mechanical range or responding to incomplete commands.

# Technologies Used:
- C and C++
- Python
- NVIDIA Jetson Nano
- STM32F446RE
- ESP32
- Bluetooth Low Energy
- UART communication
- PWM servo control
- Forward kinematics
- Inverse kinematics
- Jacobian-based control
- AprilTag detection
- Embedded state machines
- PlatformIO
- STM32Cube
- Git and GitHub

# My Contributions:
I served as the Embedded Systems Lead for the project. My primary contributions included:
- Developing the ESP32 remote-controller firmware
- Reading and processing joystick and button inputs
- Implementing wireless controller communication
- Designing the controller communication packet format
- Developing STM32 communication and command-parsing logic
- Implementing STM32 servo-control functions
- Calibrating the robotic-arm servos
- Defining servo offsets and joint limits
- Converting calculated joint angles into PWM commands
- Integrating the Jetson Nano IK output with STM32 motor control
- Debugging coordinated robotic-arm movement
- Testing inverse-kinematics results using manually calculated positions
- Supporting embedded, mechanical, and computer-vision integration
- Designing and testing the accessible remote-controller enclosure

# Project Results:
The completed system demonstrated:
- Wireless joystick and button control
- Communication between the ESP32, Jetson Nano, and STM32
- Cartesian XYZ command input
- Inverse-kinematics calculations on the Jetson Nano
- Coordinated multi-joint movement
- PWM-based servo actuation
- AprilTag-based target detection
- Remote and assisted robotic-arm operation
- Pick-and-place task execution
