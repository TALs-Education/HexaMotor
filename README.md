
# HexaMotor
### Position & Velocity Control Experiments

## Overview

HexaMotor is a hands-on experimental platform designed for university-level education, emphasizing the nuances of position and velocity control. 
Powered by Simulink, it not only generates the code for the embedded setup but also serves as an intuitive interface. 
This design choice allows students to concentrate on control implementation rather than the intricacies of code writing. 
With a foundation built around a DC motor, HexaMotor offers a tangible experience of system dynamics and control.

![Setup Image](./Images/HexaMotor%20Velocity%20Control.jpg)

![Simulink model](./Images/HexaMotor%20Position%20Control%20simulink.jpg)

## Velocity Control Experiment

### Subjects Covered

- **Introduction to DC Motor Dynamic Equations**
- **Coulomb Friction Identification and Handling**
- **System Identification Using Open Loop Step Response**  
- **System Identification Using Frequency Response**
- **Velocity Control Implementation: Proportional & Proportional-Integral Control Loops**
- **Controller Performance Under Load**

  ![Plot for Close Loop Step Response](./Images/HexaMotor%20Velocity%20Control%20step%20response.jpg)
  
## Position Control Experiment

### Subjects Covered

- **Introduction to Second Order Systems**
- **Coulomb Friction Identification and Handling**
- **System Identification Using Position Close Loop Step Response**  
- **System Identification Using Position Close Loop Frequency Response**
- **Position Control Implementation Based on Second Order System Modeling**
- **Lead-Lag Compensator Controller Design and Implementation**
- **Force FeedForward Example: Compliant Motor Control**

  ![Force sensor Image](./Images/HexaMotor%20Position%20Control.jpg)

  ![Plot for Position Close Loop Step Response](./Images/HexaMotor%20Position%20Control%20controller%20design.jpg)

  ![Plot for Frequency Response, System Identification](./Images/HexaMotor%20Position%20Control%20frequency%20response.jpg)
  
## Setup & Usage

### 4.1. DC Motor

This gearmotor is a combination of a 12 V brushed DC motor with a 20.4:1 metal spur gearbox. It also incorporates a 48 CPR quadrature encoder on the motor shaft, providing 979.62 counts per revolution of the gearbox’s output shaft.

- **Product page**: [Pololu Gearmotor](https://www.pololu.com/product/4883)
- **Data Sheet**: [Pololu 25D Metal Gearmotors PDF](https://www.pololu.com/file/0J1829/pololu-25d-metal-gearmotors.pdf)

### 4.2. Arduino Boards

The MKR Motor Carrier is an add-on board designed for MKR to control servo, DC, and stepper motors.

- **MKR Zero**: [Arduino MKR Zero Documentation](https://docs.arduino.cc/hardware/mkr-zero)
- **MKR Motor Carrier**: [Arduino Store - MKR Motor Carrier](https://store-usa.arduino.cc/products/arduino-mkr-motor-carrier)

### 4.3. Simulink Support Package

Utilize MATLAB and Simulink Support Packages for Arduino® hardware to interactively communicate with your Arduino. With Simulink, you can also perform model deployment for standalone operations on Arduino boards.

- **MathWorks Support**: [MATLAB & Simulink for Arduino](https://www.mathworks.com/hardware-support/arduino.html)

### 4.4. 3D Model – Fusion360

The entire setup has been designed in Fusion360 as an open-source project meant for educational applications.

- **Fusion360 Project**: [HexaMotor Fusion360 Model](https://a360.co/3TU1BYP)

### Usage Instructions

For detailed instructions on setting up and running experiments, refer to the provided manuals:

- **Position Control**: [HexaMotor Position Control Manual (PDF)](HexaMotor%20Position%20Control.pdf)
- **Velocity Control**: [HexaMotor Velocity Control Manual (PDF)](HexaMotor%20Velocity%20Control.pdf)

## License

This project is licensed under the GNU General Public License v3.0 - see the [LICENSE](LICENSE) file for details.
