# Magnetic Field Line-Following Robot

An embedded robotics project built with a **PIC32 remote controller and an STM32-based robot**, capable of autonomous magnetic field navigation, manual joystick control, wireless communication, and automated object pickup and delivery. 

## Features

### Autonomous Navigation
- Magnetic field sensing for guide-wire following. Done via three LC tank circuits tuned to the guide wire's signal, each with an LM358 amplifier and a peak detector feeding the STM32's ADC.
- Intersection-based navigation using predefined routes.
- Reverse-path execution.
- Custom route loading from CSV files on a PC via Bluetooth or through the remote controller
- Adjustable autonomous driving speed.
- Path progress tracking and destination detection.

### Manual Control
- Joystick-based directional control.
- PWM-based motor speed adjustment.
- Manual claw actuation.
- Path selection and autonomous/manual mode switching.

### Wireless Communication and Telemetry
- Custom Infrared (IR) communication protocol between the remote controller and robot. Each packet is a flag pulse, a 100 ms gap, and a data pulse, with values encoded in pulse length. The PIC32 generates the 38 kHz carrier by toggling the IR LED from a 76 kHz timer interrupt, and the STM32 measures pulse lengths and decodes them with a lookup table. 
- Two-way Bluetooth communication for route configuration, robot status, and path-position updates. An AT-09 BLE module streams coil readings, intersection count, and battery percentage to a Python dashboard. 
- Audio feedback indicating the selected route.

### Automated Object and obstacle Handling
- IR sensor-based object detection.
- A servo motor claw for automatic object pickup and delivery.
- Integration of navigation and claw control for autonomous operation.
- A VL53L0X time-of-flight sensor over I2C pauses the robot when an obstacle is within 100 mm.

### User Feedback
- **Blue LED:** Illuminates while the robot is following a route.
- **Green LED:** Flashes five times upon arrival and remains on while idle.
- A 16x2 LCD shows the mode, a path progress bar, and an options menu for speed, path selection, reverse mode, and custom paths on the remote controller.
- Speaker feedback corresponding to the selected path.

## System architecture 

### Remote (PIC32MX130)
 - 40 MHz MIPS32 core; joystick, pushbuttons with software debouncing, 16x2 LCD, speaker, LEDs, and IR LED driven by Timer2 interrupts.
### Robot (STM32L051)
 - 32 MHz ARM Cortex-M0+; IR receiver (TIM21), three inductor sensors (ADC), VL53L0X (I2C), IR proximity sensor, and AT-09 BLE module (USART2).
Two gear motors driven by discrete MOSFET H-bridges, isolated from the logic by an LTV-847 optocoupler, with 100 Hz software PWM from TIM2. A servo drives the claw at 50 Hz.





Developed as part of the University of British Columbia's ELEC 291/292 engineering coursework.
