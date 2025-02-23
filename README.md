# RISC-V Talent Development Program Powered by SAMSUNG and VSD
This is a RISC-V Internship using VSDSquadron Mini based  on RISC-V architecture and uses open-source tools to teach students about VLSI SoC design and RISC-V. The instructor and guide for this internship program is Mr. Kunal Ghosh, Co-Founder of VSD.

# ABOUT ME
Name: MEGHA R P
-
College: Sahyadri College of Engineering and Management, Adyar, Mangaluru.
-
Email ID: megharp.ec22@sahyadri.edu.in or ramachandrahpoojary@gmail.com
-
LinkedIn: [MEGHA R P](https://www.linkedin.com/in/megha-r-p-7a714426a)
-
# TASK 5
# PROJECT: Parking Assistant System

# OVERVIEW

The Parking Assistant System is designed to help drivers park their vehicles safely by using an ultrasonic sensor to detect the distance between the car and an obstacle. Based on the measured distance, LED indicators light up in different colors to alert the driver:

Green LED: Safe distance (no need to stop).

Yellow LED: Caution (medium distance, slow down).

Red LED: Danger (very close, stop immediately).

The system continuously measures the distance and updates the LED status in real time, providing a visual parking assistant.

# COMPONENTS REQUIRED

Microcontroller: VSDsquadron Mini (CH32V003F4U6)

Ultrasonic Sensor: HC-SR04

LEDs: Green, Yellow, Red

Resistors: 220Ω (for LEDs)

Jumper Wires

Breadboard

# HARDWARE CONNECTIONS

Ultrasonic Sensor (HC-SR04) Connections

VCC → Connect to 3.3V on VSDsquadron Mini

GND → Connect to GND on VSDsquadron Mini

TRIG → Connect to PA1 on VSDsquadron Mini

ECHO → Connect to PA2 on VSDsquadron Mini

# LED Connections

Green LED: Positive (Anode) → PD1 

Yellow LED: Positive (Anode) → PD2 

Red LED: Positive (Anode) → PD3 

All LED negatives (Cathode) → GND

