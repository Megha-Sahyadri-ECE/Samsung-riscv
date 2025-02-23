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
## Tasks
- **TASK1:** Development of C Based LAB  
- **TASK2:** Simulation with Spike  
- **TASK3:** Identification of RISC-V Instructions  
- **TASK4:** Functional Simulation of RISC-V Core  
- **TASK5:** Project Overview - Circuit Diagram  
- **TASK6:** Project Application  

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
# CODE
#include "ch32v00x.h"  // Include the CH32V00x specific header file
#include <stdint.h>     // Include standard integer types
#include <stdio.h>      // Include stdio.h for snprintf

// Define pins for Ultrasonic Sensor and LEDs
#define trigPin 1  // PA1
#define echoPin 2  // PA2
#define greenLedPin 1  // PD1
#define yellowLedPin 2  // PD2
#define redLedPin 3  // PD3

long duration;
int distance;

// Function prototypes
void delay_ms(int ms);
void gpio_set(int pin, int level);
int gpio_get(int pin);
void uart_init(void);
void uart_send(char data);
void uart_print(const char* str);
long pulseIn(int pin);

void setup() {
  // Initialize UART for debugging
  uart_init();
  uart_print("Initializing Parking Assistant...\n");

  // Set GPIO pins for ultrasonic sensor and LEDs
  gpio_set(trigPin, 0);  // Set trigger pin low initially
  gpio_set(echoPin, 1);  // Set echo pin as input (assumed)
  gpio_set(greenLedPin, 0);  // Set LED pins low initially
  gpio_set(yellowLedPin, 0);
  gpio_set(redLedPin, 0);

  uart_print("Setup complete.\n");
}

void loop() {
  // Trigger the ultrasonic sensor
  gpio_set(trigPin, 0);
  delay_ms(2);
  gpio_set(trigPin, 1);
  delay_ms(10);
  gpio_set(trigPin, 0);
  
  // Measure pulse duration
  duration = pulseIn(echoPin);
  
  // Calculate distance in cm
  distance = duration * 0.034 / 2;
  
  // Control LEDs based on distance
  if (distance > 50) {
    gpio_set(greenLedPin, 1);  // Green LED for safe distance
    gpio_set(yellowLedPin, 0);
    gpio_set(redLedPin, 0);
  } 
  else if (distance > 20 && distance <= 50) {
    gpio_set(greenLedPin, 0);
    gpio_set(yellowLedPin, 1);  // Yellow LED for caution
    gpio_set(redLedPin, 0);
  } 
  else {
    gpio_set(greenLedPin, 0);
    gpio_set(yellowLedPin, 0);
    gpio_set(redLedPin, 1);  // Red LED for danger
  }

  // Print distance to UART for debugging
  char buf[50];
  snprintf(buf, sizeof(buf), "Distance: %d cm\n", distance);
  uart_print(buf);

  delay_ms(100);
}

void delay_ms(int ms) {
  // Implement a simple delay function (blocking)
  for (int i = 0; i < ms; i++) {
    for (int j = 0; j < 1000; j++) {
      __asm("nop");  // No operation, just a delay
    }
  }
}

void gpio_set(int pin, int level) {
  // Set the GPIO pin to HIGH or LOW (level 1 or 0)
  if (level == 0) {
    if (pin == trigPin || pin == echoPin) {
      GPIO_ResetBits(GPIOA, 1 << pin);  // Reset pin to LOW (for PA1 and PA2)
    } else {
      GPIO_ResetBits(GPIOD, 1 << pin);  // Reset pin to LOW (for PD1, PD2, PD3)
    }
  } else {
    if (pin == trigPin || pin == echoPin) {
      GPIO_SetBits(GPIOA, 1 << pin);    // Set pin to HIGH (for PA1 and PA2)
    } else {
      GPIO_SetBits(GPIOD, 1 << pin);    // Set pin to HIGH (for PD1, PD2, PD3)
    }
  }
}

int gpio_get(int pin) {
  // Get the level of the GPIO pin (HIGH or LOW)
  if (pin == echoPin) {
    return GPIO_ReadInputDataBit(GPIOA, 1 << pin);  // Return the pin state for PA2 (Echo)
  } else {
    return GPIO_ReadInputDataBit(GPIOD, 1 << pin);  // Return the pin state for PD1, PD2, PD3 (LEDs)
  }
}

long pulseIn(int pin) {
  // Measure the duration of a pulse on the specified pin
  long duration = 0;

  // Wait for the pin to go HIGH
  while (gpio_get(pin) == 0);
  
  // Measure the pulse duration while the pin is HIGH
  while (gpio_get(pin) == 1) {
    duration++;
    delay_ms(1);  // Short delay to simulate timing
  }

  return duration;
}

void uart_init(void) {
  // Initialize UART for debugging (assumed to be UART1 on CH32V00x)
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);  // Enable USART1 clock
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);   // Enable GPIOA clock (for UART TX/RX)

  // Set PA9 (TX) and PA10 (RX) for UART1
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  // Initialize UART1
  USART_InitTypeDef USART_InitStructure;
  USART_InitStructure.USART_BaudRate = 9600;    // Baud rate 9600
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;  // 8 data bits
  USART_InitStructure.USART_StopBits = USART_StopBits_1;        // 1 stop bit
  USART_InitStructure.USART_Parity = USART_Parity_No;            // No parity
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
  USART_Init(USART1, &USART_InitStructure);

  USART_Cmd(USART1, ENABLE);  // Enable UART1
}

void uart_send(char data) {
  // Send a single character over UART
  while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);  // Wait for TX buffer to be empty
  USART_SendData(USART1, data);  // Send the character
}

void uart_print(const char* str) {
  // Send a string over UART
  while (*str) {
    uart_send(*str++);  // Send each character
  }
}


