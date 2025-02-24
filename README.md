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
<details>
<summary>TASK1:Development of C Based LAB</summary>


## leafpad installation
<img
src="https://github.com/Megha-Sahyadri-ECE/Samsung-riscv/blob/main/Task1/leafpad%20installation.JPG"/>


## c based lab output
<img 
src="https://github.com/Megha-Sahyadri-ECE/Samsung-riscv/blob/main/Task1/program%20sum%20compiler.JPG"/>

## c based lab program
  <img
src=""/>

## riscv based lab output(01)
<img
src=""/>

## riscv based lab output(0fast)
<img
src=""/>

## riscv based lab output
<img
src=""/>


# TASK 5
# PROJECT: Ultrasonic Obstacle Detection with Buzzer Alarm

# OVERVIEW

This project is an Ultrasonic-Based Object Detection and Alert System using the CH32V003 microcontroller and the HC-SR04 ultrasonic sensor. It measures the distance of nearby objects using ultrasonic waves and triggers an alert mechanism based on the detected distance. If an object is within a certain range, the system activates a buzzer to provide an audible warning. The HC-SR04 sensor is powered by 5V, while its Echo signal is safely converted to 3.3V using a voltage divider to ensure compatibility with the CH32V003 MCU. This system is useful for proximity sensing, obstacle detection, and safety applications.

# COMPONENTS REQUIRED

Microcontroller: VSDsquadron Mini (CH32V003F4U6)

Ultrasonic Sensor: HC-SR04

Buzzer

Breadboard

Jumper wires


# HARDWARE CONNECTIONS

HC-SR04 VCC to 5V: The HC-SR04 ultrasonic sensor needs 5V power to work, so connect the VCC pin of the sensor to a 5V supply.
HC-SR04 GND to GND: Connect the GND pin of the sensor to the ground of the system.
HC-SR04 Trig to PC0: The Trig pin of the sensor is connected to PC0 on the microcontroller to send the trigger signal. The 3.3V logic from the microcontroller is safe for this pin.
HC-SR04 Echo to PC1 (via voltage divider): The Echo pin from the sensor outputs 5V, but the microcontroller uses 3.3V logic. Use a voltage divider (1kΩ and 2kΩ resistors) to reduce the 5V signal to 3.3V, then connect it to PC1.
Buzzer + to PC3: The positive pin of the buzzer is connected to PC3 on the microcontroller to control when it turns on.
Buzzer - to GND: The negative pin of the buzzer is connected to ground.
# CODE
#include <ch32v00x.h>  // CH32V003 MCU headers
#include <system_ch32v00x.h>

#define TRIG_PIN   GPIO_Pin_0  // PC0 - Ultrasonic Trigger
#define ECHO_PIN   GPIO_Pin_1  // PC1 - Ultrasonic Echo
#define LED_PIN    GPIO_Pin_2  // PC2 - LED
#define BUZZER_PIN GPIO_Pin_3  // PC3 - Buzzer

void delay_us(uint32_t us) {
    for (volatile uint32_t i = 0; i < us * 8; i++) {
        __NOP();
    }
}

void delay_ms(uint32_t ms) {
    for (uint32_t i = 0; i < ms; i++) {
        delay_us(1000);
    }
}

void Ultrasonic_Init() {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);  // Enable GPIOC clock
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);   // Enable TIM2 clock

    GPIO_InitTypeDef GPIO_InitStruct;

    // Initialize Trigger Pin (Output)
    GPIO_InitStruct.GPIO_Pin = TRIG_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOC, &GPIO_InitStruct);

    // Initialize Echo Pin (Input)
    GPIO_InitStruct.GPIO_Pin = ECHO_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOC, &GPIO_InitStruct);

    // Initialize LED and Buzzer (Outputs)
    GPIO_InitStruct.GPIO_Pin = LED_PIN | BUZZER_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOC, &GPIO_InitStruct);

    // Configure Timer 2 (TIM2)
    TIM2->PSC = 48 - 1;   // Set prescaler (1us per count at 48MHz)
    TIM2->ATRLR = 0xFFFF; // Set auto-reload (max value)
    TIM2->CTLR1 |= TIM_CEN;  // Enable TIM2 (Bit 0: CEN)
}

uint32_t getDistance() {
    // Send Trigger Pulse
    GPIO_ResetBits(GPIOC, TRIG_PIN);
    delay_us(2);
    GPIO_SetBits(GPIOC, TRIG_PIN);
    delay_us(10);
    GPIO_ResetBits(GPIOC, TRIG_PIN);

    // Wait for Echo High
    while (GPIO_ReadInputDataBit(GPIOC, ECHO_PIN) == RESET);
    TIM2->CNT = 0;  // Reset timer
    while (GPIO_ReadInputDataBit(GPIOC, ECHO_PIN) == SET);
    uint32_t time_elapsed = TIM2->CNT;  // Read elapsed time

    // Convert Time to Distance (Speed of Sound: 343 m/s or 0.0343 cm/us)
    return (time_elapsed * 0.0343) / 2; // Distance in cm
}

void Object_Detection() {
    while (1) {
        uint32_t distance = getDistance();

        if (distance < 20) {  // If object is detected within 20 cm
            GPIO_SetBits(GPIOC, LED_PIN);  // Turn on LED
            if (distance < 10) {
                GPIO_SetBits(GPIOC, BUZZER_PIN);  // Turn on Buzzer if very close
            } else {
                GPIO_ResetBits(GPIOC, BUZZER_PIN);
            }
        } else {
            GPIO_ResetBits(GPIOC, LED_PIN | BUZZER_PIN);  // Turn off both
        }

        delay_ms(500);  // Delay for stability
    }
}

int main(void) {
    SystemInit();       // Initialize system clock
    Ultrasonic_Init();  // Initialize ultrasonic sensor, LED, and buzzer
    Object_Detection(); // Start object detection loop

    while (1);
}


