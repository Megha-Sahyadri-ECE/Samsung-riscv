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
src="https://github.com/Megha-Sahyadri-ECE/Samsung-riscv/blob/main/Task1/program%20sum.JPG"/>

## riscv based lab output(01)
<img
src="https://github.com/Megha-Sahyadri-ECE/Samsung-riscv/blob/main/Task1/sum1ton..o%20file.JPG"/>

## riscv based lab output(0fast)
<img
src="https://github.com/Megha-Sahyadri-ECE/Samsung-riscv/blob/main/Task1/calculation_main.JPG"/>

## riscv based lab output
<img
src="https://github.com/Megha-Sahyadri-ECE/Samsung-riscv/blob/main/Task1/main%20file.JPG"/>

</details>
<details>
<summary>TASK2:Simulation with Spike</summary>
  
## program to find whether the  given number is even or odd
<img
src="https://github.com/Megha-Sahyadri-ECE/Samsung-riscv/blob/main/Task2/c%20program_evenodd.PNG" />

## output of even odd program
<img
src="https://github.com/Megha-Sahyadri-ECE/Samsung-riscv/blob/main/Task2/leafpad%20evenodd%20compile%20process.PNG" />

## debugging of O1
<img 
src="https://github.com/Megha-Sahyadri-ECE/Samsung-riscv/blob/main/Task2/spike-d%20_evenodd_debug.PNG"/>

## debugging of Ofast
  <img
src="https://github.com/Megha-Sahyadri-ECE/Samsung-riscv/blob/main/Task2/debbugging%20process_Ofast.PNG"/>

## objdump of O1
<img
src="https://github.com/Megha-Sahyadri-ECE/Samsung-riscv/blob/main/Task2/debugging%20_O1_.PNG"/>

## objdump of Ofast
<img
src="https://github.com/Megha-Sahyadri-ECE/Samsung-riscv/blob/main/Task2/objdump_Ofast.PNG"/>


</details>

<details>
<summary>TASK3:Identification of RISCV instructions</summary>
  <img
src="https://github.com/Megha-Sahyadri-ECE/Samsung-riscv/blob/main/Task3/ofast_objdump.PNG"/>
<summary>RISC-V Assembly Instructions Breakdown</summary>
  
## **1. `lui a0, 0x2b`**  
**Opcode (U-Type):** `0110111`  
**Registers:** `rd = a0 (00101)`  
**Immediate:** `0x2b (0000000000101011)`  

| imm[31:12] | rd (`a0`) | opcode  |
|------------|---------|---------|
| 0000000000101011 | 00101 | 0110111 |

---
## **2. `addi sp, sp, -32`**  
**Opcode (I-Type):** `0010011`  
**Registers:** `rs1 = sp (00010)`, `rd = sp (00010)`  
**Immediate:** `-32 (111111110000)`  

| imm[11:0] | rs1 (`sp`) | funct3 | rd (`sp`) | opcode  |
|-----------|-----------|--------|---------|---------|
| 111111110000 | 00010 | 000 | 00010 | 0010011 |

---
## **3. `sd ra, 24(sp)`**  
**Opcode (S-Type):** `0100111`  
**Registers:** `rs1 = sp (00010)`, `rs2 = ra (00001)`  
**Immediate:** `24 (split as imm[11:5] = 0000001, imm[4:0] = 11000)`  

| imm[11:5] | rs2 (`ra`) | rs1 (`sp`) | funct3 | imm[4:0] | opcode  |
|-----------|-----------|-----------|--------|---------|---------|
| 0000001 | 00001 | 00010 | 011 | 11000 | 0100111 |

---

## **4. `jal ra, 1048c <printf>`**  
**Opcode (J-Type):** `1101111`  
**Registers:** `rd = ra (00001)`  
**Immediate:** `1048c (split as imm[20] = 0, imm[19:12] = 00101000, imm[11] = 1, imm[10:1] = 0100011000)`  

| imm[20] | imm[10:1] | imm[11] | imm[19:12] | rd (`ra`) | opcode  |
|---------|-----------|--------|-----------|---------|---------|
| 0 | 0100011000 | 1 | 00101000 | 00001 | 1101111 |

---
## **5. `lw a1, 12(sp)`**  
**Opcode (I-Type):** `0000011`  
**Registers:** `rs1 = sp (00010)`, `rd = a1 (00111)`  
**Immediate:** `12 (000000001100)`  

| imm[11:0] | rs1 (`sp`) | funct3 | rd (`a1`) | opcode  |
|-----------|-----------|--------|---------|---------|
| 000000001100 | 00010 | 010 | 00111 | 0000011 |

---
## **6. `ld ra, 24(sp)`**  
**Opcode (I-Type):** `0000011`  
**Registers:** `rs1 = sp (00010)`, `rd = ra (00001)`  
**Immediate:** `24 (000000011000)`  

| imm[11:0] | rs1 (`sp`) | funct3 | rd (`ra`) | opcode  |
|-----------|-----------|--------|---------|---------|
| 000000011000 | 00010 | 011 | 00001 | 0000011 |

---
## **7. `li a0, 0`**  
*(Equivalent to `addi a0, zero, 0`)*  
**Opcode (I-Type):** `0010011`  
**Registers:** `rs1 = zero (00000)`, `rd = a0 (00101)`  
**Immediate:** `0 (000000000000)`  

| imm[11:0] | rs1 (`zero`) | funct3 | rd (`a0`) | opcode  |
|-----------|-----------|--------|---------|---------|
| 000000000000 | 00000 | 000 | 00101 | 0010011 |

---
## **8. `auipc a5, 0x2b`**  
**Opcode (U-Type):** `0010111`  
**Registers:** `rd = a5 (01111)`  
**Immediate:** `0x2b (0000000000101011)`  

| imm[31:12] | rd (`a5`) | opcode  |
|------------|---------|---------|
| 0000000000101011 | 01111 | 0010111 |

---
## **9. `j 100ec <main+0x3c>`**  
**Opcode (J-Type):** `1101111`  
**Registers:** `rd = zero (00000)`  
**Immediate:** `100ec (split as imm[20] = 0, imm[19:12] = 00100000, imm[11] = 1, imm[10:1] = 0110011000)`  

| imm[20] | imm[10:1] | imm[11] | imm[19:12] | rd (`zero`) | opcode  |
|---------|-----------|--------|-----------|---------|---------|
| 0 | 0110011000 | 1 | 00100000 | 00000 | 1101111 |

---
## **10. `bnez a5, 100fc <main+0x4c>`**  
**Opcode (B-Type):** `1100011`  
**Registers:** `rs1 = a5 (01111)`, `rs2 = zero (00000)`  
**Immediate:** `100fc (split as imm[12] = 1, imm[10:5] = 000011, imm[4:1] = 1100, imm[11] = 1)`  

| imm[12] | imm[10:5] | rs2 (`zero`) | rs1 (`a5`) | funct3 | imm[4:1] | imm[11] | opcode  |
|---------|-----------|------------|-----------|--------|---------|--------|---------|
| 1 | 000011 | 00000 | 01111 | 001 | 1100 | 1 | 1100011 |

---
## **11. `lbu a5, 1944(gp)`**  
**Opcode (I-Type):** `0000011`  
**Registers:** `rs1 = gp (00100)`, `rd = a5 (01111)`  
**Immediate:** `1944 (000001111001000)`  

| imm[11:0] | rs1 (`gp`) | funct3 | rd (`a5`) | opcode  |
|-----------|-----------|--------|---------|---------|
| 000001111001000 | 00100 | 100 | 01111 | 0000011 |

---
## **12. `sub a2, a2, a0`**  
**Opcode (R-Type):** `0110011`  
**Registers:** `rs1 = a2 (00110)`, `rs2 = a0 (00101)`, `rd = a2 (00110)`  

| funct7 | rs2 (`a0`) | rs1 (`a2`) | funct3 | rd (`a2`) | opcode  |
|--------|-----------|-----------|--------|---------|---------|
| 0100000 | 00101 | 00110 | 000 | 00110 | 0110011 |

---

## **13. `mv a1, a0`**  
*(Equivalent to `addi a1, a0, 0`)*  
**Opcode (I-Type):** `0010011`  
**Registers:** `rs1 = a0 (00101)`, `rd = a1 (00111)`  
**Immediate:** `0 (000000000000)`  

| imm[11:0] | rs1 (`a0`) | funct3 | rd (`a1`) | opcode  |
|-----------|-----------|--------|---------|---------|
| 000000000000 | 00101 | 000 | 00111 | 0010011 |

---

## **14. `jr zero`**  
*(Equivalent to `jalr zero, zero, 0`)*  
**Opcode (I-Type):** `1100111`  
**Registers:** `rs1 = zero (00000)`, `rd = zero (00000)`  
**Immediate:** `0 (000000000000)`  

| imm[11:0] | rs1 (`zero`) | funct3 | rd (`zero`) | opcode  |
|-----------|-----------|--------|---------|---------|
| 000000000000 | 00000 | 000 | 00000 | 1100111 |

---
## **15. `beqz a5, 101dc <frame_dummy+0x20>`**  
*(Equivalent to `beq a5, zero, 101dc`)*  
**Opcode (B-Type):** `1100011`  
**Registers:** `rs1 = a5 (01111)`, `rs2 = zero (00000)`  
**Immediate:** `101dc (split as imm[12] = 1, imm[10:5] = 011101, imm[4:1] = 1100, imm[11] = 1)`  

| imm[12] | imm[10:5] | rs2 (`zero`) | rs1 (`a5`) | funct3 | imm[4:1] | imm[11] | opcode  |
|---------|-----------|------------|-----------|--------|---------|--------|---------|
| 1 | 011101 | 00000 | 01111 | 000 | 1100 | 1 | 1100011 |

</details>
<details>
<summary>TASK4:Functional Simulation of RISC-V Core</summary>
</summary>
<br>
Steps to perform functional simulation of RISCV

1. Download Files:
Download the code from the reference github repo.

2. Set Up Simulation Environment:
Install iverlog using commands:

        sudo apt install iverilog
        sudo apt install gtkwave

3. To run and simulate the verilog code, enter the following command:

        iverilog -o iiitb_rv32i iiitb_rv32i.v iiitb_rv32i_tb.v
        ./iiitb_rv32i

4. To see the simulation waveform in GTKWave, enter the following command:

        gtkwave iiitb_rv32i.vcd
5.apt_get gtkwave installation

<img
src="https://github.com/Megha-Sahyadri-ECE/Samsung-riscv/blob/main/Task4/apt_get%20gtkwave%20install.PNG"/>

6.iiitb_rv32i_code
 
module iiitb_rv32i_tb;

reg clk,RN;
wire [31:0]WB_OUT,NPC;

iiitb_rv32i rv32(clk,RN,NPC,WB_OUT);


always #3 clk=!clk;

initial begin 
RN  = 1'b1;
clk = 1'b1;

$dumpfile ("iiitb_rv32i.vcd"); //by default vcd
$dumpvars (0, iiitb_rv32i_tb);
  
  #5 RN = 1'b0;
  
  #300 $finish;

end
endmodule

Analysing the Output Waveform of the instructions.

1. ADD

<img
src="https://github.com/Megha-Sahyadri-ECE/Samsung-riscv/blob/main/Task4/add_.PNG"/>

  
2. SUB 

<img
src="https://github.com/Megha-Sahyadri-ECE/Samsung-riscv/blob/main/Task4/_sub_.png"/>


3. SLT
   
<img
src="https://github.com/Megha-Sahyadri-ECE/Samsung-riscv/blob/main/Task4/slt.png"/>

4. ADDI
   
<img
src="https://github.com/Megha-Sahyadri-ECE/Samsung-riscv/blob/main/Task4/addi.png"/>

5. BEQ
   
<img
src="https://github.com/Megha-Sahyadri-ECE/Samsung-riscv/blob/main/Task4/_beq_.png"/>
</details>

<details>
<summary>TASK5:Project overview-circuit diagram</summary>
This project is an Ultrasonic-Based Object Detection and Alert System using the CH32V003 microcontroller and the HC-SR04 ultrasonic sensor. It measures the distance of nearby objects using ultrasonic waves and triggers an alert mechanism based on the detected distance. If an object is within a certain range, the system activates a buzzer to provide an audible warning. The HC-SR04 sensor is powered by 5V, while its Echo signal is safely converted to 3.3V using a voltage divider to ensure compatibility with the CH32V003 MCU. This system is useful for proximity sensing, obstacle detection, and safety applications.

1.Pinout Diagram of Ultrasonic Obstacle Detection with Buzzer Alarm
<img 
src="https://github.com/Megha-Sahyadri-ECE/Samsung-riscv/blob/main/Task5/_CIRCUIT-DIAGRAM.PNG"/>
<img

2.Blinking Led Test code simulation.

https://github.com/Sudheeksha-Sahyadri-ECE/samsung-riscv/raw/refs/heads/main/task%205/blinking_led_test.mp4




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


