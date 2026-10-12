#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include "../Common/Include/stm32l051xx.h"
#include "../Common/Include/serial.h"
#include "vl53l0x.h"
#include "adc.h"

// LQFP32 pinout
//              ----------
//        VDD -|1       32|- VSS
//       PC14 -|2       31|- BOOT0
//       PC15 -|3       30|- PB7 (I2C1_SDA)
//       NRST -|4       29|- PB6 (I2C1_SCL)
//       VDDA -|5       28|- PB5
//        PA0 -|6       27|- PB4
//        PA1 -|7       26|- PB3
//        PA2 -|8       25|- PA15
//        PA3 -|9       24|- PA14
//        PA4 -|10      23|- PA13
//        PA5 -|11      22|- PA12
//        PA6 -|12      21|- PA11
//        PA7 -|13      20|- PA10 (Reserved for RXD)
//        PB0 -|14      19|- PA9  (Reserved for TXD)
//        PB1 -|15      18|- PA8
//        VSS -|16      17|- VDD
//              ----------

#define F_CPU 32000000L
#define NUNCHUK_ADDRESS 0x52

#define Center_Coil   (1<<0)    // PB0  - physical pin 14
#define Left_Coil    (1<<7)    // PA7  - physical pin 13
#define Right_Coil     (1<<1)    // PB1  - physical pin 15

//Don't ask. This is intended
#define Left_Motor_Forward  (1<<5)  // PA5 (Physical Pin 11)
#define Left_Motor_Reverse  (1<<1)  // PA1 (Physical Pin 7)
#define Right_Motor_Forward (1<<11)  // PA11 (Physical Pin 21)
#define Right_Motor_Reverse (1<<6)  // PA6 (Physical Pin 12)
#define Claw_Motor          (1<<12) // PA12 (Physical Pin 22)

#define IR_LED (1<<4) //BUS A
#define IR_Sensor (1<<4) //BUS B

#define IR_Sensor_Detects  (!(GPIOB->IDR & IR_Sensor))

#define Claw_Motor_ON      (GPIOA->ODR |=  Claw_Motor)
#define Claw_Motor_OFF     (GPIOA->ODR &= ~Claw_Motor)
#define Claw_Motor_TOGGLE  (GPIOA->ODR ^=  Claw_Motor)

#define Left_Motor_Forward_ON    (GPIOA->ODR |= Left_Motor_Forward)
#define Left_Motor_Forward_OFF   (GPIOA->ODR &=  ~Left_Motor_Forward)
#define Left_Motor_Forward_TOGGLE (GPIOA->ODR ^=  Left_Motor_Forward)

#define Left_Motor_Reverse_ON    (GPIOA->ODR |= Left_Motor_Reverse)
#define Left_Motor_Reverse_OFF   (GPIOA->ODR &=  ~Left_Motor_Reverse)
#define Left_Motor_Reverse_TOGGLE (GPIOA->ODR ^=  Left_Motor_Reverse)

#define Right_Motor_Forward_ON    (GPIOA->ODR |= Right_Motor_Forward)
#define Right_Motor_Forward_OFF   (GPIOA->ODR &=  ~Right_Motor_Forward)
#define Right_Motor_Forward_TOGGLE (GPIOA->ODR ^=  Right_Motor_Forward)

#define Right_Motor_Reverse_ON    (GPIOA->ODR |= Right_Motor_Reverse)
#define Right_Motor_Reverse_OFF   (GPIOA->ODR &=  ~Right_Motor_Reverse)
#define Right_Motor_Reverse_TOGGLE (GPIOA->ODR ^=  Right_Motor_Reverse)

volatile uint8_t ir_data = 0;
volatile uint32_t  IR_flag = 0; //0 -> flag_not_set, 1 -> path select, 2 -> joystick angle, 3 -> joystick magnitude
volatile uint32_t  IR_counter = 0;
volatile int turn_counter = 0, straight_counter;
volatile int angle = 0, magnitude = 0, path_choice = 0;
volatile int intersection_count = 0;
volatile int custom_path[17];
volatile int custom_count = 0;
volatile int path_memory = 1;
volatile int reverse_intersection_count = 0;
volatile int PWM_DUTY_CYCLE = 5;

volatile int intersection_flag = 0;
volatile bool data_ready = 0;
volatile bool turn_flag = 0;
volatile bool stop_flag = 0;
volatile bool pause_flag = 0;
volatile bool claw_flag = 0;
volatile int claw_timer = 0;
unsigned char success;
unsigned short range=0;

volatile int left_motor_forward_pwm = 0;
volatile int left_motor_reverse_pwm = 0;
volatile int right_motor_forward_pwm = 0;
volatile int right_motor_reverse_pwm = 0;
volatile int claw_motor_pwm = 0;

volatile int left_motor_forward_pwm_count = 0;
volatile int left_motor_reverse_pwm_count = 0;
volatile int right_motor_forward_pwm_count = 0;
volatile int right_motor_reverse_pwm_count = 0;
volatile int claw_motor_pwm_count = 0; 
volatile int open_claw_count = 0;
volatile int close_claw_count = 0;
volatile bool claw_mode = 0;
volatile int claw_amount = 0;
volatile bool servo_on = 0;
volatile uint8_t incoming_task = 0;
volatile uint8_t task = 0;
volatile uint8_t prev_task = 0;

//======================== function prototypes =======================================================
void forward(void);
void IR_decode(int num);
void stop(void);

void PWM_Motor(volatile int pwm, volatile int *pwm_count, int motor_chosen, int pwm_period);
void Toggle_On_Chosen_Motor(int motor_chosen);
void Toggle_Off_Chosen_Motor(int motor_chosen);
void Claw_Control (volatile bool *claw_mode, volatile int claw_amount, volatile bool *servo_on);
uint8_t USART2_Receive(void);
void turn_left(int degree);
//===================== Wait MS functions ==============================================================

void wait_1ms(void)
{
	// For SysTick info check the STM32l0xxx Cortex-M0 programming manual.
	SysTick->LOAD = (F_CPU/1000L) - 1;  // set reload register, counter rolls over from zero, hence -1
	SysTick->VAL = 0; // load the SysTick counter
	SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk; // Enable SysTick IRQ and SysTick Timer */
	while((SysTick->CTRL & BIT16)==0); // Bit 16 is the COUNTFLAG.  True when counter rolls over from zero.
	SysTick->CTRL = 0x00; // Disable Systick counter
}

void waitms(int len)
{
	while(len--) wait_1ms();
}

// ===================== I2C functions ===============================================================

void I2C_init (void)
{
	RCC->IOPENR  |= BIT1;// peripheral clock enable for port B (I2C pins are in port B)
	RCC->APB1ENR  |= BIT21; // peripheral clock enable for I2C1 (page 177 of RM0451 Reference manual)
	
	//Configure PB6 for I2C1_SCL, pin 29 in LQFP32 package (page 44 of datasheet en.DM00108219.pdf)
	GPIOB->MODER = (GPIOB->MODER & ~(BIT12|BIT13)) | BIT13; // PB6 AF-Mode (page 200 of RM0451 Reference manual)
	GPIOB->AFR[0] |= BIT24; // AF1 selected (page 204 of RM0451 Reference manual)
	
	//Configure PB7 for I2C1_SDA, pin 30 in LQFP32 package (page 44 of datasheet en.DM00108219.pdf)
	GPIOB->MODER = (GPIOB->MODER & ~(BIT14|BIT15)) | BIT15; // PB7 AF-Mode (page 200 of RM0451 Reference manual)
	GPIOB->AFR[0] |= BIT28; // AF1 selected (page 204 of RM0451 Reference manual)
	
	GPIOB->OTYPER   |= BIT6 | BIT7; // I2C pins (PB6 and PB7)  must be open drain
	GPIOB->OSPEEDR  |= BIT12 | BIT14; // Medium speed for PB6 and PB7

	// This configures the I2C clock (Check page 564 of RM0451).  SCLK must be 100kHz or less.
	I2C1->TIMINGR = (uint32_t)0x70420f13;
}

unsigned char i2c_read_addr8_data8(unsigned char address, unsigned char * value)
{
	
	// First we send the address we want to read from:
	I2C1->CR1 = I2C_CR1_PE;
	I2C1->CR2 = I2C_CR2_AUTOEND | (1 << 16) | (0x52);
	I2C1->CR2 |= I2C_CR2_START; // Go
	while ((I2C1->ISR & I2C_ISR_TXE) != I2C_ISR_TXE) {} // Wait for address to go

	I2C1->TXDR = address; // send data
	while ((I2C1->ISR & I2C_ISR_TXE) != I2C_ISR_TXE) {} // Check Tx empty

	waitms(1);
	
	// Second: we gatter the data sent by the slave device
	I2C1->CR1 = I2C_CR1_PE | I2C_CR1_RXIE;
	I2C1->CR2 = I2C_CR2_AUTOEND | (1 << 16) | I2C_CR2_RD_WRN | (0x52); // Read one byte from slave 0x52
	I2C1->CR2 |= I2C_CR2_START; // Go
    
	while ((I2C1->ISR & I2C_ISR_RXNE) != I2C_ISR_RXNE) {} // Wait for data to arrive
	*value=I2C1->RXDR; // Reading 'receive' register clears RXNE flag

	waitms(1);

	return 1;
}

unsigned char i2c_read_addr8_data16(unsigned char address, unsigned short * value)
{
	// First we send the address we want to read from:
	I2C1->CR1 = I2C_CR1_PE;
	I2C1->CR2 = I2C_CR2_AUTOEND | (1 << 16) | (0x52);
	I2C1->CR2 |= I2C_CR2_START; // Go
	while ((I2C1->ISR & I2C_ISR_TXE) != I2C_ISR_TXE) {} // Wait for address to go

	I2C1->TXDR = address; // send data
	while ((I2C1->ISR & I2C_ISR_TXE) != I2C_ISR_TXE) {} // Check Tx empty

	waitms(1);
	
	// Second: we gatter the data sent by the slave device
	I2C1->CR1 = I2C_CR1_PE | I2C_CR1_RXIE;
	I2C1->CR2 = I2C_CR2_AUTOEND | (2<<16) | I2C_CR2_RD_WRN | (0x52); // Read two bytes from slave 0x52
	I2C1->CR2 |= I2C_CR2_START; // Go
    
	while ((I2C1->ISR & I2C_ISR_RXNE) != I2C_ISR_RXNE) {} // Wait for data to arrive
	*value=I2C1->RXDR*256; // Reading 'receive' register clears RXNE flag
   
	while ((I2C1->ISR & I2C_ISR_RXNE) != I2C_ISR_RXNE) {} // Wait for data to arrive
	*value+=I2C1->RXDR; // Reading 'receive' register clears RXNE flag
    
	waitms(1);
	
	return 1;
}

unsigned char i2c_write_addr8_data8(unsigned char address, unsigned char value)
{
	I2C1->CR1 = I2C_CR1_PE;
	I2C1->CR2 = I2C_CR2_AUTOEND | (2 << 16) | (0x52);
	I2C1->CR2 |= I2C_CR2_START;
	while ((I2C1->ISR & I2C_ISR_TXE) != I2C_ISR_TXE) {} // Wait for address to go

	I2C1->TXDR = address; // send register address
	while ((I2C1->ISR & I2C_ISR_TXE) != I2C_ISR_TXE) {} // Check Tx empty

	I2C1->TXDR = value; // send data
	while ((I2C1->ISR & I2C_ISR_TXE) != I2C_ISR_TXE) {} // Check Tx empty

	waitms(1);

	return 1;
}

// From the VL53L0X datasheet:
//
// The registers shown in the table below can be used to validate the user I2C interface.
// Address (After fresh reset, without API loaded)
//    0xC0 0xEE
//    0xC1 0xAA
//    0xC2 0x10
//    0x51 0x0099 (0x0096 after initialization)
//    0x61 0x0000
//
// Not needed, but it was useful to debug the I2C interface, so left here.
void validate_I2C_interface (void)
{
	unsigned char val8 = 0;
	unsigned short val16 = 0;
	
    printf("\n");   
    
    i2c_read_addr8_data8(0xc0, &val8);
    printf("Reg(0xc0): 0x%02x\r\n", val8);

    i2c_read_addr8_data8(0xc1, &val8);
    printf("Reg(0xc1): 0x%02x\r\n", val8);

    i2c_read_addr8_data8(0xc2, &val8);
    printf("Reg(0xc2): 0x%02x\r\n", val8);
    
    i2c_read_addr8_data16(0x51, &val16);
    printf("Reg(0x51): 0x%04x\r\n", val16);

    i2c_read_addr8_data16(0x61, &val16);
    printf("Reg(0x61): 0x%04x\r\n", val16);
    
    printf("\r\n");
}

// ===================== Initialization functions ===============================================================

void USART2_Init(void) {
    // 1. Enable Clock for GPIOA and USART2
    RCC->IOPENR   |= RCC_IOPENR_GPIOAEN;
    RCC->APB1ENR  |= RCC_APB1ENR_USART2EN;

    // 2. Configure PA2 (TX) and PA3 (RX) as Alternate Function (AF4)
    // Clear bits for PA2 and PA3
    GPIOA->MODER &= ~(GPIO_MODER_MODE2 | GPIO_MODER_MODE3);
    // Set to Alternate Function Mode (10)
    GPIOA->MODER |= (GPIO_MODER_MODE2_1 | GPIO_MODER_MODE3_1);
    
    // Set AFR for PA2 and PA3 to AF4 (0100)
    GPIOA->AFR[0] &= ~((0xF << 8) | (0xF << 12)); // Clear AFR2 and AFR3
    GPIOA->AFR[0] |=  ((0x4 << 8) | (0x4 << 12)); // Set AF4 for both

    // 3. Configure Baud Rate (assuming 32MHz clock)
    USART2->BRR = 32000000L / 9600L;

    // 4. Enable Transmitter, Receiver, and USART
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

void Configure_Motor_Pins(void)
{
    // 1. Enable Clock for Port A and Port B
    RCC->IOPENR |= (BIT0 | BIT1);

    // --- PORT A: PA0, PA1, PA5, PA6, PA12 as outputs ---
    GPIOA->MODER &= ~(GPIO_MODER_MODE11 | GPIO_MODER_MODE1 | GPIO_MODER_MODE5 |
                      GPIO_MODER_MODE6 | GPIO_MODER_MODE12);
    GPIOA->MODER |=  (GPIO_MODER_MODE11_0 | GPIO_MODER_MODE1_0 | GPIO_MODER_MODE5_0 |
                      GPIO_MODER_MODE6_0 | GPIO_MODER_MODE12_0);

    // Medium speed for PA motor pins
    GPIOA->OSPEEDR |= (BIT22 | BIT2 | BIT10 | BIT12 | BIT24);
    // Start all PA motor pins LOW
    GPIOA->ODR &= ~(BIT11 | BIT1 | BIT5 | BIT6 | BIT12);

    // --- PORT B: PB4 as digital INPUT (IR sensor) ---
    GPIOB->MODER &= ~(GPIO_MODER_MODE4);   // 00 = input mode
    // Enable internal pull-up (adjust to PUPD4_1 for pull-down if sensor needs it)
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD4);
    GPIOB->PUPDR |=  (GPIO_PUPDR_PUPD4_0); // 01 = pull-up

    GPIOA->MODER &= ~(GPIO_MODER_MODE4);   // 00 = input mode
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD4);
    GPIOA->PUPDR |=  (GPIO_PUPDR_PUPD4_0); // 01 = pull-up
}

void Configure_ADC_Pins (void)
{
    // 1. Enable clocks for Port A and Port B (if not already done)
    RCC->IOPENR |= (RCC_IOPENR_GPIOAEN | RCC_IOPENR_GPIOBEN);

    /* Existing code for your other sensors */
    // PA7 (pin 13) - Left_Coil - analog mode
    GPIOA->MODER |= (GPIO_MODER_MODE7_0 | GPIO_MODER_MODE7_1);

    // PB0 (pin 14) - Right_Coil - analog mode
    GPIOB->MODER |= (GPIO_MODER_MODE0_0 | GPIO_MODER_MODE0_1);

    // PB1 (pin 15) - Center_Coil - analog mode
    GPIOB->MODER |= (GPIO_MODER_MODE1_0 | GPIO_MODER_MODE1_1);

	// PA0 (pin 6) - battery - analog mode
	GPIOA->MODER |= (GPIO_MODER_MODE0_0 | GPIO_MODER_MODE0_1);
}

// ========================================= Turning functions =========================================

void TIM2_Init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    
    TIM2->PSC = 31;        // 1 MHz tick
    TIM2->ARR = 999;  // one interrupt per millisecond

    TIM2->SR &= ~TIM_SR_UIF; 
    TIM2->DIER |= TIM_DIER_UIE;
    NVIC_SetPriority(TIM2_IRQn, 0);
    NVIC_EnableIRQ(TIM2_IRQn);
    TIM2->CR1 |= TIM_CR1_CEN;
}

void TIM2_Handler(void) {
    if (TIM2->SR & TIM_SR_UIF) {
        TIM2->SR &= ~TIM_SR_UIF;
        
        if (turn_flag == 1 && turn_counter == 0) {

            turn_flag = 0;
            
        } else if (turn_flag == 1) turn_counter--;

    }

    PWM_Motor(left_motor_forward_pwm, &left_motor_forward_pwm_count, 0, 10);
    PWM_Motor(left_motor_reverse_pwm, &left_motor_reverse_pwm_count, 1, 10);
    PWM_Motor(right_motor_forward_pwm, &right_motor_forward_pwm_count, 2, 10);
    PWM_Motor(right_motor_reverse_pwm, &right_motor_reverse_pwm_count, 3, 10);
    PWM_Motor(claw_motor_pwm, &claw_motor_pwm_count, 4, 20);
    Claw_Control(&claw_mode, claw_amount, &servo_on);
    
    /*if (claw_timer > 0) {
        claw_timer--;
    } else {
        //claw_motor_pwm = 0;
    }*/
    incoming_task = USART2_Receive();

    if(incoming_task == 'C' || prev_task == 'C') {
        prev_task = 'C';
        if(incoming_task == 'R') {
            IR_flag = 4;
            IR_decode(3);
        }
        else if(incoming_task == 'L') {
            IR_flag = 4;
            IR_decode(2);
        }
        else if(incoming_task == 'F') {
            IR_flag = 4;
            IR_decode(1);
        }
        else if(incoming_task == 'S') {
            IR_flag = 4;
            IR_decode(4);
            prev_task = 0;
            incoming_task = 0;
        }
    }

    if(incoming_task == '1') {
        IR_flag = 1;
        IR_decode(1);
        task = incoming_task;
    }
    else if(incoming_task == '2') {
       IR_flag = 1;
       IR_decode(2);
    }
    else if(incoming_task == '3') {
       IR_flag = 1;
       IR_decode(3);
    } else if (incoming_task == '4') {
         IR_flag = 1;
         IR_decode(4);
    } else if (incoming_task == '5') {
         IR_flag = 1;
         IR_decode(5);
    }

    /*
    printf ("%d\r\n", intersection_count);
    printf("Left: %ld\r\n", readADC(ADC_CHSELR_CHSEL7));
    printf("Right: %ld\r\n", readADC(ADC_CHSELR_CHSEL9));
    printf("Intersection: %ld\r\n", readADC(ADC_CHSELR_CHSEL8));
    printf("IR_counter: %ld\nIR_flag: %ld\npath_choice: %ld\nmagnitude: %ld\nAngle: %ld\n\r", IR_counter, IR_flag, path_choice, magnitude, angle);
	*/
}

void PWM_Motor (volatile int pwm, volatile int *pwm_count, int motor_chosen, int pwm_period) {
    if (pwm != 0) {
        if((*pwm_count) < pwm) {
            Toggle_On_Chosen_Motor(motor_chosen);
            (*pwm_count)++;
        }
        else if((*pwm_count) >= pwm && (*pwm_count) < pwm_period) {
            Toggle_Off_Chosen_Motor(motor_chosen);
            (*pwm_count)++;     
        }
    }
    if(pwm == 0) {
        Toggle_Off_Chosen_Motor(motor_chosen);
        (*pwm_count) = 0;
    }
    if((*pwm_count) >= pwm_period) {
        (*pwm_count) = 0;
    }
}

void Toggle_On_Chosen_Motor (int motor_chosen) {
    if(motor_chosen == 0) {
        Left_Motor_Forward_ON;
    }
    else if(motor_chosen == 1) {
        Left_Motor_Reverse_ON;
    }
    else if(motor_chosen == 2) {
        Right_Motor_Forward_ON;
    }
    else if(motor_chosen == 3) {
        Right_Motor_Reverse_ON;
    }
    else if(motor_chosen == 4) {
        Claw_Motor_ON;
    }
}

void Toggle_Off_Chosen_Motor (int motor_chosen) {
    if(motor_chosen == 0) {
        Left_Motor_Forward_OFF;
    }
    else if(motor_chosen == 1) {
        Left_Motor_Reverse_OFF;
    }
    else if(motor_chosen == 2) {
        Right_Motor_Forward_OFF;
    }
    else if(motor_chosen == 3) {
        Right_Motor_Reverse_OFF;
    }
    else if(motor_chosen == 4) {
        Claw_Motor_OFF;
    }
}
// claw_mode = 0 open the claw and claw_mode = 1 close the claw
void Claw_Control (volatile bool *claw_mode, volatile int claw_amount, volatile bool *servo_on) {
    if(!(*servo_on)){
        close_claw_count = 0;
        open_claw_count = 0;
        claw_motor_pwm = 0;
        (*servo_on) = 0;
        return;
    }
    else if (!(*claw_mode)) {
        if(open_claw_count < claw_amount) {
            claw_motor_pwm = 2;
            open_claw_count++;
        }
        else {
            claw_motor_pwm = 0;
            open_claw_count = 0;
            (*servo_on) = 0;
        }
    }
    else if (*claw_mode) {
        if(close_claw_count < claw_amount) {
            claw_motor_pwm = 1;
            close_claw_count++;
        }
        else {
            claw_motor_pwm = 0;
            close_claw_count = 0;
            (*servo_on) = 0;
        }
    }

}


// ========================================= IR Receiver functions ==================================

void TIM21_Init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_TIM21EN;
 
    TIM21->PSC = 31;
    TIM21->ARR = 99;

    TIM21->SR &= ~TIM_SR_UIF; 
    TIM21->DIER |= TIM_DIER_UIE;
    
    NVIC_SetPriority(TIM21_IRQn, 0);
    NVIC_EnableIRQ(TIM21_IRQn);
    
    TIM21->CR1 |= TIM_CR1_CEN;
}

volatile int idle_counter = 0;

void TIM21_Handler(void) {
    if (TIM21->SR & TIM_SR_UIF) {
        TIM21->SR &= ~TIM_SR_UIF;

        // Reading PA4 (Pin 10) for Communication
        if (!(GPIOA->IDR & GPIO_IDR_ID4)) { 
            IR_counter++; 
            idle_counter = 0; // Reset silence timeout
        } 
        else {
            if (IR_counter > 0) {
                int decoded_val = (IR_counter + 5) / 10;
                IR_counter = 0; 
                
                // Filter microscopic noise spikes (ignore num = 0)
                if (decoded_val > 0) {
                    IR_decode(decoded_val);
                }
            }
            
            // Prevent Desync if PIC32 signal drops
            idle_counter++;
            if (idle_counter > 40) { 
                IR_flag = 0; 
            }
        }
    }
}

void IR_decode(int num) { //If 0.1 ms is too precise we can adjust it to a range instead

    if (IR_flag == 0) {
    
        IR_flag = num; //1 ms = path_flag = 1, 2 ms = angle_flag, 3ms = magnitude_flag, and so on
        if (IR_flag > 6) IR_flag = 0;
    
    } else if (IR_flag == 1) {
        
        if (num == 5) {

            path_memory = path_choice;
            if (path_memory == 0) path_memory = 1;  
            path_choice = 5; //1 ms -> path_choice = 1, so on

            if (path_memory == 1) reverse_intersection_count = 6;
            else if (path_memory == 2) reverse_intersection_count = 5;
            else if (path_memory == 3) reverse_intersection_count = 6;
            else if (path_memory == 4) reverse_intersection_count = custom_count - 2;

            turn_left(3250);
         
        } else {
        
            path_choice = num; //1 ms -> path_choice = 1, so on
            intersection_count = 0;
            
        }
        
        if (magnitude == 0) PWM_DUTY_CYCLE = 5;
        else PWM_DUTY_CYCLE = magnitude;
        angle = 0;
        
        stop_flag = 0;
        IR_flag = 0;
        if (path_choice == 4)custom_path[custom_count] = 4;
        claw_flag = 1;
        
    } else if (IR_flag == 2) {
    
        IR_flag = 0;
        path_choice = 0;

        if (num <= 3) angle = 0;
        else if (num <= 6) angle = 90;
        else if (num <= 9) angle = 270;
        else if (num <= 12) angle = 180;
        else angle = 0;
        
    } else if (IR_flag == 3) {
        
        magnitude = num;
        if (magnitude < 5) magnitude = 0;
        if (magnitude > 7) magnitude = 8;
        
        PWM_DUTY_CYCLE = magnitude;
        
        IR_flag = 0;
        path_choice = 0;
        
    } else if (IR_flag == 4) {
    
        IR_flag = 0;
        custom_path[custom_count] = num;
        custom_count++;
    
    } else if (IR_flag == 5) {
        
        magnitude = 0;
        angle = 0;
        IR_flag = 0;
    
    } else if (IR_flag == 6) {
    
        if (num == 1) {
        
            claw_amount = 70;
            claw_mode = 1;
            servo_on = 1;
        
        } else if (num == 2) {
            
            claw_amount = 70;
            claw_mode = 0;
            servo_on = 1;
        
        }
        
        IR_flag = 0;
        
    } else IR_flag = 0;
    
    IR_counter = 0;

}

// ================================== Motor functions ========================================
void forward(void) {
	/*Left_Motor_Reverse_OFF;
	Right_Motor_Reverse_OFF;
	Left_Motor_Forward_ON;
	Right_Motor_Forward_ON;*/

    left_motor_reverse_pwm = 0;
    right_motor_reverse_pwm = 0;
    left_motor_forward_pwm = PWM_DUTY_CYCLE;
    right_motor_forward_pwm = PWM_DUTY_CYCLE;

	straight_counter++;
	
}

void reverse(void) {
	/*Left_Motor_Forward_OFF;
	Right_Motor_Forward_OFF;
	Left_Motor_Reverse_ON;
	Right_Motor_Reverse_ON;*/

    left_motor_reverse_pwm = PWM_DUTY_CYCLE;
    right_motor_reverse_pwm = PWM_DUTY_CYCLE;
    left_motor_forward_pwm = 0;
    right_motor_forward_pwm = 0;
}

void stop(void) {
	/*Left_Motor_Forward_OFF;
	Right_Motor_Forward_OFF;
	Left_Motor_Reverse_OFF;
	Right_Motor_Reverse_OFF;*/

    left_motor_reverse_pwm = 0;
    right_motor_reverse_pwm = 0;
    left_motor_forward_pwm = 0;
    right_motor_forward_pwm = 0;

    if (stop_flag == 0) {
        
        claw_amount = 70;
        claw_mode = 0;
        servo_on = 1;
        
    }

	straight_counter = 0;
	stop_flag = 1;
    
}

void pause(void) {
	/*Left_Motor_Forward_OFF;
	Right_Motor_Forward_OFF;
	Left_Motor_Reverse_OFF;
	Right_Motor_Reverse_OFF;*/

    left_motor_reverse_pwm = 0;
    right_motor_reverse_pwm = 0;
    left_motor_forward_pwm = 0;
    right_motor_forward_pwm = 0;

	straight_counter = 0;
}

//I added a simple degree parameter using timer 2 to determine how long to turn for before going forward again.
//TODO: Find angular velocity cause *10 is just rudimentry rn
void turn_right(int degree) {
	/*Right_Motor_Forward_OFF;
	Left_Motor_Reverse_OFF;
	Right_Motor_Reverse_ON;
	Left_Motor_Forward_ON;*/

    left_motor_reverse_pwm = 0;
    right_motor_reverse_pwm = PWM_DUTY_CYCLE;
    left_motor_forward_pwm = PWM_DUTY_CYCLE;
    right_motor_forward_pwm = 0;
	
	turn_counter = degree;
	turn_flag = 1;
	
	straight_counter = 0;

}

void turn_left(int degree) {
	/*Left_Motor_Reverse_ON;
	Right_Motor_Reverse_OFF;
	Left_Motor_Forward_OFF;
	Right_Motor_Forward_ON;*/

    left_motor_reverse_pwm = PWM_DUTY_CYCLE;
    right_motor_reverse_pwm = 0;
    left_motor_forward_pwm = 0;
    right_motor_forward_pwm = PWM_DUTY_CYCLE;
	
	turn_counter = degree;
	turn_flag = 1;
	straight_counter = 0;
}

void line_following(void) {

		int line_follow = readADC(ADC_CHSELR_CHSEL9) - readADC(ADC_CHSELR_CHSEL7);
		
        //if (readADC(ADC_CHSELR_CHSEL8) > 0 && turn_counter == 0) forward();
        
		if (abs(line_follow) > 400 && line_follow > 0 && turn_counter == 0) turn_right(1);
		else if (abs(line_follow) > 400 && line_follow < 0 && turn_counter == 0) turn_left(1);
		
		else if (turn_counter == 0) forward();
}

void intersection_found(int direction){
    
	if (direction == 1) forward();
	else if (direction == 2) turn_left(1300);
	else if (direction == 3) turn_right(1300);
	else if (direction == 4) stop();
	
}

void reverse_intersection_found(int direction){
    
	if (direction == 1) forward();
	else if (direction == 2) turn_right(1300);
	else if (direction == 3) turn_left(1300);
	else if (direction == 4) stop();
	
}

void joystick_control(void) {

    if ((angle < 45 || angle > 310) && magnitude != 0) forward();
    else if (angle > 130 && angle < 210 && magnitude != 0) reverse();
    else if (turn_counter == 0 && magnitude != 0) {
        if (angle < 190) turn_right(1);
        else turn_left(1);
    }
    else if (magnitude == 0) pause();

}

uint8_t USART2_Receive(void) {
    if (USART2->ISR & USART_ISR_RXNE) {
        return (uint8_t)(USART2->RDR);
    }
    return 0;
}

//====================== Main program - Initalization =========================

void main(void)
{

	//int path1[] = {1,2,2,1,3,2,3,4}; // 1: forward, 2: left, 3: right, 4: stop
    int path1[] = {1,2,2,1,3,2,3,4};
	int path2[] = {2,3,2,3,1,1,4}; // 1: forward, 2: left, 3: right, 4: stop
	int path3[] = {3,1,3,2,3,2,1,4}; // 1: forward, 2: left, 3: right, 4: stop
    int print_timer = 0;
    
    int test = 1;

    USART2_Init();
	initADC();
	TIM2_Init();
	I2C_init();
    validate_I2C_interface();
	success = vl53l0x_init();
	Configure_ADC_Pins();
	Configure_Motor_Pins();
	
	NVIC_SetPriority(TIM2_IRQn, 0);  // Highest priority for PWM
    NVIC_SetPriority(TIM21_IRQn, 1); // Lower priority for IR
	
	//need delay between I2C init and TIM21 (IR) init
	waitms(500);
	
	TIM21_Init();
    printf("Testing1");
    
    /* //Claw debugging code, single movement
    claw_amount = 70;
    claw_mode = 1;
    servo_on = 1;
    */
	        
//========================================= Main Loop ======================================

    while (1)
    {  

        //bluetooth printfs
        
       /*printf ("%d\r\n", intersection_count);
        printf("Left: %ld\r\n", readADC(ADC_CHSELR_CHSEL7));
        printf("Right: %ld\r\n", readADC(ADC_CHSELR_CHSEL9));
        printf("Intersection: %ld\r\n", readADC(ADC_CHSELR_CHSEL8));
        printf("IR_counter: %ld\nIR_flag: %ld\npath_choice: %ld\nmagnitude: %ld\nAngle: %ld\n\r", IR_counter, IR_flag, path_choice, magnitude, angle);
        printf("Turn_Counter %d\r\n", turn_counter);
        printf("Intersection_Flag %d\r\n", intersection_flag);*/

        if (print_timer++ >= 10) {
            printf("Left:%ld\n", readADC(ADC_CHSELR_CHSEL7));
            printf("Right:%ld\n", readADC(ADC_CHSELR_CHSEL9));
            printf("Intersection:%ld\n", readADC(ADC_CHSELR_CHSEL8));
            printf("IR_counter:%ld\n", IR_counter);
            printf("path_choice:%ld\n", path_choice);
            printf("magnitude:%ld\n", magnitude);
            printf("Angle:%ld\n", angle);
            printf("Turn_Counter:%d\n", turn_counter);
            printf("Intersection_Flag:%d\n", intersection_flag);
            printf("Index:%d\n", intersection_count);
            printf("Battery: %d\r\n", readADC(ADC_CHSELR_CHSEL0));
            printf("Reverse_Intersection_Count: %d\r\n", reverse_intersection_count);
            print_timer = 0;
            if (task != 0) {
                printf("Task:%c\r\n", (char)task);
            }
        }
        

        //debuggging printfs
        
        //printf ("%d\r\n", intersection_count);
        
        //printf("Left: %ld\r\n", readADC(ADC_CHSELR_CHSEL7));
        //printf("Right: %ld\r\n", readADC(ADC_CHSELR_CHSEL9));
        //printf("Intersection: %ld\r\n", readADC(ADC_CHSELR_CHSEL8));

        
        //printf("IR_counter: %ld\r\n", IR_counter);   
        //printf("IR_flag: %ld\r\n", IR_flag);
        //printf("path_choice: %ld\r\n", path_choice);
        //printf("magnitude: %ld\r\n", magnitude);
        //printf("Angle: %ld\r\n", angle);
        /*if (task != '0') {
            printf("Task:%c\r\n", (char)task);
            //magnitude = 30;
        }*/
        
        /*
        printf("Obstacle: %ld\r\n", range);
        printf("Intersection_delay: %ld\r\n", intersection_flag);
        
	    printf("Custom Path Array: [");
        
	    for (int i = 0; i < 17; i++) {
	        // Access the volatile array and print the value
	        printf("%d", custom_path[i]);
	        
	        // Add a comma and space between numbers, but not after the last one
	        if (i < 16) {
	            printf(", ");
	        }
	    }
	    */
	    
	    //printf("\r\n"); // \r\n is essential for PuTTY to start a new line correctly
        //printf("\x1b[2J\x1b[H"); // ANSI: Clear from cursor to end of line.
        
        
        //debugging permananat movement commands
        //forward();
        //reverse();
        /*if (test == 1) {
            turn_right(90);
            test = 0;
        }*/
        //turn_left(90);
        //line_following();

		//Obstacle detection
		if (IR_counter == 0 && IR_flag == 0) {
		    TIM21->CR1 &= ~TIM_CR1_CEN;
			success = vl53l0x_read_range_single(&range);
			IR_counter = 0; // Clear any partial counts caused by I2C noise
			TIM21->CR1 |= TIM_CR1_CEN;
		}
        if (success && range < 100) {
            pause_flag = 1;
            success = 0;
        } else pause_flag = 0;

        //claw IR_sensor found
        if (IR_Sensor_Detects && claw_flag == 1 && path_choice != 0) {
            claw_amount = 80;
            claw_mode = 1;
            servo_on = 1;
            claw_flag = 0;
        }

		//Intersection found 
		if (path_choice != 0) {

            // STATE 1: Are we currently executing a timed turn?
            if (turn_counter > 0) {
                // LOCKOUT: Do absolutely nothing! 
                // Let the PWM keep spinning the motors until turn_counter hits 0.
            }

            // STATE 2: Are we clearing the intersection? (Driving forward after a turn)
            else if (intersection_flag > 0) {
            
                forward();
                intersection_flag--;

            }

            // STATE 3: Normal Driving & Intersection Detection
            else {
                // Do we see a NEW intersection?
                if (readADC(ADC_CHSELR_CHSEL8) > 900) {
                    
                    if (path_choice == 5) {   

                        if (reverse_intersection_count == 0) stop();
                        else if (path_memory == 1) reverse_intersection_found(path1[reverse_intersection_count]);
                        else if (path_memory == 2) reverse_intersection_found(path2[reverse_intersection_count]);
                        else if (path_memory == 3) reverse_intersection_found(path3[reverse_intersection_count]);
                        else if (path_memory == 4) reverse_intersection_found(custom_path[reverse_intersection_count]);
                        reverse_intersection_count--;

                    } else {
                        if (path_choice == 1) intersection_found(path1[intersection_count]);
                        else if (path_choice == 2) intersection_found(path2[intersection_count]);
                        else if (path_choice == 3) intersection_found(path3[intersection_count]);
                        else if (path_choice == 4) intersection_found(custom_path[intersection_count]);
                        
                        intersection_count++;
                    }

                    // Set the flag to drive forward for a few loops AFTER the turn finishes
                    // 15 loops * 20ms = 300ms of forward driving to clear the tape
                    intersection_flag = 5; 

                } else {
                    // Normal Line Following
                    if (stop_flag != 1 && pause_flag != 1) line_following();
                    else if (pause_flag == 1) pause();
                    else stop();
                }
            }

        } else {
            joystick_control();
        }
        
		waitms(20);
	}
}