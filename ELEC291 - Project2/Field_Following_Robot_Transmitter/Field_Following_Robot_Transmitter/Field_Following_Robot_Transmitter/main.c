#include <XC.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include <sys/attribs.h>
#include "lcd.h"
 
// Configuration Bits (somehow XC32 takes care of this)
#pragma config FNOSC = FRCPLL       // Internal Fast RC oscillator (8 MHz) w/ PLL
#pragma config FPLLIDIV = DIV_2     // Divide FRC before PLL (now 4 MHz)
#pragma config FPLLMUL = MUL_20     // PLL Multiply (now 80 MHz)
#pragma config FPLLODIV = DIV_2     // Divide After PLL (now 40 MHz)
#pragma config FWDTEN = OFF         // Watchdog Timer Disabled
#pragma config FPBDIV = DIV_1       // PBCLK = SYCLK
#pragma config FSOSCEN = OFF        // Turn off secondary oscillator on A4 and B4

// Defines
#define SYSCLK 40000000L
#define DEF_FREQ 4096 //audio frequency 
#define IR_FREQ 76000L
#define Baud2BRG(desired_baud)( (SYSCLK / (16*desired_baud))-1)

#define PATH_ONE   (PORTB&(1<<6))
#define PATH_TWO   (PORTB&(1<<10))
#define PATH_THREE (PORTB&(1<<12))

//VERY ARBITAUARY PIN DECLARATION
#define PATH_FOUR  (PORTB&(1<<13))

#define JOY_STICK_PRESS (PORTB&(1<<15))

//arbituary pin definitions
//#define IR_LED (PORTB&(1<<15))

//======================================== Global variables (Mainly flags) =====================================

//data values
volatile int IR_counter = 0;
volatile int pause = 2*76; //76 triggers per ms * 2
volatile unsigned char IR_Enable = 0; // Set to 1 from main code to start transmitting
volatile int joy_stick_flip = 0;
volatile int path_length = 0; //length of the path, used for progress bar
volatile int custom_path_index = 0; //used for custom path
volatile int custom_path[16];
volatile int path = 0; //path select
volatile int blink_count;
volatile int speaker_beep;

//non-volatile ones ig? If something breaks, check these first
double joy_con_x, joy_con_y; //raw values from joycon
int angle = 0, magnitude = 0;     //converted values to be sent to robot
static uint32_t tick_count = 0, second_count; //second_count actually counts 0.1 seconds

//flags
volatile int reverse_flag = 0;
volatile int blink_flag = 0;
volatile int auto_mode = 0; //flag for auto mode, 0 = completely no responce, 1 = auto, 2 = manual
volatile int options_menue = 0;
volatile int custom_path_flag = 0; //flag for custom path
volatile int display_flag = 0; //flag for display_switching/options mode
volatile char speaker_flag = 0; //used to turn speaker on/off
volatile int speaker_toggle = 0;
volatile int IR_flag = 0; //flag for IR LED

//function prototypes
void IR_call(int, int);

//=================================================== functions (UART and ADC) ===================================================

// Use the core timer to wait for 1 ms.
void wait_1ms(void)
{
    unsigned int ui;
    _CP0_SET_COUNT(0); // resets the core timer count

    // get the core timer count
    while ( _CP0_GET_COUNT() < (SYSCLK/(2*1000)) );
}

void delayms(int len)
{
	while(len--) wait_1ms();
}
 
void UART2Configure(int baud_rate)
{
    // Peripheral Pin Select
    U2RXRbits.U2RXR = 4;    //SET RX to RB8
    RPB9Rbits.RPB9R = 2;    //SET RB9 to TX

    U2MODE = 0;         // disable autobaud, TX and RX enabled only, 8N1, idle=HIGH
    U2STA = 0x1400;     // enable TX and RX
    U2BRG = Baud2BRG(baud_rate); // U2BRG = (FPb / (16*baud)) - 1
    
    U2MODESET = 0x8000;     // enable UART2
}

void ADCConf(void)
{
    AD1CON1CLR = 0x8000;    // disable ADC before configuration
    AD1CON1 = 0x00E0;       // internal counter ends sampling and starts conversion (auto-convert), manual sample
    AD1CON2 = 0;            // AD1CON2<15:13> set voltage reference to pins AVSS/AVDD
    AD1CON3 = 0x0f01;       // TAD = 4*TPB, acquisition time = 15*TAD 
    AD1CON1SET=0x8000;      // Enable ADC
}

int ADCRead(char analogPIN)
{
    AD1CHS = analogPIN << 16;    // AD1CHS<16:19> controls which analog pin goes to the ADC
 
    AD1CON1bits.SAMP = 1;        // Begin sampling
    while(AD1CON1bits.SAMP);     // wait until acquisition is done
    while(!AD1CON1bits.DONE);    // wait until conversion done
 
    return ADC1BUF0;             // result stored in ADC1BUF0
}

//===================================== Timer subroutines =======================================================

void SetupTimer1 (void)
{
	__builtin_disable_interrupts();
	PR1 =(SYSCLK/DEF_FREQ)-1; // since SYSCLK/FREQ = PS*(PR1+1)
	TMR1 = 0;
	T1CONbits.TCKPS = 0; // Pre-scaler: 1
	T1CONbits.TCS = 0; // Clock source
	T1CONbits.ON = 1;
	IPC1bits.T1IP = 5;
	IPC1bits.T1IS = 0;
	IFS0bits.T1IF = 0;
	IEC0bits.T1IE = 1;
	
	INTCONbits.MVEC = 1; //Int multi-vector
	__builtin_enable_interrupts();
}

void __ISR(_TIMER_1_VECTOR, IPL5SOFT) Timer1_Handler(void)
{
    IFS0CLR = _IFS0_T1IF_MASK;

    if (speaker_toggle) {
        LATAbits.LATA0 = !LATAbits.LATA0;
    }

    tick_count++;
    if (tick_count >= DEF_FREQ/10) {  // DEF_FREQ ticks = 1 second
        
        tick_count = 0;
        
        if (auto_mode == 1) {
        
            second_count++;
            
            if (second_count % 2 == 0 && speaker_flag) {
                
                speaker_toggle = !speaker_toggle;
                speaker_beep++;
                
                if (speaker_beep == 2*path) {
                    speaker_flag = 0;
                    speaker_beep = 0;
                    speaker_toggle = 0;
                }
                
            }
            
            if (second_count % 10 == 0 && blink_flag) {
            
                LATAbits.LATA1 = !LATAbits.LATA1;
                blink_count++;
                
                if (blink_count == 9) {
                    blink_flag = 0;
                    blink_count = 0;
                    auto_mode = 0;
                }
                
            }
        
        } if(auto_mode == 2 && second_count % 10 == 0) {
        
            if (joy_stick_flip == 1) {
            
                if (angle > 315 || angle < 45) IR_call(2,2);
                else if (angle > 45 && angle < 135) IR_call(2,5);
                else if (angle > 225 && angle < 315) IR_call(2,8);
                else if (angle > 135 && angle < 225) IR_call(2,11);
                joy_stick_flip = 0;
                
            } else if (joy_stick_flip == 0) {
            
                IR_call(3,magnitude);
                joy_stick_flip = 1;
		    }
		    
        }
        
    }
}

void SetupTimer2(void)
{
    __builtin_disable_interrupts();

    PR2 = (SYSCLK / IR_FREQ) - 1;  // Period register for 76 kHz
    TMR2 = 0;
    T2CONbits.TCKPS = 0;            // Prescaler: 1:1
    T2CONbits.TCS   = 0;            // Internal peripheral clock
    T2CONbits.ON    = 1;            // Start timer

    IPC2bits.T2IP = 6;              // Priority 6 — higher than Timer 1 (5)
    IPC2bits.T2IS = 0;              // Sub-priority 0
    IFS0bits.T2IF = 0;              // Clear any pending flag
    IEC0bits.T2IE = 1;              // Enable Timer 2 interrupt

    INTCONbits.MVEC = 1;            // Multi-vector mode (already set, but safe to repeat)
    __builtin_enable_interrupts();
}

void __ISR(_TIMER_2_VECTOR, IPL6SOFT) Timer2_Handler(void)
{
    IFS0CLR = _IFS0_T2IF_MASK;     // Clear Timer 2 interrupt flag

    if (IR_flag != 0) {
    
        LATBbits.LATB0 = !LATBbits.LATB0;  // Toggle IR LED pin NEEDS TO CHECK RIGHT PIN LATER
        IR_flag--;
        
    } else if (IR_flag == 0 && IR_counter != 0 && pause != 0) {
    
        pause--;
        LATBbits.LATB0 = 0;
        
    } else if (IR_counter != 0) {
    
        LATBbits.LATB0 = !LATBbits.LATB0;  // Toggle IR LED pin NEEDS TO CHECK RIGHT PIN LATER
        IR_counter--;
        
    } else LATBbits.LATB0 = 0;                 // Ensure LED is off when disabled
}

//============================================= IR functions ===========================================

void IR_call(int flag, int value) {

    if (IR_counter > 0 || IR_flag > 0) return;
    // Disable interrupts to "lock" the variables
    unsigned int status = __builtin_disable_interrupts(); 

    IR_flag = flag * 76;
    IR_counter = value * 76;
    pause = 2 * 76;
    
    // Restore interrupts to their previous state
    __builtin_mtc0(_CP0_STATUS, _CP0_STATUS_SELECT, status); 
    
}

// ===================================== joystick subroutines ======================================


// At startup, sample the resting position
volatile double x_center, y_center;

void calibrate_joystick(void) {
    long sx = 0, sy = 0;
    int i;
    for (i = 0; i < 32; i++) {
        sx += ADCRead(3);
        sy += ADCRead(4);
        delayms(1);
    }
    x_center = sx / 32.0;
    y_center = sy / 32.0;
}

void joy_stick(double x, double y, double x_center, double y_center, int* angle, int* magnitude) {

    double nx = (x - x_center) / x_center;
    double ny = (y - y_center) / y_center;

    // Calculate magnitude as Euclidean distance, clamped to [0, 1]
    double mag = 10.0*sqrt(nx * nx + ny * ny);
    if (mag < 5) mag = 3;
    *magnitude = (int)mag;

    // Calculate angle in degrees using atan2, result is in [-180, 180]
    // atan2 returns radians, so convert to degrees
    double raw_angle = atan2(ny, nx) * (180.0 / M_PI);

    // Shift range from [-180, 180] to [0, 360]
    if (raw_angle < 0.0)
        raw_angle += 360.0;
        
    if (mag == 3) raw_angle = 2;
    if (raw_angle == 0) raw_angle = 2;
    
    *angle  = (int)raw_angle;
    if (*angle == 0) *angle = 2;
}

// ============================================ LCD functions ===============================================

void LCD_display() {

    char buffer[17], buffer2[17];
    int y, time_per_hash = 100000, number_of_hash = 0;
    
    buffer[0] = '\0';
    buffer2[0] = '\0';
    
    if (path == 1 || path == 3) {
    
        time_per_hash = 50;
        number_of_hash = second_count/time_per_hash;
        
    } else if (path == 2) {
    
        time_per_hash = 44;
        number_of_hash = second_count/time_per_hash;
        
    } else if (path == 4) {
    
        time_per_hash = 100*custom_path_index/16;
        if (time_per_hash == 0) time_per_hash = 100000;
        
        number_of_hash = second_count/time_per_hash;
        
    }
    
    if (number_of_hash > 16) number_of_hash = 16;

    if (display_flag == 0) {
        
        sprintf(buffer2, "Path: ");
        
        if (auto_mode == 0) strcat(buffer2, "waiting");
        else if (auto_mode == 2) strcat(buffer2, "manual");
        else if (reverse_flag == 1) strcat(buffer2, "Reverse");
        else if (auto_mode == 1) {
        
            for (y = 0; y < 16; y++) {
			    if (y < number_of_hash) {
			        buffer[y] = '#';
			    } else {
			        buffer[y] = ' '; // Fill the rest with spaces to "erase" old hashes
			    }
			}
			buffer[16] = '\0'; // Cap it at 16
			LCDprint(buffer, 2, 1);
            
            sprintf(buffer, "%d", path);
            strcat(buffer2, buffer);
            
        }
        
        LCDprint(buffer2,1,1);
        
    }
}

void options_display() {

    char buffer[17], buffer2[17];
    int y;
    
    memset(buffer, ' ', 16); 
    buffer[16] = '\0';
    memset(buffer2, ' ', 16);
    buffer2[16] = '\0';
    
    if (options_menue == 1) {
        snprintf(buffer, sizeof(buffer), "Auto path:");
        snprintf(buffer2, sizeof(buffer2), "Path:%d Sp:%d R:%d", path, magnitude,reverse_flag);
    } else if (options_menue == 2) {
    
        sprintf(buffer, "Custom path:");
    
	    for (y = 0; y < 16; y++) {
	    
	        if (y < custom_path_index) {
	            if (custom_path[y] == 1) buffer2[y] = 'F';
	            else if (custom_path[y] == 2) buffer2[y] = 'L';
	            else if (custom_path[y] == 3) buffer2[y] = 'R';
	        } else {
	            buffer2[y] = ' '; // Clear the rest of the 16 chars
	        }
	    }
	    buffer2[16] = '\0';
	    
	}
	
	LCDprint(buffer,1,1);
    LCDprint(buffer2, 2, 1);
}

void LED_display() {
    
    int path_done = 0;
    
    if (path == 1 || path == 3) path_done = 800;
    else if (path == 2) path_done = 700;
    else if (path == 4) path_done = 100*custom_path_index;
    
    if (auto_mode == 1 && second_count > path_done && second_count < path_done + 10) {
    
        blink_flag = 1;
        LATBbits.LATB14 = 1;    // High
        
        auto_mode = 0;
        LCDprint("                ",2,1);
        
    } else if (auto_mode == 0) {
    
        LATAbits.LATA1 = 0;     // High
        LATBbits.LATB14 = 1;    // High
    
    } else if (auto_mode == 2) {
    
        LATAbits.LATA1 = 0;     // High
        LATBbits.LATB14 = 0;    // High
    
    } else if (auto_mode == 1 && second_count < path_done) {
    
        LATAbits.LATA1 = 1;     // High
        LATBbits.LATB14 = 0;    // High
    
    }

}

//============================================================================================================

/* Pinout for DIP28 PIC32MX130:
                                          --------
                                   MCLR -|1     28|- AVDD 
  VREF+/CVREF+/AN0/C3INC/RPA0/CTED1/RA0 -|2     27|- AVSS 
        VREF-/CVREF-/AN1/RPA1/CTED2/RA1 -|3     26|- AN9/C3INA/RPB15/SCK2/CTED6/PMCS1/RB15
   PGED1/AN2/C1IND/C2INB/C3IND/RPB0/RB0 -|4     25|- CVREFOUT/AN10/C3INB/RPB14/SCK1/CTED5/PMWR/RB14
  PGEC1/AN3/C1INC/C2INA/RPB1/CTED12/RB1 -|5     24|- AN11/RPB13/CTPLS/PMRD/RB13
   AN4/C1INB/C2IND/RPB2/SDA2/CTED13/RB2 -|6     23|- AN12/PMD0/RB12
     AN5/C1INA/C2INC/RTCC/RPB3/SCL2/RB3 -|7     22|- PGEC2/TMS/RPB11/PMD1/RB11
                                    VSS -|8     21|- PGED2/RPB10/CTED11/PMD2/RB10
                     OSC1/CLKI/RPA2/RA2 -|9     20|- VCAP
                OSC2/CLKO/RPA3/PMA0/RA3 -|10    19|- VSS
                         SOSCI/RPB4/RB4 -|11    18|- TDO/RPB9/SDA1/CTED4/PMD3/RB9
         SOSCO/RPA4/T1CK/CTED9/PMA1/RA4 -|12    17|- TCK/RPB8/SCL1/CTED10/PMD4/RB8
                                    VDD -|13    16|- TDI/RPB7/CTED3/PMD5/INT0/RB7
                    PGED3/RPB5/PMD7/RB5 -|14    15|- PGEC3/RPB6/PMD6/RB6
                                          --------
*/

//23, 21
void main(void)
{
    
    int y;
    
    DDPCON = 0;
	CFGCON = 0;
    
    //digital pins - input (WILL NEED TO CLEAN UP LATER)
    //0 -> IR, 6,10,12,13 -> buttons, 15 -> joy_stick button
    ANSELB &= ~((1<<0) | (1<<6) | (1<<10) | (1<<12) | (1<<13) | (1<<15)); // Setas digital I/O
    TRISB  |=  ((1<<0) | (1<<6) | (1<<10) | (1<<12) | (1<<13) | (1<<15)); // Configure all as inputs
    CNPUB  |=  ((1<<0) | (1<<6) | (1<<10) | (1<<12) | (1<<13) | (1<<15)); // Enable pull-ups on all

    //output pins
    TRISAbits.TRISA0 = 0;   // RA0 as output (speaker)
    LATAbits.LATA0   = 0;   // start low
    
    //LEDs
    ANSELAbits.ANSA1 = 0;  // RA1 as digital
    TRISAbits.TRISA1 = 0;  // RA1 as output
    ANSELBbits.ANSB14 = 0; // RB14 as digital
    TRISBbits.TRISB14 = 0; // RB14 as output
    LATAbits.LATA1 = 1;     // start low
    LATBbits.LATB14 = 1;    // start low
    
    ANSELBbits.ANSB0 = 0;   // RB0 as Digital //Not too sure, will have to double check
    TRISBbits.TRISB0 = 0;   // RB0 as Output

	//analog pins
    ANSELBbits.ANSB2 = 1;   // set RB2 (AN4, pin 6 of DIP28) as analog pin
    TRISBbits.TRISB2 = 1;   // set RB2 as an input
    ANSELBbits.ANSB1 = 1;   // set RB1 (AN5, pin 7 of DIP28) as analog pin
    TRISBbits.TRISB1 = 1;   // set RB1 as an input
    
    //setup function calls
    UART2Configure(115200);  // Configure UART2 for a baud rate of 115200
    LCD_4BIT();
    ADCConf(); // Configure ADC
    SetupTimer1();
    SetupTimer2();
    calibrate_joystick();
	//UART1Configure();
 
	delayms(500); // Wait a bit to give putty a chance to start
    
	while(1)
	{
	    joy_con_x = ADCRead(3);
	    joy_con_y = ADCRead(4);
	    
	    if (JOY_STICK_PRESS == 0) {
			waitms(20);
			if (JOY_STICK_PRESS == 0) {
				
				if (auto_mode != 2) {
				    auto_mode = 2; //turns on manual mode
				    path = 0; //path 0 = no path ig? idk yet
				    second_count = 0;
				  
		            LCDprint("                ",2,1);
				} else if (auto_mode == 2) {
				    auto_mode = 0;
				    angle = 1;
				    magnitude = 1;
				    IR_call(5,1);
				}
				
				while (JOY_STICK_PRESS == 0) {};
				waitms(20);
			}
			
		}
	    
    	if (PATH_ONE == 0) {
			waitms(20);
			if (PATH_ONE == 0) {
			    
			    //0 -> progress bar/path
			    //1 -> options_menue
			    //2 -> custom_path
			    
			    
			    if (auto_mode == 2) {
			    
			        IR_call(6,1);
			    
			    } else if (options_menue == 0) {
			        options_menue = 1;
			        path = 1;
			        magnitude = 1;
			        second_count = 0;
			        auto_mode = 0;
			        
			        
		            custom_path_index = 0;
		            for (y = 0; y < 16; y++) custom_path[y] = 0;
		            
			    } else if (options_menue == 1) {
			        options_menue = 2;
			        IR_call(3,magnitude);
			    } else if (options_menue == 2) {
			    
			        options_menue = 0;
			        second_count = 0;
			        auto_mode = 1;

		            LCDprint("                ",1,1);
		            LCDprint("                ",2,1);
				    
			        if (reverse_flag == 0) {
			            IR_call(1,path);
			            speaker_flag = 1;
			        } else {
			            IR_call(1,5);
			            speaker_flag = 1;
			        }
			        
			    }
                
				while (PATH_ONE == 0) waitms(20);
			}
			
		}
		
		if (PATH_TWO == 0) {
			waitms(20);
			if (PATH_TWO == 0) {
			
			    if (auto_mode == 2) {
			    
			        IR_call(6,2);
			
			    } else if (options_menue == 1) {
			    
			        if (path == 4) path = 1;
			        else path++;
			    
			    } else if (options_menue == 2 && custom_path_index <= 14) {
                    custom_path[custom_path_index] = 1;
                    custom_path_index++;
                    IR_call(4,1);
                }
			
				while (PATH_TWO == 0) waitms(20);
			}
				
		}
		
		if (PATH_THREE == 0) {
			waitms(20);
			if (PATH_THREE == 0) {
			
				if (options_menue == 1) {
			    
			        if (magnitude == 10) magnitude = 1;
			        else magnitude++;
			        
			        IR_call(3,magnitude);
			    
			    } else if (options_menue == 2 && custom_path_index <= 14) {
                    custom_path[custom_path_index] = 2;
                    custom_path_index++;
                    IR_call(4,2);
                }
				
				while (PATH_THREE == 0) waitms(20);
			}
		}
		
		if (PATH_FOUR == 0) {
			waitms(20);
			if (PATH_FOUR == 0) {
			
				if (options_menue == 1) {
			    
			        if (reverse_flag == 0) reverse_flag = 1;
			        else if (reverse_flag == 1) reverse_flag = 0;
			    
			    } else if (options_menue == 2 && custom_path_index <= 14) {
                    custom_path[custom_path_index] = 3;
                    custom_path_index++;
                    IR_call(4,3);
                }
				
				while (PATH_FOUR == 0) waitms(20);
			}
		}
		
		if (auto_mode == 2) joy_stick(joy_con_x, joy_con_y, x_center, y_center, &angle, &magnitude);
		
		//debugging printf statements
		
		printf("Angle: %d\r\n", angle);
		printf("Mag: %d\r\n", magnitude);
		
		if (options_menue == 0)
		    LCD_display();
		else options_display();
		
		LED_display();
		
		waitms(100);
	}
}
