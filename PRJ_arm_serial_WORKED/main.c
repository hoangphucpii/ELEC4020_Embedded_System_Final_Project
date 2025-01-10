/*
ELEC4020 - Embedded System Project: Human Arm Motion Recognition and Control
Team Members: Chau Hoang Phuc, Nguyen Ngoc Phuc Tien, Vu Duc Duy
Cohort: Electrical Engineering - Cohort 3

Description:
This C code runs on the TIVA TM4C123GH6PM microcontroller. It performs the following tasks:
1. Reads motion data received from the Python application via UART.
2. Decodes the data packet using a state machine.
3. Sends PWM signals to 4 servos corresponding to:
    - Elbow angle
    - Wrist angle
    - Thumb-index finger distance
    - Base angle

Key Features:
- UART configuration for receiving and decoding data.
- State machine implementation for reliable decoding.
- PWM signal generation for servo motor control.

Data Format:
The data received over UART is in the format:
    [elbow_angle, wrist_angle, thumb_index_distance, base_angle]

Instructions:
- Ensure UART baud rate and settings match with the sender's configuration.
- Calibrate the servos before operation to ensure accurate movement.
*/







#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>         // For strtof()
#include <math.h>           // For roundf
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_ints.h"    // For INT_UART0
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/debug.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "driverlib/interrupt.h"
#include "driverlib/uart.h"
#include "driverlib/rom.h"

// ============================================
//          DEFINITIONS & GLOBALS
// ============================================

// ----------------------------------------------------------------
//                      GLOBALS
// ----------------------------------------------------------------


#define PWM_FREQUENCY 50

// Adjust variables for each servo
volatile uint32_t ui32Adjust1 = 625;   // Servo 1  Grabber
volatile uint32_t ui32Adjust2 = 1625; // Servo 2   Wrist
volatile uint32_t ui32Adjust3 = 225;  // Servo 3   Elbow
volatile uint32_t ui32Adjust4 = 925;  // Servo 4   Base

volatile uint32_t ui32Load;
volatile uint32_t ui32PWMClock;

// Pulse width limits (adjust if needed)
#define SERVO1_MIN 625
#define SERVO1_MAX 1065
#define SERVO2_MIN 225
#define SERVO2_MAX 1625
#define SERVO3_MIN 225
#define SERVO3_MAX 1625
#define SERVO4_MIN 225
#define SERVO4_MAX 1625



float angle1 = 0;
float angle2 = 0;
float angle3 = 0;
float angle4 = 0;


// ============================================
//         UART Parsing State Machine
// ============================================

// Define states for parsing
typedef enum {
    WAIT_FOR_BRACKET,  // Not yet seen '['
    READING_SERVO1,    // Reading the first number until ','
    READING_SERVO2,    // Reading the second number until ','
    READING_SERVO3,    // Reading the third number until ','
    READING_SERVO4     // Reading the fourth number until ']'
} ParseState;

static volatile ParseState g_parseState = WAIT_FOR_BRACKET;

// We’ll store characters for each servo’s value here
static char servo1Str[24];
static char servo2Str[24];
static char servo3Str[24];
static char servo4Str[24];


// Indices to track how many characters we’ve stored
static int servo1Idx = 0;
static int servo2Idx = 0;
static int servo3Idx = 0;
static int servo4Idx = 0;


char c;





// ============================================
//         UART Interrupt Handler
// ============================================
void UART0IntHandler(void)
{
    uint32_t ui32Status;

    // 1) Get interrupt status and clear it
    ui32Status = UARTIntStatus(UART0_BASE, true);
    UARTIntClear(UART0_BASE, ui32Status);

    // 2) Process all available characters in FIFO
    while (UARTCharsAvail(UART0_BASE))
    {
        c = (char)UARTCharGet(UART0_BASE);

        // --- State Machine Parsing ---
        switch (g_parseState)
        {
            case WAIT_FOR_BRACKET:
                if (c == '[') {
                    servo1Idx = servo2Idx = servo3Idx = servo4Idx = 0;
                    servo1Str[0] = servo2Str[0] = servo3Str[0] = servo4Str[0] = '\0';
                    g_parseState = READING_SERVO1;
                }
                break;

            case READING_SERVO1:
                if (c == ',') {
                    servo1Str[servo1Idx] = '\0';
                    g_parseState = READING_SERVO2;
                } else if (c == '[') {
                    servo1Idx = 0;
                    g_parseState = READING_SERVO1;
                } else if (c == ']') {
                    g_parseState = WAIT_FOR_BRACKET;
                } else {
                    if (servo1Idx < (int)sizeof(servo1Str) - 1) {
                        servo1Str[servo1Idx++] = c;
                        servo1Str[servo1Idx] = '\0';
                    }
                }
                break;

            case READING_SERVO2:
                if (c == ',') {
                    servo2Str[servo2Idx] = '\0';
                    g_parseState = READING_SERVO3;
                } else if (c == '[') {
                    servo1Idx = 0;
                    g_parseState = READING_SERVO1;
                } else if (c == ']') {
                    g_parseState = WAIT_FOR_BRACKET;
                } else {
                    if (servo2Idx < (int)sizeof(servo2Str) - 1) {
                        servo2Str[servo2Idx++] = c;
                        servo2Str[servo2Idx] = '\0';
                    }
                }
                break;

            case READING_SERVO3:
                if (c == ',') {
                    servo3Str[servo3Idx] = '\0';
                    g_parseState = READING_SERVO4;
                } else if (c == '[') {
                    servo1Idx = 0;
                    g_parseState = READING_SERVO1;
                } else if (c == ']') {
                    g_parseState = WAIT_FOR_BRACKET;
                } else {
                    if (servo3Idx < (int)sizeof(servo3Str) - 1) {
                        servo3Str[servo3Idx++] = c;
                        servo3Str[servo3Idx] = '\0';
                    }
                }
                break;

            case READING_SERVO4:
                if (c == ']') {
                    servo4Str[servo4Idx] = '\0';

                    // Convert parsed strings to floats
                    angle1 = strtof(servo1Str, NULL);
                    angle2 = strtof(servo2Str, NULL);
                    angle3 = strtof(servo3Str, NULL);
                    angle4 = strtof(servo4Str, NULL);

                    // Map angles to PWM pulse widths
                    ui32Adjust1 = angle1;
                    ui32Adjust2 = angle2;
                    ui32Adjust3 = angle3;
                    ui32Adjust4 = angle4;

                    // Update servo pulse widths
                    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, ui32Adjust1);
                    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, ui32Adjust2);
                    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, ui32Adjust3);
                    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, ui32Adjust4);

                    g_parseState = WAIT_FOR_BRACKET;
                }
                else if (c == '[') {
                    servo1Idx = 0;
                    g_parseState = READING_SERVO1;
                }
                else if (c == ',') {
                    // Ignore malformed comma
                }
                else {
                    if (servo4Idx < (int)sizeof(servo4Str) - 1) {
                        servo4Str[servo4Idx++] = c;
                        servo4Str[servo4Idx] = '\0';
                    }
                }
                break;
        }
    }
}


// ----------------------------------------------------------------
//                          Main
// ----------------------------------------------------------------

int main(void)
{
    // ------------------------------
    // 1) Setup System and PWM
    // ------------------------------
    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
    SysCtlPWMClockSet(SYSCTL_PWMDIV_64);

    // Enable peripherals
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    // Wait for peripherals to be ready
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_UART0));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));

    // ------------------------------
    // 2) UART Configuration
    // ------------------------------
    GPIOPinConfigure(GPIO_PA0_U0RX);  // Configure PA0 as UART RX
    GPIOPinConfigure(GPIO_PA1_U0TX);  // Configure PA1 as UART TX
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Set UART0 to 115200 baud, 8-N-1
    UARTConfigSetExpClk(UART0_BASE,
                        SysCtlClockGet(),
                        115200,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    // ------------------------------
    // 3) Enable UART Interrupts
    // ------------------------------
    IntMasterDisable();
    IntRegister(INT_UART0, UART0IntHandler);
    UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
    IntEnable(INT_UART0);
    IntMasterEnable();
    // ------------------------------
    // 4) Configure PWM outputs
    // ------------------------------
    GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    GPIOPinConfigure(GPIO_PD0_M1PWM0);  // PD0 -> M1PWM0 -> Servo 1
    GPIOPinConfigure(GPIO_PD1_M1PWM1);  // PD1 -> M1PWM1 -> Servo 2
    GPIOPinConfigure(GPIO_PB4_M0PWM2);  // PB4 -> M0PWM2 -> Servo 3
    GPIOPinConfigure(GPIO_PB5_M0PWM3);  // PB5 -> M0PWM1 -> Servo 4


    // Calculate the PWM period
    ui32PWMClock = SysCtlClockGet() / 64;
    ui32Load = (ui32PWMClock / PWM_FREQUENCY) - 1;

    // Configure PWM generators
    PWMGenConfigure(PWM1_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN); // M1PWM0 & M1PWM1 (Servo 1 & 2)
    PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN); // M0PWM2 (Servo 3)
    PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN); // Servo 4

    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, ui32Load);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, ui32Load);
//    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ui32Load); // Servo 4

    // Initialize servos to default
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, ui32Adjust1); // Servo 1
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, ui32Adjust2); // Servo 2
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, ui32Adjust3); // Servo 3
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, ui32Adjust4);  // (PWM0_BASE, PWM_OUT_1, ui32Adjust4) if servo 4


    // Enable outputs
    PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT, true); // Servo 1 & 2
    PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, true);                // Servo 3
    PWMOutputState(PWM0_BASE, PWM_OUT_3_BIT, true);                //Servo 4

    // Enable PWM generators
    PWMGenEnable(PWM1_BASE, PWM_GEN_0); // servo 1 & 2
    PWMGenEnable(PWM0_BASE, PWM_GEN_1); // servo 3
//    PWMGenEnable(PWM0_BASE, PWM_GEN_0); // if servo 4
    // ------------------------------
    // 5) Main Loop
    // ------------------------------
    while (1) {
        // The UART0IntHandler() runs automatically when new data arrives.
        // We parse the brackets and commas in the interrupt.
        // The main loop can do other tasks or just idle.
    }
}
