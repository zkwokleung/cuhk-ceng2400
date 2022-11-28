#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "inc/hw_gpio.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "utils/uartstdio.h"
/*
 * Motor functions
 */
#define PWM_FREQUENCY 55
#define SERVO_MIN_PITCH 20
#define SERVO_INIT_PITCH 110
#define SERVO_MAX_PITCH 110
#define SERVO_MIN_YAW 20
#define SERVO_INIT_YAW 160
#define SERVO_MAX_YAW 160

// Store the pwm clock
volatile uint32_t ui32Load;
volatile uint32_t ui32PWMClock;

// Store the value of the servo
volatile uint32_t ui32ServoYawValue;
volatile uint32_t ui32ServoPitchValue;

// Store the UART input
char uartReceive[100];
int uartReceiveCount = 0;

// Bonus
bool doingMove = false;

/*
 * Generic Utilities
 */
void delayMS(int ms)
{
    SysCtlDelay((SysCtlClockGet() / (3 * 1000)) * ms); // less accurate
}

// Set the left/right rotation of the servo
void SetServoYaw(int value)
{
    if (value > SERVO_MAX_YAW || value < SERVO_MIN_YAW)
    {
        return;
    }
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, value * ui32Load / 1000);
}

// Set the up/down rotation of the servo
void SetServoPitch(int value)
{
    if (value > SERVO_MAX_PITCH || value < SERVO_MIN_PITCH)
    {
        return;
    }
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, value * ui32Load / 1000);
}

void InitializePWM()
{
    SysCtlPWMClockSet(SYSCTL_PWMDIV_64);

    // Enable GPIOD to output signals to servo
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    // Enable PWM0 and PWM1 to generate PWM signals
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0 | SYSCTL_PERIPH_PWM1);

    // Set PD0 and PD1 as PWM pin
    GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    GPIOPinConfigure(GPIO_PD0_M1PWM0);
    GPIOPinConfigure(GPIO_PD1_M1PWM1);

    // Get the frequency of the pwm control clock
    ui32PWMClock = SysCtlClockGet() / 64;
    ui32Load = (ui32PWMClock / PWM_FREQUENCY) - 1;
    PWMGenConfigure(PWM1_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN);
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, ui32Load);

    // Enable PWM
    PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT, true);
    PWMGenEnable(PWM1_BASE, PWM_GEN_0);
}

/*
 * UART Functions to handle the communication via UART.
 * While UART0 is transferring data to PC,
 * UART5 is transferring data to HC5, the BlueTooth device
 */
void UARTStringPut(uint32_t ui32Base, char *str)
{
    int i;
    for (i = 0; str[i] != '\0'; i++)
    {
        UARTCharPut(ui32Base, str[i]);
    }
}

void UARTIntPut(uint32_t ui32Base, int value)
{
    char temp[10];
    char result[10];
    int tempCount = 0;
    int resultCount = 0;

    if (value == 0)
    {
        result[0] = '0';
        result[1] = '\0';
        UARTCharPut(ui32Base, '0');
    }

    if (value < 0)
    {
        result[resultCount++] = '-';
        value *= -1;
    }

    // Covert to char from LSB
    while (value > 0)
    {
        temp[tempCount++] = (value % 10) + '0';
        value /= 10;
    }

    // Put back the result from MSB
    while (--tempCount >= 0)
    {
        result[resultCount++] = temp[tempCount];
    }
    result[resultCount] = '\0';

    UARTStringPut(ui32Base, result);
}

void InitializeUART(void)
{
    // enable UART0 and GPIOA.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    // Configure PA0 for RX
    // Configure PA1 for TX
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    // Set PORTA pin0 and pin1 as UART type
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // set UART base addr., clock get and baud rate.
    // used to communicate with computer
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    // enable UART5 and GPIOE
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART5);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    // Configure board PE4 for RX
    // configure board PE5 for TX
    GPIOPinConfigure(GPIO_PE4_U5RX);
    GPIOPinConfigure(GPIO_PE5_U5TX);
    // set PORTE pin4 and pin5 as UART type
    GPIOPinTypeUART(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    // set UART base addr., system clock, baud rate
    // used to communicate with HC-05
    UARTConfigSetExpClk(UART5_BASE, SysCtlClockGet(), 38400,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2 | GPIO_PIN_1 | GPIO_PIN_3, 2);

    // set interrupt for receiving and showing values
    IntMasterEnable();
    IntEnable(INT_UART0);
    UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
    IntEnable(INT_UART5);
    UARTIntEnable(UART5_BASE, UART_INT_RX | UART_INT_RT);
}

void InitializeButton(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE + GPIO_O_CR)  |= 0x01;
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;
    GPIODirModeSet(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0, GPIO_DIR_MODE_IN);
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
}

void Initialize(void)
{
    // set clock
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    InitializeButton();

    // Initialize UART
    InitializeUART();

    InitializePWM();

    // set the servo's initial position
    SetServoPitch(SERVO_INIT_PITCH);
    SetServoYaw(SERVO_INIT_YAW);
}

int main(void)
{
    Initialize();

    while (1)
    {
        // Check whether the button is pressed
        if(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4)==0x00)
        {
            doingMove = true;
            // Nodding
            SetServoPitch(20);
            delayMS(300);
            SetServoPitch(80);
            delayMS(300);
            SetServoPitch(20);
            delayMS(300);
            SetServoPitch(80);
            delayMS(300);
            SetServoPitch(20);
            doingMove = false;
        }

        // Check whether the button is pressed
        if(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_0)==0x00)
        {
            doingMove = true;
            // Shaking
            SetServoYaw(60);
            delayMS(300);
            SetServoYaw(120);
            delayMS(300);
            SetServoYaw(60);
            delayMS(300);
            SetServoYaw(120);
            delayMS(300);
            SetServoYaw(120);
            doingMove = false;
        }
    }
}

void UART0IntHandler(void)
{
    uint32_t ui32Status;

    ui32Status = UARTIntStatus(UART5_BASE, true); // get interrupt status

    UARTIntClear(UART0_BASE, ui32Status); // clear the asserted interrupts
}

// check whether there are any items in the FIFO of UART5.
// get characters from UART5 that communicates with bluetooth.
// send received characters to UART0 that communicates with PC.
void UART5IntHandler(void)
{
    if(doingMove)
        return;

    uint32_t ui32Status;

    ui32Status = UARTIntStatus(UART5_BASE, true); // get interrupt status

    UARTIntClear(UART5_BASE, ui32Status); // clear the asserted interrupts

    while (UARTCharsAvail(UART5_BASE)) // loop while there are chars
    {
        char c = UARTCharGet(UART5_BASE);

        // If it is an enter key, process the data entered
        if (c == 10 || c == 13)
        {
            uartReceive[uartReceiveCount] = '\0';
            uartReceiveCount = 0;

            // Process the received value and send it to the servo
            if(uartReceive[0] == 'p' || uartReceive[0] == 'P')
            {
                ui32ServoPitchValue = atoi(uartReceive + 1);
                // Set pitch value
                SetServoPitch(ui32ServoPitchValue);
            }
            else if(uartReceive[0] == 'y' || uartReceive[0] == 'Y')
            {
                ui32ServoYawValue = atoi(uartReceive + 1);
                // Set yaw value
                SetServoYaw(ui32ServoYawValue);
            }
        }
        else
        {
            // Store the character
            uartReceive[uartReceiveCount++] = c;
        }
    }

    delayMS(50);       // delay some time
}
