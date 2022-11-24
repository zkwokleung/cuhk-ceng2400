#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_ints.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/debug.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/interrupt.h"
#include "driverlib/uart.h"

#define PWM_FREQUENCY 55

#define SERVO_MIN_YAW 20
#define SERVO_MAX_YAW 140
#define SERVO_CENTER_YAW 90
#define SERVO_MIN_PITCH 50
#define SERVO_MAX_PITCH 120
#define SERVO_CENTER_PITCH 60

// Store the pwm clock
volatile uint32_t ui32Load;
volatile uint32_t ui32PWMClock;

// Store the value of the servo
volatile uint32_t ui32ServoYawValue;
volatile uint32_t ui32ServoPitchValue;

// Store the UART input
char uartReceive[100];
int uartReceiveCount = 0;

// Set the left/right rotation of the servo
void SetServoYaw(int value)
{
    if(value > SERVO_MAX_YAW || value < SERVO_MIN_YAW)
    {
        return;
    }
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, value * ui32Load/1000);
}

// Set the up/down rotation of the servo
void SetServoPitch(int value)
{
    if(value > SERVO_MAX_PITCH || value < SERVO_MIN_PITCH)
    {
        return;
    }
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, value * ui32Load/1000);
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
    ui32PWMClock = SysCtlClockGet()/64;
    ui32Load = (ui32PWMClock / PWM_FREQUENCY) - 1;
    PWMGenConfigure(PWM1_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN);
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, ui32Load);

    // Enable PWM
    PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT, true);
    PWMGenEnable(PWM1_BASE, PWM_GEN_0);
}

void InitializeUART()
{
    // Enable UART0 and GPIOA to send signals via UART
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    // Configure and enable UART
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    IntMasterEnable();
    IntEnable(INT_UART0);
    UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
}

void InitializeLED()
{
    // Enable PF2 for led response
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);
}

void Initialize()
{
    SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);
    InitializePWM();
    InitializeUART();
    InitializeLED();

    // set the servo's initial position
    SetServoPitch(SERVO_CENTER_YAW);
    SetServoYaw(SERVO_CENTER_PITCH);
}

int main()
{
    Initialize();

    while(1)
    {
    }
}

void UARTInt0Handler(void)
{
    uint32_t ui32Status;

    ui32Status = UARTIntStatus(UART0_BASE, true); // get interrupt status

    UARTIntClear(UART0_BASE, ui32Status); // clear the asserted interrupts

    while (UARTCharsAvail(UART0_BASE)) // loop while there are chars
    {
        char c = UARTCharGet(UART0_BASE);

        // If it is an enter key, process the data entered
        if (c == 10 || c == 13)
        {
            UARTCharPut(UART0_BASE, '\n');
            UARTCharPut(UART0_BASE, '\r');
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
            UARTCharPut(UART0_BASE, c);                            // echo character
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2); // blink LED
            SysCtlDelay(SysCtlClockGet() / (1000 * 3));            // delay ~1 msec
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);          // turn off LED
            uartReceive[uartReceiveCount++] = c;
        }
    }
}
