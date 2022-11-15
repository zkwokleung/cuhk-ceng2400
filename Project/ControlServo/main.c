// Control a servo with two buttons
// author: Xiangyu Wen
// Date: 2022.11.01

// hardware connection:
// servo (lower) orange wire -> PD0
// servo (lower) red wire -> V Bus
// servo (lower) brown wire -> GND

// servo (upper) orange wire -> PD1 (recommended)
// servo (upper) red wire -> V Bus
// servo (upper) brown wire -> GND

// What you need to do:
// 1. ignore the button-driven rotation function
// 2. try to control the servos with your computer
//    (you are recommended to use UART to communicate with PC and get values to control the servo.)
// 3. think about how to control two servos with the PWM (try to implement it).

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

#define SERVO_MIN_VALUE 26
#define SERVO_MAX_VALUE 141
#define SERVO_CENTER_VALUE 83

//ui8Adjust is used to contol the servo angle.
//We initialize ui8Adjust to 83 to make sure the servo is at the center position.
volatile uint32_t ui32Load;
volatile uint32_t ui32PWMClock;
volatile uint32_t ui8Adjust;

// used to store the UART input
char uartReceive[100];
int uartReceiveCount = 0;

void SetServoValue(int value)
{
    if(value > SERVO_MAX_VALUE || value < SERVO_MIN_VALUE)
    {
        return;
    }
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, value * ui32Load/1000);
}

void ProcessUARTForServo()
{
    int value = atoi(uartReceive);
    SetServoValue(value);
}

int main()
{
    ui8Adjust = 83;

    SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);
    SysCtlPWMClockSet(SYSCTL_PWMDIV_64);

    // Enable PWM1 to generate PWM signals
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
    // Enable GPIOD to output signals to servo
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    // Enable GPIOF to use buttons
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    // Enable UART0 and GPIOA to send signals via UART
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);


    GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_0);
    GPIOPinConfigure(GPIO_PD0_M1PWM0);

    // Configure and enable UART
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    IntMasterEnable();
    IntEnable(INT_UART0);
    UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);

    // set GPIOs for buttons
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE + GPIO_O_CR)  |= 0x01;
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;
    GPIODirModeSet(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0, GPIO_DIR_MODE_IN);
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);


    ui32PWMClock = SysCtlClockGet()/64;
    ui32Load = (ui32PWMClock / PWM_FREQUENCY) - 1;
    PWMGenConfigure(PWM1_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN);
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, ui32Load);

    // set the servo's initial position
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, ui8Adjust * ui32Load/1000);
    PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT, true);
    PWMGenEnable(PWM1_BASE, PWM_GEN_0);


    while(1)
    {
        // Check whether the button is pressed
        if(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4)==0x00)
        {
            ui8Adjust--; // this variable is used to control the servo angle
            if(ui8Adjust < 26) // set the working zone from -90 to 90
            {
                ui8Adjust = 26;
            }
            PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, ui8Adjust * ui32Load/1000);
        }

        // Check whether the button is pressed
        if(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_0)==0x00)
        {
            ui8Adjust++;
            if(ui8Adjust > 141) // set the working zone
            {
                ui8Adjust = 141;
            }
            PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, ui8Adjust * ui32Load/1000);
        }

        // since the main controlling function is implemented in while loop
        // we need to use delay function to control the rotating speed of the servo.
        SysCtlDelay(100000);
    }

}

void UARTIntHandler(void)
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
            ProcessUARTForServo();
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
