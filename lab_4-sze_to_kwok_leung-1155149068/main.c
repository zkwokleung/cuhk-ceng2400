#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/tm4c123gh6pm.h"
#include <stdbool.h>
#include <stdint.h>

void GPIO_PORtF_Handler(void);
void Timer0IntHandler(void);
void delayMs(int n);
void delayUs(int n);

uint32_t ui32Period_0;
uint32_t ui32Period_1;
uint8_t color = 2;

int main(void)
{
    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);

    // Set the first timer
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    ui32Period_0 = (SysCtlClockGet()) / 16;
    TimerLoadSet(TIMER0_BASE, TIMER_A, ui32Period_0 - 1);

    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    // Set the second timer
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
    TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
    ui32Period_1 = SysCtlClockGet() * 2;
    TimerLoadSet(TIMER1_BASE, TIMER_A, ui32Period_1 - 1);

    IntEnable(INT_TIMER1A);
    TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

    // Enable the timers
    IntMasterEnable();
    TimerEnable(TIMER0_BASE, TIMER_A);
    TimerEnable(TIMER1_BASE, TIMER_A);

    // Configure switch interrupt
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_4); // PF4 input
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    GPIOIntEnable(GPIO_PORTF_BASE, GPIO_INT_PIN_4);                 // interrupt enable
    GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_FALLING_EDGE); // only interrupt at falling edge (pressed)
    GPIOIntRegister(GPIO_PORTF_BASE, GPIO_PORtF_Handler);           // dynamic isr registering

    while (1)
    {
    }
}

// Handle the switch input
void GPIO_PORtF_Handler(void)
{
    GPIOIntClear(GPIO_PORTF_BASE, GPIO_INT_PIN_4);
    static int hz = 8;

    hz /= 2;
    hz = (hz < 2) ? 8 : hz;
    ui32Period_0 = SysCtlClockGet() / (hz * 2);
    TimerLoadSet(TIMER0_BASE, TIMER_A, ui32Period_0 - 1);
}

// Handle the blinking
void Timer0IntHandler(void)
{
    // Clear the timer interrupt
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    // Read the current state of the GPIO pin and
    if (GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3))
    {
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0);
    }
    else
    {
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, color);
    }
}

// Handle the color change
void Timer1IntHandler(void)
{
    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

    color = color * 2;
    color = (color > 8) ? 2 : color;
}
