#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/pwm.h"
#include "utils/uartstdio.h"
#include "inc/hw_ints.h"
#include "driverlib/interrupt.h"

#define PWM_FREQUENCY 55

#define SERVO_MIN_YAW 26
#define SERVO_MAX_YAW 141
#define SERVO_CENTER_YAW 83
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

    // Configure PA ports and enable UART0
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    // Enable UART5 and GPIOE to send signals via BlueTooth
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART5);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    // Configure PE ports and enable UART5
    GPIOPinConfigure(GPIO_PE4_U5RX);
    GPIOPinConfigure(GPIO_PE5_U5TX);
    GPIOPinTypeUART(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5);
    UARTConfigSetExpClk(UART5_BASE, SysCtlClockGet(), 38400,
        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    // Enable Interrupt
    IntMasterEnable();
    IntEnable(INT_UART0);
    UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
    IntEnable(INT_UART5);
    UARTIntEnable(UART5_BASE, UART_INT_RX | UART_INT_RT);
}

void Initialize(void)
{
    // set clock
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    InitializeUART();
    InitializePWM();
}

int main(void) {
    Initialize();

    UARTCharPut(UART0_BASE, 'W');
    UARTCharPut(UART0_BASE, 'a');
    UARTCharPut(UART0_BASE, 'i');
    UARTCharPut(UART0_BASE, 't');
    UARTCharPut(UART0_BASE, 'i');
    UARTCharPut(UART0_BASE, 'n');
    UARTCharPut(UART0_BASE, 'g');
    UARTCharPut(UART0_BASE, '.');
    UARTCharPut(UART0_BASE, '.');
    UARTCharPut(UART0_BASE, '.');
    UARTCharPut(UART0_BASE, '\n');

    while (1)
    {
        if (UARTCharsAvail(UART0_BASE)) UARTCharPut(UART5_BASE, UARTCharGet(UART0_BASE));
    }

}

void UART0IntHandler(void)
{
    uint32_t ui32Status;

    ui32Status = UARTIntStatus(UART5_BASE, true); //get interrupt status

    UARTIntClear(UART0_BASE, ui32Status); //clear the asserted interrupts

    while(UARTCharsAvail(UART0_BASE)) //loop while there are chars
    {
        UARTCharPut(UART5_BASE, UARTCharGet(UART0_BASE)); //echo character
        SysCtlDelay(SysCtlClockGet() / (1000 * 3)); //delay some time
    }
}

//check whether there are any items in the FIFO of UART5.
//get characters from UART5 that communicates with bluetooth.
//send received characters to UART0 that communicates with PC.
void UART5IntHandler(void)
{
    uint32_t ui32Status;

    ui32Status = UARTIntStatus(UART5_BASE, true); //get interrupt status

    UARTIntClear(UART5_BASE, ui32Status); //clear the asserted interrupts

    while(UARTCharsAvail(UART5_BASE)) //loop while there are chars
    {
        UARTCharPut(UART0_BASE, UARTCharGet(UART5_BASE)); //echo character
        SysCtlDelay(SysCtlClockGet() / (1000 * 3)); //delay some time
    }
}

