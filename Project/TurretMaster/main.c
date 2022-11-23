#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "inc/hw_ints.h"
#include "driverlib/interrupt.h"

/*
 * Generic Utilities
 */
void delayMS(int ms)
{
    SysCtlDelay((SysCtlClockGet() / (3 * 1000)) * ms); // less accurate
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
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 38400,
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

void Initialize(void)
{
    // set clock
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    // Initialize UART
    InitializeUART();

    // Wait for two seconds before initializing slave configuration
    delayMS(2000);

    // Initialize the system as slave
    InitializeSlave();
}

int main(void)
{
    Initialize();

    while (1)
    {
    }
}

void UART0IntHandler(void)
{
    uint32_t ui32Status;

    ui32Status = UARTIntStatus(UART5_BASE, true); // get interrupt status

    UARTIntClear(UART0_BASE, ui32Status); // clear the asserted interrupts

    while (UARTCharsAvail(UART0_BASE)) // loop while there are chars
    {
        UARTCharPut(UART5_BASE, UARTCharGet(UART0_BASE)); // echo character
        SysCtlDelay(SysCtlClockGet() / (1000 * 3));       // delay some time
    }
}

// check whether there are any items in the FIFO of UART5.
// get characters from UART5 that communicates with bluetooth.
// send received characters to UART0 that communicates with PC.
void UART5IntHandler(void)
{
    uint32_t ui32Status;

    ui32Status = UARTIntStatus(UART5_BASE, true); // get interrupt status

    UARTIntClear(UART5_BASE, ui32Status); // clear the asserted interrupts

    while (UARTCharsAvail(UART5_BASE)) // loop while there are chars
    {
        // TODO: Handle the received data
        UARTCharPut(UART0_BASE, UARTCharGet(UART5_BASE)); // echo character
        SysCtlDelay(SysCtlClockGet() / (1000 * 3));       // delay some time
    }
}
