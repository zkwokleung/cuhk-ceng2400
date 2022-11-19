// Bluetooth Serial Echo program

// Author: Xu Zhijian Oct,20,2022

// Simple Bluetooth serial reading from Host device (PC/Phone)

/*
Hardware connection:
RXD -> PE5
TXD -> PE4
GND -> GND
VCC -> VBUS
EN -> VCC (for AT mode)

Software Implementation:
UART0 used to communicate with computer (only send in this demo)
UART5 used to communicate with HC05 (only receive in this demo)

Procedures:
1. Smartphone/PC with bluetooth use bluetooth serial to send message to HC05,
2. HC05 is connected to the UART5 on Tiva board. HC05 sends the message via UART5.
3. Board receive character from HC05 in UART5 and forward to UART0 to computer.
4. (recommended tools are listed in slide appendix).
*/


#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "inc/hw_ints.h"
#include "driverlib/interrupt.h"

int main(void) {

    // set clock
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

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

    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2|GPIO_PIN_1|GPIO_PIN_3, 2) ;

    // set interrupt for receiving and showing values
    IntMasterEnable();
    IntEnable(INT_UART5);
    UARTIntEnable(UART5_BASE, UART_INT_RX | UART_INT_RT);

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

