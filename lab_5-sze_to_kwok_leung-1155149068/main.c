#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"

#define MAX_QUEUE_SIZE 100

struct
{
    char data[MAX_QUEUE_SIZE];
    uint32_t head;
    uint32_t tail;
    uint32_t count;
} typedef Queue;

void Q_Init(Queue *q)
{
    int i;
    for (i = 0; i < MAX_QUEUE_SIZE; i++)
        q->data[i] = 0;
    q->head = 0;
    q->tail = 0;
    q->count = 0;
}

int Q_Empty(Queue *q)
{
    return q->count == 0;
}

int Q_Full(Queue *q)
{
    return q->count == MAX_QUEUE_SIZE;
}

int Q_Enequeue(Queue *q, char data)
{
    if (!q || Q_Full(q))
        return -1;
    q->data[q->tail] = data;
    q->tail = (q->tail + 1) % MAX_QUEUE_SIZE;
    q->count++;
    return 0;
}

char Q_Dequeue(Queue *q)
{
    if (!q || Q_Empty(q))
        return 0;
    char data = q->data[q->head];
    q->head = (q->head + 1) % MAX_QUEUE_SIZE;
    q->count--;
    return data;
}

Queue q;

int main(void)
{
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

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

    Q_Init(&q);

    UARTCharPut(UART0_BASE, 'E');
    UARTCharPut(UART0_BASE, 'n');
    UARTCharPut(UART0_BASE, 't');
    UARTCharPut(UART0_BASE, 'e');
    UARTCharPut(UART0_BASE, 'r');
    UARTCharPut(UART0_BASE, ' ');
    UARTCharPut(UART0_BASE, 'T');
    UARTCharPut(UART0_BASE, 'e');
    UARTCharPut(UART0_BASE, 'x');
    UARTCharPut(UART0_BASE, 't');
    UARTCharPut(UART0_BASE, ':');
    UARTCharPut(UART0_BASE, ' ');

    while (1)
    {
    }
}

void UARTStringPut(char *str)
{
    int i;
    for (i = 0; str[i] != '\0'; i++)
    {
        UARTCharPut(UART0_BASE, str[i]);
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

        // If it is an enter key, then print the queue
        if (c == 10 || c == 13)
        {
            UARTCharPut(UART0_BASE, '\n');
            UARTCharPut(UART0_BASE, '\r');
            while (!Q_Empty(&q))
            {
                UARTCharPut(UART0_BASE, Q_Dequeue(&q));
            }
            UARTCharPut(UART0_BASE, '\n');
            UARTCharPut(UART0_BASE, '\r');
        }
        else
        {
            UARTCharPut(UART0_BASE, c);                            // echo character
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2); // blink LED
            SysCtlDelay(SysCtlClockGet() / (1000 * 3));            // delay ~1 msec
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);          // turn off LED

            // Convert to upper case and enqueue
            if (c >= 'a' && c <= 'z')
                c = c - 'a' + 'A';

            Q_Enequeue(&q, c);
        }
    }
}
