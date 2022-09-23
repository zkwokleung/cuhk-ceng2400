#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"

uint8_t ui8PinData=2;
int32_t SW1 = 0;
int32_t SW2 = 0;

void OnRGB(uint32_t delay){
    // Write RGB color
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, ui8PinData);
                // Delay
                SysCtlDelay(delay);

                // Change color
                if(ui8PinData == 0x0E)
                    ui8PinData = 0x02;
                else
                    ui8PinData += 2;
}

int main(void)
{
    SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);

    // Unlock PF0
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0X01;
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;
    // Use PF4 and PF0 for switch state 
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_4 | GPIO_PIN_0);
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4 | GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    while(1)
    {
        SW1 = GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4);
        SW2 = GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_0);

        if (SW2 == 0){
            // Pin1 is low => SW2 is pressed
            // Shorter Delay
            OnRGB(1000000);
        }else{
            if (SW1 == GPIO_PIN_4) {
                // Pin4 is high => SW1 is not pressed
                // Set data to red (010)
                ui8PinData = 2;
                // Write to PIN
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, ui8PinData);
                SysCtlDelay(2000000);
            }
            else {
                // Pin4 is low => SW1 is pressed
                OnRGB(2000000);
            }
        }
    }
}
