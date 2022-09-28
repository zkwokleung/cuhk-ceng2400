/*
 * lab3_2.c
 *
 *  Created on: 14 Sep 2021
 *      Author: rygao
 */


#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"

#define datapins GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
#define RS GPIO_PIN_5
#define RW GPIO_PIN_6
#define EN GPIO_PIN_7
#define ROW GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4
#define COL GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6

#define NUMPAD_KEY_NONE 0xF
#define NUMPAD_KEY_0 0x0
#define NUMPAD_KEY_1 0x1
#define NUMPAD_KEY_2 0x2
#define NUMPAD_KEY_3 0X3
#define NUMPAD_KEY_4 0x4
#define NUMPAD_KEY_5 0x5
#define NUMPAD_KEY_6 0x6
#define NUMPAD_KEY_7 0x7
#define NUMPAD_KEY_8 0x8
#define NUMPAD_KEY_9 0x9
#define NUMPAD_KEY_STAR 0xA
#define NUMPAD_KEY_HASHTAG 0xB
#define MAX_PWD_LENGTH 16

enum State { set_pwd, read_pwd, verify_pwd, correct_pwd, incorrect_pwd, };

int numpad_read();
void display_input(int input);
void clear_row(int row);
void flushInput(uint32_t ui32Port, uint8_t ui8Pins);
void delayMs(int n);
void delayUs(int n);
void LCD_command(unsigned char command);
void LCD_data(unsigned char data);
void LCD_init(void);
void LCD_position(int hor_x, int ver_y);
void LCD_display_message(char *message_str);

int main(void)
{
    SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, datapins);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, RS|RW|EN);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, ROW);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, COL);
    GPIOPadConfigSet(GPIO_PORTC_BASE, COL,GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPU);

    enum State next_state = set_pwd;
    int input_length = 0;
    uint64_t pwd = 0;
    int pwd_length = 0;
    uint64_t input_pwd = 0;
    short pwd_entered = 0;

    while(1)
    {
        switch(next_state)
        {
        case set_pwd:
            // Display the message
            LCD_init();
            LCD_position(0,1);
            LCD_display_message("Please set PWD:");
            LCD_position(0,2);

            // Read numpad input
            while(!pwd_entered)
            {
                int ipt = numpad_read();
                if(ipt != NUMPAD_KEY_NONE)
                {
                    if(ipt == NUMPAD_KEY_HASHTAG)
                    {
                        // Enter is pressed
                        if(input_length)
                        {
                            // Move to next state if the pwd is set
                            pwd_entered = 1;
                            pwd_length = input_length;
                            next_state = read_pwd;
                        }
                    }
                    else if(input_length < MAX_PWD_LENGTH)
                    {
                        // Keep reading and displaying the pwd if it have not reach the maximum length
                        display_input(ipt);
                        input_length++;
                        pwd = pwd << 4;
                        pwd += ipt;
                    }
                }
            }
            break;

        case read_pwd:
                clear_row(1);
                clear_row(2);
               LCD_position(0,1);
               LCD_display_message("Plz enter PWD:");
               LCD_position(0,2);

               pwd_entered = 0;
               input_pwd = 0;
               input_length = 0;

               // Read numpad input
               while(!pwd_entered)
               {
                   int ipt = numpad_read();
                   if(ipt != NUMPAD_KEY_NONE)
                   {
                       if(ipt == NUMPAD_KEY_HASHTAG)
                       {
                           // Move to next state if the pwd is entered
                           pwd_entered = 1;
                           next_state = verify_pwd;
                       }
                       else if(input_length < MAX_PWD_LENGTH)
                       {
                           // Keep reading and displaying the pwd if it have not reach the maximum length
                           display_input(ipt);
                           input_length++;
                           input_pwd = input_pwd << 4;
                           input_pwd += ipt;
                       }
                   }
               }
            break;

        case verify_pwd:
            if(input_pwd == pwd)
                next_state = correct_pwd;
            else
                next_state = incorrect_pwd;
            break;

        case correct_pwd:
            clear_row(1);
            clear_row(2);
            LCD_position(0, 1);
            LCD_display_message("Correct!");

            delayMs(1500);
            next_state = read_pwd;
            break;

        case incorrect_pwd:
            clear_row(1);
            clear_row(2);
            LCD_position(0, 1);

            // Check password length
            if(input_length < pwd_length)
            {
                LCD_display_message("Wrong[too short]");
            }else if(input_length > pwd_length)
            {
                LCD_display_message("Wrong[too long]");
            }
            else
            {
                LCD_display_message("Wrong! try again");
            }

            // Back to read password state
            delayMs(1500);
            next_state = read_pwd;
            break;
        }

    }
}

int numpad_read(){
    int val = NUMPAD_KEY_NONE;
    GPIOPinWrite(GPIO_PORTE_BASE,ROW,0x1C); // first row
    if (!GPIOPinRead(GPIO_PORTC_BASE,GPIO_PIN_4)) {
        val = NUMPAD_KEY_1;
         flushInput(GPIO_PORTC_BASE,GPIO_PIN_4);
    }
    else if(!GPIOPinRead(GPIO_PORTC_BASE,GPIO_PIN_5)) {
        val = NUMPAD_KEY_2;
        flushInput(GPIO_PORTC_BASE,GPIO_PIN_5);
    }
    else if(!GPIOPinRead(GPIO_PORTC_BASE,GPIO_PIN_6)) {
        val = NUMPAD_KEY_3;
        flushInput(GPIO_PORTC_BASE,GPIO_PIN_6);
    }

    GPIOPinWrite(GPIO_PORTE_BASE,ROW,0x1A); // second row
    if (!GPIOPinRead(GPIO_PORTC_BASE,GPIO_PIN_4)) {
        val = NUMPAD_KEY_4;
        flushInput(GPIO_PORTC_BASE,GPIO_PIN_4);
     }
     else if(!GPIOPinRead(GPIO_PORTC_BASE,GPIO_PIN_5)) {
         val = NUMPAD_KEY_5;
        flushInput(GPIO_PORTC_BASE,GPIO_PIN_5);
     }
     else if(!GPIOPinRead(GPIO_PORTC_BASE,GPIO_PIN_6)) {
         val = NUMPAD_KEY_6;
        flushInput(GPIO_PORTC_BASE,GPIO_PIN_6);
     }

    GPIOPinWrite(GPIO_PORTE_BASE,ROW,0x16); // third row
    if (!GPIOPinRead(GPIO_PORTC_BASE,GPIO_PIN_4)) {
        val = NUMPAD_KEY_7;
        flushInput(GPIO_PORTC_BASE,GPIO_PIN_4);
    }
    else if(!GPIOPinRead(GPIO_PORTC_BASE,GPIO_PIN_5)) {
        val = NUMPAD_KEY_8;
        flushInput(GPIO_PORTC_BASE,GPIO_PIN_5);
    }
    else if(!GPIOPinRead(GPIO_PORTC_BASE,GPIO_PIN_6)) {
        val = NUMPAD_KEY_9;
        flushInput(GPIO_PORTC_BASE,GPIO_PIN_6);

    }

    GPIOPinWrite(GPIO_PORTE_BASE,ROW,0x0E); // forth row
    if (!GPIOPinRead(GPIO_PORTC_BASE,GPIO_PIN_4)) {
        val = NUMPAD_KEY_STAR;
        flushInput(GPIO_PORTC_BASE,GPIO_PIN_4);
    }
    else if(!GPIOPinRead(GPIO_PORTC_BASE,GPIO_PIN_5)) {
        val = NUMPAD_KEY_0;
        flushInput(GPIO_PORTC_BASE,GPIO_PIN_5);
    }
    else if(!GPIOPinRead(GPIO_PORTC_BASE,GPIO_PIN_6)) {
        val = NUMPAD_KEY_HASHTAG;
        flushInput(GPIO_PORTC_BASE,GPIO_PIN_6);
    }

    return val;
}

void clear_row(int row)
{
    LCD_position(0,row);
    LCD_display_message("                    ");
}

void flushInput(uint32_t ui32Port, uint8_t ui8Pins){
    /* wait until the key is release to avoid redundant inputs. */
    while(!GPIOPinRead(ui32Port, ui8Pins)) {
        delayMs(100);
    }
}

void display_input(int input)
{
    if(input == NUMPAD_KEY_HASHTAG)
        return;

    if(input == NUMPAD_KEY_STAR)
        LCD_data('*');
    else
        LCD_data(input + 48);
}

void delayMs(int n)
{
    int i,j;
    for(i = 0;i < n;i++)
        for(j = 0;j < 3180; j++)
            {}
}

void delayUs( int n)
{
    int i,j;
    for(i = 0;i < n;i++)
        for(j = 0;j < 3; j++)
            {}
}

void LCD_init(void)
{
    delayMs(20); /* initialization sequence */
    LCD_command(0x30);
    delayMs(50);
    LCD_command(0x30);
    delayUs(500);
    LCD_command(0x30);

    LCD_command(0x38); // set 8-bit data, 2-line, 5x7 font
    LCD_command(0x06); // cursor move direction: increase
    LCD_command(0x01); // display clear
    LCD_command(0x02); // cursor move to first digit
    LCD_command(0x0F); //turn on display, cursor blinking
}

void LCD_command(unsigned char command)
{
    GPIOPinWrite(GPIO_PORTA_BASE, RS, 0x00); //RS = 0->command mode
    GPIOPinWrite(GPIO_PORTA_BASE, RW, 0x00); //RW = 0->writing mode
    GPIOPinWrite(GPIO_PORTB_BASE, datapins, command); // Write the command to data pins
    GPIOPinWrite(GPIO_PORTA_BASE, EN, 0x80); // Set EN to high
    delayUs(500);
    GPIOPinWrite(GPIO_PORTA_BASE, EN, 0x00); // Set EN to low, a high-to-low pulse
    if (command < 4)
        delayMs(20); /* command 1 and 2 needs up to 1.64ms */
    else
        delayUs(400); /* all others 40 us */
}

void LCD_data(unsigned char data)
{
    GPIOPinWrite(GPIO_PORTA_BASE, RS, 0x20); //RS = 0->command mode
    GPIOPinWrite(GPIO_PORTA_BASE, RW, 0x00); //RW = 0->writing mode
    GPIOPinWrite(GPIO_PORTB_BASE, datapins, data); // Write the command to data pins
    GPIOPinWrite(GPIO_PORTA_BASE, EN, 0x80); // Set EN to high
    delayUs(500);
    GPIOPinWrite(GPIO_PORTA_BASE, EN, 0x00); // Set EN to low, a high-to-low pulse
}

void LCD_position(int hor_x, int ver_y) {
        switch(ver_y) {
    case 1:
                LCD_command(0x80+hor_x);
                delayMs(50);
        break;
    case 2:
                LCD_command(0xc0+hor_x);
                delayMs(50);
        break;
    case 3:
                LCD_command(0x94+hor_x);
                delayMs(50);
        break;
    case 4:
                LCD_command(0xd4+hor_x);
                delayMs(50);
        break;
        }
}

void LCD_display_message(char *message_str) {
    int n;

    n=0;
    while (message_str[n]!='\0') {
        LCD_data(message_str[n]);
        n++;
    }
}
