// Read data from MPU6050, and show them via UART
// author: Xiangyu Wen
// Date: 2022.11.01

// hardware connection:
// MPU VCC -> pad 3.3v
// MPU SCL -> pad PB2
// MPU SDA -> pad PB3

// Software functions:
// In this project folder, you may only focus on the main.c and tm4c123gh6pm_startup_ccs.c,
// the other files support the function of MPU6050 reading.
// I2C0 used to communicate with MPU

// send package format:
// R aaa bbb ccc x


// NOTE:
// the raw data of these three axes are all from -180 to 180.
// No matter which two axes you decide to use to control the servo,
// you need to ensure that the the scale of servo Pitch angle should be constrained to 20 - 110,
// and the scale of servo Yaw angle should be constrained to 20 - 160.
// Otherwise, the servo may be broken!

// What you need to do:
// 1. design a parser to process the raw MPU data to a proper format according to the NOTE.
// 2. show the processed data via UART

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdbool.h>
#include "sensorlib/i2cm_drv.h"
#include "sensorlib/hw_mpu6050.h"
#include "sensorlib/mpu6050.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "inc/hw_i2c.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/debug.h"
#include "driverlib/interrupt.h"
#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"

#include <math.h>

#include "utils/uartstdio.h"
#include "driverlib/uart.h"

#include "include.h"

// A boolean that is set when a MPU6050 command has completed.
volatile bool g_bMPU6050Done;

// I2C master instance
tI2CMInstance g_sI2CMSimpleInst;

//Device frequency
int clockFreq;


void InitI2C0(void)
{
    //enable I2C module 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);

    //reset module
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);

    //enable GPIO peripheral that contains I2C 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    // Configure the pin muxing for I2C0 functions on port B2 and B3.
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);

    // Select the I2C function for these pins.
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

    // Enable and initialize the I2C0 master module.  Use the system clock for
    // the I2C0 module.
    // I2C data transfer rate set to 400kbps.
    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), true);

    //clear I2C FIFOs
    HWREG(I2C0_BASE + I2C_O_FIFOCTL) = 80008000;

    // Initialize the I2C master driver.
    I2CMInit(&g_sI2CMSimpleInst, I2C0_BASE, INT_I2C0, 0xff, 0xff, SysCtlClockGet());

}

void delayMS(int ms) {
    //ROM_SysCtlDelay( (ROM_SysCtlClockGet()/(3*1000))*ms ) ;  // more accurate
    SysCtlDelay( (SysCtlClockGet()/(3*1000))*ms ) ;  // less accurate
}

// The function that is provided by this example as a callback when MPU6050
// transactions have completed.
void MPU6050Callback(void *pvCallbackData, uint_fast8_t ui8Status){
    // See if an error occurred.
    if (ui8Status != I2CM_STATUS_SUCCESS){
        // An error occurred, so handle it here if required.
    }
    // Indicate that the MPU6050 transaction has completed.
    g_bMPU6050Done = true;
}


// The interrupt handler for the I2C module.
void I2CMSimpleIntHandler(void){
    // Call the I2C master driver interrupt handler.
    I2CMIntHandler(&g_sI2CMSimpleInst);
}


// read data from MPU6050.
static const float dt = 1 / 200.0;
static const int ZERO_OFFSET_COUN = (int)(200);

static const float dt_2 = 1 / 150.0;
static const int ZERO_OFFSET_COUN_2 = (int)(150);

static int g_GetZeroOffset = 0;
static float gyroX_offset = 0.0f, gyroY_offset = 0.0f, gyroZ_offset = 0.0f;


void MPU6050Example(int *pitch, int *roll, int *yaw)
{
    double fAccel[3], fGyro[3];
    double tmp;
    float gyroX, gyroY, gyroZ;

    MPU6050_Read(&fAccel[0], &fAccel[1],&fAccel[2], &fGyro[0],&fGyro[1],&fGyro[2],&tmp);

    gyroX = fGyro[0];
    gyroY = fGyro[1];
    gyroZ = fGyro[2];



    if (g_GetZeroOffset++ < ZERO_OFFSET_COUN)
    {
        gyroX_offset += gyroX * dt;
        gyroY_offset += gyroY * dt;
        gyroZ_offset += gyroZ * dt;
    }

    // remove zero shift
    gyroX -= gyroX_offset;
    gyroY -= gyroY_offset;
    gyroZ -= gyroZ_offset;

    static float integralX = 0.0f, integralY = 0.0f, integralZ = 0.0f;
    if (g_GetZeroOffset > ZERO_OFFSET_COUN_2)
    {
        integralX += gyroX * dt_2;
        integralY += gyroY * dt_2;
        integralZ += gyroZ * dt_2;
        if (integralX > 360)
            integralX -= 360;
        if (integralX < -360)
            integralX += 360;
        if (integralY > 360)
            integralY -= 360;
        if (integralY < -360)
            integralY += 360;
        if (integralZ > 360)
            integralZ -= 360;
        if (integralZ < -360)
            integralZ += 360;
    }

    *pitch = (int)integralX;
    *roll = (int)integralY;
    *yaw = (int)integralZ;
    delayMS(5);
}

int X = 0, Y = 0, Z = 0;
int main(){
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    // initialize I2C, you may do not care this part
    InitI2C0();

    MPU6050_Config(0x68, 1, 1);
    MPU6050_Calib_Set(903, 156, 1362, -4, 56, -16);


    while(1){
        // get raw data from MPU6050
        MPU6050Example(&X, &Y, &Z);
        // scale to a proper Master rotation


        // NOTE:
        // the raw data of these three axes are all from -180 to 180.
        // No matter which two axes you decide to use to control the servo,
        // you need to ensure that the the scale of servo Pitch angle should be constrained to 20 - 110,
        // and the scale of servo Yaw angle should be constrained to 20 - 160.
        // Otherwise, the servo may be broken!

        // design your parse to process the collected raw MPU data here


        // show data here (recommend to use interrupt)
        // example: R aaa bbb ccc x

    }

    return(0);
}



