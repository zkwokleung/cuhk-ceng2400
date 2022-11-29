#include <math.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "inc/hw_gpio.h"
#include "inc/hw_i2c.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "include.h"
#include "sensorlib/hw_mpu6050.h"
#include "sensorlib/i2cm_drv.h"
#include "sensorlib/mpu6050.h"
#include "utils/uartstdio.h"

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
}

/*
 * MPU functions
 */
#define MIN_PITCH_ANGLE 45
#define INIT_PITCH_ANGLE 50
#define MAX_PITCH_ANGLE 110
#define MIN_YAW_ANGLE 20
#define INIT_YAW_ANGLE 55
#define MAX_YAW_ANGLE 89

// Storing the data from the MPU and the data to be sent via UART
int X = 0, Y = 0, Z = 0, pitch = 0, yaw = 0;

// A boolean that is set when a MPU6050 command has completed.
volatile bool g_bMPU6050Done;

// I2C master instance
tI2CMInstance g_sI2CMSimpleInst;

// read data from MPU6050.
static const float dt = 1 / 200.0;
static const int ZERO_OFFSET_COUN = (int)(200);

static const float dt_2 = 1 / 150.0;
static const int ZERO_OFFSET_COUN_2 = (int)(150);

static int g_GetZeroOffset = 0;
static float gyroX_offset = 0.0f, gyroY_offset = 0.0f, gyroZ_offset = 0.0f;

void InitializeMPU(void)
{
    MPU6050_Config(0x68, 1, 1);
    MPU6050_Calib_Set(903, 156, 1362, -4, 56, -16);
}

// The function that is provided by this example as a callback when MPU6050
// transactions have completed.
void MPU6050Callback(void *pvCallbackData, uint_fast8_t ui8Status)
{
    // See if an error occurred.
    if (ui8Status != I2CM_STATUS_SUCCESS)
    {
        // An error occurred, so handle it here if required.
    }
    // Indicate that the MPU6050 transaction has completed.
    g_bMPU6050Done = true;
}

void GetNormalizedPitchYaw(int X, int Y, int Z, int *pitch, int *yaw)
{
    // For convenience, we use:
    //     negative Y as pitching up, positive Y as pitching down,
    //     positive Z as yawing left, negative Z as yawing right,
    //     and we don't care about the X.

    static int lastX = 0, lastY = 0, lastZ = 0;

    // calculate delta change
    double deltaX = X - lastX, deltaY = Y - lastY, deltaZ = Z - lastZ;

    // Scale the delta value so that the control feels normal
    //    deltaY *= 2;
//        deltaZ *= 1.5;

    *pitch += deltaX, *yaw += deltaZ;

    // Clamp the pitch
    if (*pitch < MIN_PITCH_ANGLE)
        *pitch = MIN_PITCH_ANGLE;
    else if (*pitch > MAX_PITCH_ANGLE)
        *pitch = MAX_PITCH_ANGLE;

    // Clamp the yaw
    if (*yaw < MIN_YAW_ANGLE)
        *yaw = MIN_YAW_ANGLE;
    else if (*yaw > MAX_YAW_ANGLE)
        *yaw = MAX_YAW_ANGLE;

    lastX = X, lastY = Y, lastZ = Z;
}

void GetMPU6050Data(int *pitch, int *roll, int *yaw)
{
    double fAccel[3], fGyro[3];
    double tmp;
    float gyroX, gyroY, gyroZ;

    MPU6050_Read(&fAccel[0], &fAccel[1], &fAccel[2], &fGyro[0], &fGyro[1], &fGyro[2], &tmp);

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
}

/*
 * I2C Functions
 */
void InitI2C0(void)
{
    // enable I2C module 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);

    // reset module
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);

    // enable GPIO peripheral that contains I2C 0
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

    // clear I2C FIFOs
    HWREG(I2C0_BASE + I2C_O_FIFOCTL) = 80008000;

    // Initialize the I2C master driver.
    I2CMInit(&g_sI2CMSimpleInst, I2C0_BASE, INT_I2C0, 0xff, 0xff, SysCtlClockGet());
}

void InitializeButton(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE + GPIO_O_CR)  |= 0x01;
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;
    GPIODirModeSet(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0, GPIO_DIR_MODE_IN);
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
}

/*
 * Main System
 */
void Initialize(void)
{
    // Initialize Data
    yaw = INIT_YAW_ANGLE;
    pitch = INIT_PITCH_ANGLE;

    // set clock
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    // Initialize UART
    InitializeUART();

    // initialize I2C, you may do not care this part
    InitI2C0();

    // Initialize MPU6050
    InitializeMPU();
}

int main(void)
{
    Initialize();

    while (1)
    {
    }
}

void I2CIntHandler(void)
{
    // Call the I2C master driver interrupt handler.
    I2CMIntHandler(&g_sI2CMSimpleInst);

    // Get the data from the MPU
    GetMPU6050Data(&X, &Y, &Z);

    // Normalize the data
    GetNormalizedPitchYaw(X, Y, Z, &pitch, &yaw);

    // Send the data to UART5
    UARTCharPut(UART5_BASE, 'y');
    UARTIntPut(UART5_BASE, yaw);
    UARTStringPut(UART5_BASE, "\n\r");

    delayMS(5);

    UARTCharPut(UART5_BASE, 'p');
    UARTIntPut(UART5_BASE, pitch);
    UARTStringPut(UART5_BASE, "\n\r");

    delayMS(5);
}
