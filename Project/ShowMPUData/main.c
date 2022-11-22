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

#define MIN_PITCH_ANGLE 20
#define MAX_PITCH_ANGLE 110
#define MIN_YAW_ANGLE 20
#define MAX_YAW_ANGLE 160

// Storing the data from the MPU and the data to be sent via UART
int X = 0, Y = 0, Z = 0, pitch = 0, yaw = 0;

// A boolean that is set when a MPU6050 command has completed.
volatile bool g_bMPU6050Done;

// I2C master instance
tI2CMInstance g_sI2CMSimpleInst;

//Device frequency
int clockFreq;

// read data from MPU6050.
static const float dt = 1 / 200.0;
static const int ZERO_OFFSET_COUN = (int)(200);

static const float dt_2 = 1 / 150.0;
static const int ZERO_OFFSET_COUN_2 = (int)(150);

static int g_GetZeroOffset = 0;
static float gyroX_offset = 0.0f, gyroY_offset = 0.0f, gyroZ_offset = 0.0f;

void ResetMPUData(void)
{
    X=0;
    Y=0;
    Z=0;
    pitch =0;
    yaw = 0;
    g_GetZeroOffset = 0;
    gyroX_offset = 0;
    gyroY_offset = 0;
    gyroZ_offset = 0;
}

char* Int_toString(int value)
{
    char temp[20];
    char *result = malloc(sizeof(char) * 20);
    int tempCount = 0;
    int resultCount = 0;

    if(value == 0)
    {
        result[0] = '0';
        result [1] = '\0';
        return result;
    }

    if(value < 0)
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

    return &result[0];
}

void InitializeMPU(void)
{
    MPU6050_Config(0x68, 1, 1);
    MPU6050_Calib_Set(903, 156, 1362, -4, 56, -16);
}

void GPIO_PORtF_Handler(void);
void InitializeButton(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    // set GPIOs for buttons
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE + GPIO_O_CR)  |= 0x01;
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;
    GPIODirModeSet(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0, GPIO_DIR_MODE_IN);
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0);   // PF4 input
    GPIOIntEnable(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0);                 // interrupt enable
    GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0, GPIO_FALLING_EDGE); // only interrupt at falling edge (pressed)
    GPIOIntRegister(GPIO_PORTF_BASE, GPIO_PORtF_Handler);           // dynamic isr registering
}

void InitializeUART()
{
    // Enable UART0 and GPIOA to send signals via UART
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    // Configure and enable UART
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    IntMasterEnable();
    IntEnable(INT_UART0);
    UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
}

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
    UARTStringPut(ui32Base, Int_toString(value));
}

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


void GetMPUValue(int *pitch, int *roll, int *yaw)
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

void GetNormalizedPitchYaw(int X, int Y, int Z, int *pitch, int *yaw)
{
    // For convenience, we use:
    //     negative Y as pitching up, positive Y as pitching down,
    //     positive Z as yawing left, negative Z as yawing right,
    //     and we don't care about the X.

    Y *= -1, Z *= -1;

    static int lastX = 0, lastY = 0, lastZ = 0;

    // calculate delta change
    int deltaY = Y - lastY, deltaZ = Z - lastZ;

    // Scale the delta value so that the control feels normal
    deltaY *= 2;
    deltaZ *= 2;

    *pitch += deltaY, *yaw += deltaZ;

    // Clamp the pitch
    if(*pitch < MIN_PITCH_ANGLE)
        *pitch = MIN_PITCH_ANGLE;
    else if(*pitch > MAX_PITCH_ANGLE)
        *pitch = MAX_PITCH_ANGLE;

    // Clamp the yaw
    if(*yaw < MIN_YAW_ANGLE)
        *yaw = MIN_YAW_ANGLE;
    else if(*yaw > MAX_YAW_ANGLE)
        *yaw = MAX_YAW_ANGLE;

    lastX = X, lastY = Y, lastZ = Z;
}

void Initialize(void)
{
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    // Initialize I2C
    InitI2C0();

    // Initialize UART
    InitializeUART();

    // Initialize MPU
    InitializeMPU();

    // Initialize Buttons
    InitializeButton();
}

int main(){
    Initialize();

    while(1){
    }
}

// The interrupt handler for the I2C module.
void I2CMSimpleIntHandler(void){
    // Call the I2C master driver interrupt handler.
    I2CMIntHandler(&g_sI2CMSimpleInst);

    // Get the value from the MPU
    GetMPUValue(&X, &Y, &Z);
    // Normalize the value for the servo
    GetNormalizedPitchYaw(X,Y, Z, &pitch, &yaw);

    // Display the Value in UART
    UARTCharPut(UART0_BASE, 'R');
    UARTCharPut(UART0_BASE, ' ');
    UARTIntPut(UART0_BASE, X);
    UARTCharPut(UART0_BASE, ' ');
    UARTIntPut(UART0_BASE, Y);
    UARTCharPut(UART0_BASE, ' ');
    UARTIntPut(UART0_BASE, Z);
    UARTStringPut(UART0_BASE, " x\n\r");

    delayMS(5);
}

// Handle the button input
void GPIO_PORtF_Handler(void)
{
    GPIOIntClear(GPIO_PORTF_BASE, GPIO_INT_PIN_4 | GPIO_INT_PIN_0);
    ResetMPUData();
}
