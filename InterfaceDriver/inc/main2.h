// Headers


#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "F28x_Project.h"

#include "user_structures.h"
#include "extern_functions.h"

#ifdef _FLASH
#include "hw_memmap.h"
#include "hw_types.h"
#endif




//
// TODO Defines, variables and structures
ArretirStatusEnum ArretirStatus = { 1, 2, 3 };
ArretirCommandEnum ArretirCommand = { 0xA0, 0xC0, 0xB0, 0xD0, 0xE0, 0xE1, 0xE2, 0xE3 };

SciOperatingModeEnum SciOperatingMode =
{ 0, 4, 5, 1, 2, 3,
  0, 4, 5, 1, 2, 3 };


SciCommandEnum SciCommand = { 0xAA, 0xA3, 0xB1, 0xFF, 32};
SciRxCommandStruct  sciRxCommand = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
SciTxCommandStruct  sciTxCommand = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

TimerStruct timer = { 0, 0, 0, 0, 0, 0 };







//
char arretirStatusClosing = 0;


Uint8 driverSwitcher;

Uint8 oldCommand = 0x00;
char temperatureDS1624Top = -127;
char temperatureDS1624Bot = -127;
char temperatureDS1624TopNew = -127;

long DS1624Timer = 0;
char heatStatus = 2;
char heatFlag = 2;
char heatStatusToSendOld = 2;
char heatStatusToSendNew = 2;
char heatStatusWasSent = 2;

unsigned char hollStatus = 0;


char techFlag = 0;
char stableTimeFlag = 0;
//long arretirWorkingTime = 0;
extern unsigned long arretirWorkingTime;
extern char arretirError;



#define ADC_PU_SCALE_FACTOR        0.000244140625     //1/2^12
#define ADC_PU_PPB_SCALE_FACTOR    0.000488281250     //1/2^11
#define SD_PU_SCALE_FACTOR         0.000030517578125  // 1/2^15

#define REFERENCE_VDAC     0
#define REFERENCE_VREF     1


#define StatusCOM  0xC0
#define Driver     0xB1
#define Komp       0xA1
#define Velocity   0xC1
#define Limit      0xC2
#define Targeting  0xD0
#define lenght_Com 8
#define Finish     0xFF

#define Arr_Off    0
#define Arr_On     1
#define Arr_Stop   2

#define OPEN  0
#define CLOSE 1
#define STOP  2


#define Heating_On      0xD4        //включить нагрев прибора
#define Heating_Off     0xD5        //выключить нагрев прибора

#define Status_Drv_On_Nignii 0x16
#define Status_Drv_Off_Nignii 0<<4
#define Status_Drv_On_verhniy 0x32
#define Status_Drv_Off_verhniy 0<<5
#define ARR_do_nothing 2
#define DELAY      10000

//------**TMP100
// TMP100 commands
#define TMP100_SLAVE            0x49 //TMP102   //0x48    // slave address TMP100 (ADDR0=ADDR1=0)
#define POINTER_TEMPERATURE     0
#define POINTER_CONFIGURATION   1
#define POINTER_T_LOW           2
#define POINTER_T_HIGH          3

int temperature = -127;    // temperature = 2' Сomplement of temperature (-128 ... +127 Celsius)
int pressure = -127;
// is an I8Q8 - Value
//------**TMP100
bool ManualHeatingOn = false;

Uint16 Bukva=0;
unsigned char index2=0;
unsigned char index3=0;
//unsigned char y=0;
unsigned char y3=0;
unsigned char flag = 1;
Uint8 function = 3;
unsigned char EF;

Uint32  CurrentTime = 0;
Uint32  timerForTmp100 = 0;

Uint16 sensorSample;
int16 sensorTemp;


////----------------------ADIS16488------------------
Uint16 sData;  // send data
Uint16 rData;  // received data
MemsDataStruct   *memsData;
Uint16      errorCounter = 0;
float32      deltaXSec = 0;
float32      deltaYSec = 0;
float32      deltaZSec = 0;
int32      xGyroInt32 = 0;
int32      yGyroInt32 = 0;
int32      zGyroInt32 = 0;
int32      X_Gyro = 0;
int32      Y_Gyro = 0;
int32      Z_Gyro = 0;
Uint16     pressureOfMems = 0;
int16      temperatureOfMems = 0;

float32 xCoef = -0.6;
float32 yCoef = 0.56;
float32 zCoef = -0.25;

volatile Uint32 xintCount;
Uint32 Xint1CountOld = 0;



int32 CounterForI2C = 0;    //счетчик обращений к I2C



Uint8 sciTxArray[32];
Uint16 sciRxBufferCounter = 0;

Uint16 sciRxData = 0;
unsigned char sciRxBufferArray[32];
unsigned char sciRxArray[32];

int16 spiRxData = 0;
int16 spiTxData = 0;

#define RESOLUTION_12BIT   0 //12-bit resolution
#define RESOLUTION_16BIT   1 //16-bit resolution (not supported for all variants)

// Function Prototypes
#define SIGNAL_SINGLE          0 //single-ended channel conversions (12-bit mode only)
#define SIGNAL_DIFFERENTIAL    1 //differential pair channel conversions

Uint32 sciRxFlag = 0;


// Reload
#pragma DATA_SECTION(firmwareBuffer, "FirmwareBufferSection");
Uint16 firmwareBuffer[2];

#pragma DATA_SECTION(firmwareLengthInRam, "FirmwareLengthSection");
Uint16 firmwareLengthInRam = 0;
Uint16 firmwareLength = 0;

#pragma DATA_SECTION(firmwareSumInRam, "FirmwareSumSection");
Uint16 firmwareSumInRam = 0;
Uint16 firmwareSum = 0;

Uint16 firmwareCounter = 0;

#define JUMP_TO_FLASHAPI asm("    JMP 0x080000")

long xLong = 0;
long yLong = 0;
long zLong = 0;

char stabilizationFlag = FALSE;
char oldStabilizationFlag = FALSE;
unsigned long memsCounter = 0;
unsigned long memsTimer = 0;
float memsXSum = 0;
float memsYSum = 0;
float memsZSum = 0;

/// Reload

//
// TODO Prototypes
//


void DriversOff(void);
void DriversOn(void);
void RestartVideoAndPlis(void);
//void CheckDrivers(void);
void Wait(long delay);

void InitI2c(void);

interrupt void xint1_isr(void);

void SciSendData(unsigned char lenght, unsigned char *arr);

void ReadMems(void);
void InitSci(void);
void InitSystem(void);
void InitInterrupts(void);
void InitEveryGpioWeNeed(void);
void InitXint(void);
void InitAdc(void);
void InitSpia(void);
void StartCpuTimer(void);
void ReadTmp100(void);
void StartDrivers(void);
void StartMems(void);
void spi_xmit(Uint16 a);

void CheckHeat(void);
void CheckArretirCommand(void);
void CheckArretirStatus(void);

void InitVariables(void);

void SciParseNormal();
void SciParseBeginReload(void);
void SciParseRecieveThirdPart();
void SciParseEndReload();
void SciParseRecieveFirstPart();
void SciParseRecieveSecondPart();
void SciPrepareNormal();
void SciParseReload();
void SciPrepareReload();
void CheckSci();
void CheckTmp100();

//
// TODO Rarely used functions
//

void InitSci(void)
{

    SciaRegs.SCICCR.all = 0x0007;   // 1 stop bit,  No loopback
    // No parity,8 char bits,
    // async mode, idle-line protocol
    SciaRegs.SCICTL1.all = 0x0003;  // enable TX, RX, internal SCICLK,
    SciaRegs.SCICTL2.bit.RXBKINTENA = 1;
    SciaRegs.SCIHBAUD.all = 0x0000;
    SciaRegs.SCILBAUD.all = 0x000B;
    SciaRegs.SCICTL1.all = 0x0023;  // Relinquish SCI from Reset*/
}

void ConfigureADC(void)
{
    //Write ADC configurations and power up the ADC for both ADC A
    Uint16 i;

    EALLOW;

    //write configurations for ADC-A
    // External REFERENCE must be provided
    AdcaRegs.ADCCTL2.bit.PRESCALE   = 6; //set ADCCLK divider to /4
    AdcaRegs.ADCCTL2.bit.RESOLUTION = RESOLUTION_12BIT;
    AdcaRegs.ADCCTL2.bit.SIGNALMODE = SIGNAL_SINGLE;

    //Set pulse positions to late
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;

    //power up the ADC
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;

    //write configurations for ADC-B
    // External REFERENCE must be provided
    AdcbRegs.ADCCTL2.bit.PRESCALE   = 6; //set ADCCLK divider to /4
    AdcbRegs.ADCCTL2.bit.RESOLUTION = RESOLUTION_12BIT;
    AdcbRegs.ADCCTL2.bit.SIGNALMODE = SIGNAL_SINGLE;

    //Set pulse positions to late
    AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;

    //power up the ADC
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;

    //  //write configurations for ADC-C
    // External REFERENCE must be provided
    AdccRegs.ADCCTL2.bit.PRESCALE   = 6; //set ADCCLK divider to /4
    AdccRegs.ADCCTL2.bit.RESOLUTION = RESOLUTION_12BIT;
    AdccRegs.ADCCTL2.bit.SIGNALMODE = SIGNAL_SINGLE;

    //Set pulse positions to late
    AdccRegs.ADCCTL1.bit.INTPULSEPOS = 1;

    //power up the ADC
    AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    //
    //  //write configurations for ADC-D
    // External REFERENCE must be provided
    AdcdRegs.ADCCTL2.bit.PRESCALE   = 6; //set ADCCLK divider to /4
    AdcdRegs.ADCCTL2.bit.RESOLUTION = RESOLUTION_12BIT;
    AdcdRegs.ADCCTL2.bit.SIGNALMODE = SIGNAL_SINGLE;

    //Set pulse positions to late
    AdcdRegs.ADCCTL1.bit.INTPULSEPOS = 1;

    //power up the ADC
    AdcdRegs.ADCCTL1.bit.ADCPWDNZ = 1;

    //delay for > 1ms to allow ADC time to power up
    for(i = 0; i < 1000; i++){
        asm("   RPT#255 || NOP");
    }

    EDIS;
}

//
// ConfigureEPWM - Configure EPWM SOC and compare values
//
void InitEpwm(void)
{
    EALLOW;
    //
    // Assumes ePWM clock is already enabled
    //
    EPwm1Regs.ETSEL.bit.SOCAEN    = 0;    // Disable SOC on A group
    EPwm1Regs.ETSEL.bit.SOCASEL    = 4;   // Select SOC on up-count
    EPwm1Regs.ETPS.bit.SOCAPRD = 1;       // Generate pulse on 1st event
    EPwm1Regs.CMPA.bit.CMPA = 0x0800;     // Set compare A value to 2048 counts
    EPwm1Regs.TBPRD = 0x1000;             // Set period to 4096 counts
    EPwm1Regs.TBCTL.bit.CTRMODE = 3;      // freeze counter
    EDIS;
}

//
// SetupADCEpwm - Configure ADC EPWM acquisition window and trigger
//
void SetupADCEpwm(void)
{
    Uint16 tempsensor_acqps;

    tempsensor_acqps = 139; //temperature sensor needs at least 700ns
    //acquisition time
    //
    //Select the channels to convert and end of conversion flag
    //
    EALLOW;
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 13;  //SOC0 will convert internal
    //connection A13
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = tempsensor_acqps; //sample window is 100
    //SYSCLK cycles
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 5; //trigger on ePWM1 SOCA/C
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0; //end of SOC0 will set INT1 flag
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
    EDIS;
}





