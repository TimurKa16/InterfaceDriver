
//#define FLASH  0

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "F28x_Project.h"

#include "user_structures.h"
#include "extern_functions.h"

//#ifdef _FLASH
//#include "hw_memmap.h"
//#include "hw_types.h"
//#endif


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

//#define ARR_open   0
//#define ARR_close  1
#define ARR_open   0xA0
#define ARR_close  0xC0
#define Drv_On_Nignii 0xD0
#define Drv_Off_Nignii 0xD1
#define Drv_On_verhniy 0xD2
#define Drv_Off_verhniy 0xD3
#define Status_Drv_On_Nignii 0x16
#define Status_Drv_Off_Nignii 0<<4
#define Status_Drv_On_verhniy 0x32
#define Status_Drv_Off_verhniy 0<<5
#define ARR_do_nothing 2
#define DELAY      10000
Uint16 Bukva=0;
unsigned char index2=0;
unsigned char index3=0;
//unsigned char y=0;
unsigned char y3=0;
unsigned char flag = 1;
Uint8 function = 3;
unsigned char EF;

Uint32  AppCounter = 0;
Uint32  CurrentTime = 0;

bool  FirstKommandToArretirIsHappened = false;

Uint16 sensorSample;
int16 sensorTemp;

ComPort_t My_Com_32;    //массив приема данных


StatusFlags_t       Flags;

Counters_t          My_counter;

InsideKomands_t     My_Komand1;
OutsideKomands_t    Komand_Com32;

Uint8               My_Com_32_Tx[64];   //массив передачи данных в ПЛИС интерфейса
OutsideMessage_t    My_Otvet32;

////----------------------ADIS16488------------------
Uint16 sdata;  // send data
Uint16 rdata;  // received data
BRFData_t   *BRFData;
Uint16      ErrorOfCount_DATA_CNT = 0;
int32      DeltaXSek = 0;
int32      DeltaYSek = 0;
int32      DeltaZSek = 0;
int32      X_GyroInt32 = 0;
int32      Y_GyroInt32 = 0;
int32      Z_GyroInt32 = 0;
int32      X_Gyro = 0;
int32      Y_Gyro = 0;
int32      Z_Gyro = 0;
Uint16     PressureInPa = 0;
int16      TempOfADIS = 0;

volatile Uint32 Xint1Count;
Uint32 Xint1CountOld = 0;

interrupt void xint1_isr(void);
////----------------------ADIS16488------------------

void init_variables(void)
{
    memset(&My_Com_32, 0, sizeof(ComPort_t));
    My_Com_32.Start1 = 0xAA;
    My_Com_32.Start2 = 0xB1;

    memset(&Komand_Com32, 0, sizeof(OutsideKomands_t));
    memset(&My_counter, 0, sizeof(Counters_t));
    memset(&My_Komand1, 0, sizeof(InsideKomands_t));
    memset(&Flags, 0, sizeof(StatusFlags_t));
    memset(My_Com_32_Tx, 0, sizeof(My_Com_32_Tx));

    memset(&My_Otvet32, 0, sizeof(OutsideMessage_t));

}





///------------ADIS16488-------------

void delay_loop()
{
    long i;
    for (i = 0; i < 1000000; i++) {}
}

void spi_xmit(Uint16 a)
{
    SpiaRegs.SPITXBUF = a;
}

void spi_fifo_init()
{
    SpiaRegs.SPIFFTX.all = 0xE040;
    SpiaRegs.SPIFFRX.all = 0x2044;
    SpiaRegs.SPIFFCT.all = 0x0;

    InitSpi();
}

interrupt void xint1_isr(void)
{
    Xint1Count++;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}
///------------ADIS16488-------------




/////////////////////-----main function------///////////////////////////////
void main(void)
{

    //#ifdef _FLASH
    //// Copy time critical code and Flash setup code to RAM
    //// This includes the following functions:  InitFlash();
    //// The  RamfuncsLoadStart, RamfuncsLoadSize, and RamfuncsRunStart
    //// symbols are created by the linker. Refer to the device .cmd file.
    ///////    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (size_t)&RamfuncsLoadSize);
    //#endif

    init_variables();
    InitSysCtrl();
    //
    //#ifdef _FLASH
    //// Call Flash Initialization to setup flash waitstates
    //// This function must reside in RAM
    //    InitFlash();
    //#endif


    //
    // Step 2. Initialize GPIO:
    // This example function is found in the F2837xD_Gpio.c file and
    // illustrates how to set the GPIO to it's default state.
    //
    //
    // Step 3. Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    //
    //DINT;

    //
    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the F2837xD_PieCtrl.c file.
    //
    InitPieCtrl();

    //
    // Disable CPU interrupts and clear all CPU interrupt flags:
    //
    IER = 0x0000;
    IFR = 0x0000;

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in F2837xD_DefaultIsr.c.
    // This function is found in F2837xD_PieVect.c.
    //
    InitPieVectTable();

    ///------------ADIS16488-------------
    BRFData = (struct BRFData_struct *)malloc(sizeof(BRFData_t));
    memset(BRFData,0,sizeof(*BRFData));

    //    InitSpiaGpio();
    //    DINT;

    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.XINT1_INT = &xint1_isr;
    EDIS;    // This is needed to disable write to EALLOW protected registers

    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;          // Enable the PIE block
    PieCtrlRegs.PIEIER1.bit.INTx4 = 1;          // Enable PIE Group 1 INT4
    PieCtrlRegs.PIEIER1.bit.INTx5 = 1;          // Enable PIE Group 1 INT5
    IER |= M_INT1;                              // Enable CPU INT1
    //    EINT;                                       // Enable Global Interrupts

    ///// для ADIS16497
    EALLOW;

    GpioCtrlRegs.GPBMUX1.bit.GPIO35 = 0;  // GPIO25 = GPIO25
    GpioCtrlRegs.GPBDIR.bit.GPIO35 = 0;   // GPIO25 = input
    GpioCtrlRegs.GPBQSEL1.bit.GPIO35 = 0;        // XINT1 Synch to SYSCLKOUT only

    GpioCtrlRegs.GPBMUX1.bit.GPIO36 = 0;  // GPIO25 = GPIO25
    GpioCtrlRegs.GPBDIR.bit.GPIO36 = 0;   // GPIO25 = input
    GpioCtrlRegs.GPBQSEL1.bit.GPIO36 = 0;     // XINT1 Synch to SYSCLKOUT only

    GpioCtrlRegs.GPAPUD.bit.GPIO6   = 1; // disable pull ups
    GpioCtrlRegs.GPAMUX1.bit.GPIO6  = 0; // choose GPIO for mux option
    GpioCtrlRegs.GPADIR.bit.GPIO6   = 1; // set as output
    GpioDataRegs.GPASET.bit.GPIO6 = 1;


    EDIS;
    ///// для ADIS16497



    EALLOW;
    InputXbarRegs.INPUT4SELECT = 35;      //Set XINT1 source to GPIO-pin
    EDIS;

    Xint1Count = 0; // Count XINT1 interrupts
    XintRegs.XINT1CR.bit.POLARITY = 1;
    XintRegs.XINT1CR.bit.ENABLE = 1;            // Enable XINT1
    ///------------ADIS16488-------------


    InitCpuTimers();    // basic setup CPU Timer0, 1 and 2

    ///ConfigCpuTimer(&CpuTimer0, 150, DELAY); // CPU - Timer0 at 100 milliseconds
    ConfigCpuTimer(&CpuTimer0, 200, 1000); // CPU - Timer0 точность 1 мс

    InitGpio(); // Skipped for this example

    EALLOW;
    //  GpioCtrlRegs.GPAPUD.bit.GPIO23 = 0;   // Enable pullup on GPIO28
    //  GpioCtrlRegs.GPAQSEL2.bit.GPIO23 = 3; // Asynch input
    //  GpioCtrlRegs.GPAMUX2.bit.GPIO23 = 3;  // GPIO28 = SCIRXDA
    //  GpioCtrlRegs.GPAPUD.bit.GPIO22 = 0;   // Enable pullup on GPIO29
    //  GpioCtrlRegs.GPAMUX2.bit.GPIO22 = 3;  // GPIO29 = SCITXDA

    GpioCtrlRegs.GPAPUD.bit.GPIO24   = 1; // disable pull ups
    GpioCtrlRegs.GPAMUX2.bit.GPIO24  = 0; // choose GPIO for mux option
    GpioCtrlRegs.GPADIR.bit.GPIO24  = 1; // set as output
    GpioDataRegs.GPACLEAR.bit.GPIO24 = 1;

    GpioCtrlRegs.GPAPUD.bit.GPIO25   = 1; // disable pull ups
    GpioCtrlRegs.GPAMUX2.bit.GPIO25  = 0; // choose GPIO for mux option
    GpioCtrlRegs.GPADIR.bit.GPIO25  = 1; // set as output
    GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;


    GpioCtrlRegs.GPAPUD.bit.GPIO28   = 0; // disable pull ups
    GpioCtrlRegs.GPAMUX2.bit.GPIO28  = 1; // choose GPIO for mux option
    //  GpioCtrlRegs.GPADIR.bit.GPIO28  = 1; // set as output
    //GpioDataRegs.GPACLEAR.bit.GPIO28 = 1;

    GpioCtrlRegs.GPAPUD.bit.GPIO29   = 0; // disable pull ups
    GpioCtrlRegs.GPAMUX2.bit.GPIO29  = 1; // choose GPIO for mux option
    GpioCtrlRegs.GPAQSEL2.bit.GPIO28 = 3;
    // GpioCtrlRegs.GPADIR.bit.GPIO29  = 1; // set as output
    //GpioDataRegs.GPACLEAR.bit.GPIO29 = 1;

    GpioCtrlRegs.GPBPUD.bit.GPIO48   = 1; // enable pull ups
    GpioCtrlRegs.GPBMUX2.bit.GPIO48  = 0; // choose GPIO for mux option
    GpioCtrlRegs.GPBDIR.bit.GPIO48  = 0; // set as input
    //GpioDataRegs.GPBCLEAR.bit.GPIO48 = 1;
    GpioCtrlRegs.GPBQSEL2.bit.GPIO48 = 3; // Asynch input

    GpioCtrlRegs.GPBPUD.bit.GPIO49   = 1; // enable pull ups
    GpioCtrlRegs.GPBMUX2.bit.GPIO49  = 0; // choose GPIO for mux option
    GpioCtrlRegs.GPBDIR.bit.GPIO49  = 0; // set as input
    //GpioDataRegs.GPBCLEAR.bit.GPIO49 = 1;
    GpioCtrlRegs.GPBQSEL2.bit.GPIO49 = 3; // Asynch input

    GpioCtrlRegs.GPBPUD.bit.GPIO51   = 1; // disable pull ups
    GpioCtrlRegs.GPBMUX2.bit.GPIO51  = 0; // choose GPIO for mux option
    GpioCtrlRegs.GPBDIR.bit.GPIO51  = 1; // set as output
    // GpioDataRegs.GPBCLEAR.bit.GPIO51 = 1;

    GpioCtrlRegs.GPBPUD.bit.GPIO53   = 1; // disable pull ups
    GpioCtrlRegs.GPBMUX2.bit.GPIO53  = 0; // choose GPIO for mux option
    GpioCtrlRegs.GPBDIR.bit.GPIO53   = 1; // set as output
    GpioDataRegs.GPBSET.bit.GPIO53 = 1;

    GpioCtrlRegs.GPBPUD.bit.GPIO54   = 1; // disable pull ups
    GpioCtrlRegs.GPBMUX2.bit.GPIO54  = 0; // choose GPIO for mux option
    GpioCtrlRegs.GPBDIR.bit.GPIO54   = 1; // set as output
    GpioDataRegs.GPBCLEAR.bit.GPIO54 = 1;


    EDIS;

    // Map ISR functions
    //
    EALLOW;
    PieVectTable.ADCA1_INT = &adca1_isr; //function for ADCA interrupt 1
    PieVectTable.TIMER0_INT = &cpu_timer0_isr;
    PieCtrlRegs.PIEIER3.bit.INTx1 = 1;  // Enable PWM11INT in PIE group 3
    //  PieVectTable.SCIB_RX_INT=SCIB_RX_isr;
    PieVectTable.SCIA_RX_INT=SCIB_RX_isr;
    PieCtrlRegs.PIEIER9.bit.INTx1 = 1;
    // PieCtrlRegs.PIEIER9.bit.INTx3 = 1;
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;
    EDIS;

    //
    // Configure the ADC and power it up
    //
    ConfigureADC();
    EALLOW;


    AdcaRegs.ADCSOC0CTL.bit.CHSEL     = 0;   // A0
    AdcaRegs.ADCSOC0CTL.bit.ACQPS     = 30;   //
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL   = 5;   //

    AdcaRegs.ADCSOC1CTL.bit.CHSEL     = 1;    // A1
    AdcaRegs.ADCSOC1CTL.bit.ACQPS     = 30;   //
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL   = 5;    //

    AdcaRegs.ADCSOC2CTL.bit.CHSEL     = 2;    // A2
    AdcaRegs.ADCSOC2CTL.bit.ACQPS     = 30;
    AdcaRegs.ADCSOC2CTL.bit.TRIGSEL   = 5;

    AdcaRegs.ADCSOC3CTL.bit.CHSEL     = 3;    // A3
    AdcaRegs.ADCSOC3CTL.bit.ACQPS     = 30;
    AdcaRegs.ADCSOC3CTL.bit.TRIGSEL   = 5;

    AdcaRegs.ADCSOC4CTL.bit.CHSEL     = 4;   // A4
    AdcaRegs.ADCSOC4CTL.bit.ACQPS     = 30;   //
    AdcaRegs.ADCSOC4CTL.bit.TRIGSEL   = 5;   //

    AdcaRegs.ADCSOC5CTL.bit.CHSEL     = 5;   // A5
    AdcaRegs.ADCSOC5CTL.bit.ACQPS     = 30;   //
    AdcaRegs.ADCSOC5CTL.bit.TRIGSEL   = 5;   //

    AdcbRegs.ADCSOC0CTL.bit.CHSEL     = 0;   // B0
    AdcbRegs.ADCSOC0CTL.bit.ACQPS     = 30;   //
    AdcbRegs.ADCSOC0CTL.bit.TRIGSEL   = 5;   //

    AdcbRegs.ADCSOC1CTL.bit.CHSEL     = 1;    //B1
    AdcbRegs.ADCSOC1CTL.bit.ACQPS     = 30;   //
    AdcbRegs.ADCSOC1CTL.bit.TRIGSEL   = 5;    //

    AdcbRegs.ADCSOC2CTL.bit.CHSEL    = 2;     // B2
    AdcbRegs.ADCSOC2CTL.bit.ACQPS    = 30;    //
    AdcbRegs.ADCSOC2CTL.bit.TRIGSEL  = 5;     //

    AdcbRegs.ADCSOC3CTL.bit.CHSEL    = 3;     // B3
    AdcbRegs.ADCSOC3CTL.bit.ACQPS    = 30;    //
    AdcbRegs.ADCSOC3CTL.bit.TRIGSEL  = 5;     //


    AdcbRegs.ADCSOC4CTL.bit.CHSEL    = 4;     // B4
    AdcbRegs.ADCSOC4CTL.bit.ACQPS    = 30;    //
    AdcbRegs.ADCSOC4CTL.bit.TRIGSEL  = 5;     //

    AdcbRegs.ADCSOC5CTL.bit.CHSEL    = 4;     // B5
    AdcbRegs.ADCSOC5CTL.bit.ACQPS    = 30;    //
    AdcbRegs.ADCSOC5CTL.bit.TRIGSEL  = 5;     //

    AdcdRegs.ADCSOC0CTL.bit.CHSEL     = 0;    // D0
    AdcdRegs.ADCSOC0CTL.bit.ACQPS     = 30;
    AdcdRegs.ADCSOC0CTL.bit.TRIGSEL   = 5;
    // Setting up link from EPWM to ADC
    EPwm1Regs.ETSEL.bit.SOCASEL = 1;
    EPwm1Regs.ETPS.bit.SOCAPRD  = ET_1ST;     // Generate pulse on 1st even
    EPwm1Regs.ETSEL.bit.SOCAEN  = 1;          // Enable SOC on A group
    EPwm1Regs.ETCLR.bit.SOCA = 1;       /* Clear SOCA flag */
    EDIS;
    //
    // Initialize the temperature sensor
    // Note: The argument needs to change if using a VREFHI voltage other than 3.0V
    //
    //  InitTempSensor(3.0);

    //
    // Configure the ePWM
    //
    ConfigureEPWM();

    //
    // Setup the ADC for ePWM triggered conversions on temperature sensor
    //
    //SetupADCEpwm();

    //
    // Enable global Interrupts and higher priority real-time debug events:
    //

    IER |= M_INT3; // Enable group 3 interrupts
    IER |= M_INT1; // Enable group 1 interrupts
    IER |= M_INT9;
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

    //
    // enable PIE interrupt
    //

    scib_echoback_init();
    //
    // sync ePWM
    //
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;

    //
    // start ePWM
    //
    EPwm1Regs.ETSEL.bit.SOCAEN = 1;  //enable SOCA
    EPwm1Regs.TBCTL.bit.CTRMODE = 0; //unfreeze, and enter up count mode



    /* GpioCtrlRegs.GPAPUD.bit.GPIO23 = 0;   // Enable pullup on GPIO28
    GpioCtrlRegs.GPAQSEL2.bit.GPIO23 = 3; // Asynch input
    GpioCtrlRegs.GPAMUX2.bit.GPIO23 = 3;  // GPIO28 = SCIRXDA
    GpioCtrlRegs.GPAPUD.bit.GPIO22 = 0;   // Enable pullup on GPIO29
    GpioCtrlRegs.GPAMUX2.bit.GPIO22 = 3;  // GPIO29 = SCITXDA*/

    GpioCtrlRegs.GPAPUD.bit.GPIO24   = 1; // disable pull ups
    GpioCtrlRegs.GPAMUX2.bit.GPIO24  = 0; // choose GPIO for mux option
    GpioCtrlRegs.GPADIR.bit.GPIO24  = 1; // set as output
    GpioDataRegs.GPACLEAR.bit.GPIO24 = 1;

    GpioCtrlRegs.GPAPUD.bit.GPIO25   = 1; // disable pull ups
    GpioCtrlRegs.GPAMUX2.bit.GPIO25  = 0; // choose GPIO for mux option
    GpioCtrlRegs.GPADIR.bit.GPIO25  = 1; // set as output
    GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;



    GpioCtrlRegs.GPBPUD.bit.GPIO48   = 0; // enable pull ups
    GpioCtrlRegs.GPBMUX2.bit.GPIO48  = 0; // choose GPIO for mux option
    GpioCtrlRegs.GPBDIR.bit.GPIO48  = 0; // set as input
    GpioDataRegs.GPBCLEAR.bit.GPIO48 = 1;

    GpioCtrlRegs.GPBPUD.bit.GPIO49   = 0; // enable pull ups
    GpioCtrlRegs.GPBMUX2.bit.GPIO49  = 0; // choose GPIO for mux option
    GpioCtrlRegs.GPBDIR.bit.GPIO49  = 0; // set as input
    GpioDataRegs.GPBCLEAR.bit.GPIO49 = 1;

    GpioCtrlRegs.GPBPUD.bit.GPIO51   = 1; // disable pull ups
    GpioCtrlRegs.GPBMUX2.bit.GPIO51  = 0; // choose GPIO for mux option
    GpioCtrlRegs.GPBDIR.bit.GPIO51  = 1; // set as output
    // GpioDataRegs.GPBCLEAR.bit.GPIO51 = 1;

    GpioCtrlRegs.GPBPUD.bit.GPIO53   = 1; // disable pull ups
    GpioCtrlRegs.GPBMUX2.bit.GPIO53  = 0; // choose GPIO for mux option
    GpioCtrlRegs.GPBDIR.bit.GPIO53   = 1; // set as output
    GpioDataRegs.GPBCLEAR.bit.GPIO53 = 1;

    GpioCtrlRegs.GPBPUD.bit.GPIO54   = 1; // disable pull ups
    GpioCtrlRegs.GPBMUX2.bit.GPIO54  = 0; // choose GPIO for mux option
    GpioCtrlRegs.GPBDIR.bit.GPIO54   = 1; // set as output
    GpioDataRegs.GPBCLEAR.bit.GPIO54 = 1;

    GpioDataRegs.GPBSET.bit.GPIO51 = 1;
    GpioDataRegs.GPBCLEAR.bit.GPIO53 = 1;
    GpioDataRegs.GPBCLEAR.bit.GPIO54 = 1;

    EDIS;
    /*
   Stop();

   GpioDataRegs.GPACLEAR.bit.GPIO24 = 1;
   GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;
   Stop();

   GpioDataRegs.GPASET.bit.GPIO24 = 1;
   GpioDataRegs.GPASET.bit.GPIO25 = 1;
   Stop();

   GpioDataRegs.GPACLEAR.bit.GPIO24 = 1;
   GpioDataRegs.GPASET.bit.GPIO25 = 1;
   Stop();

   GpioDataRegs.GPASET.bit.GPIO24 = 1;
   GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;
   Stop();
     */


    ///------------ADIS16488-------------
    EALLOW;

    GpioCtrlRegs.GPBMUX1.bit.GPIO35 = 0;  // GPIO25 = GPIO25
    GpioCtrlRegs.GPBDIR.bit.GPIO35 = 0;   // GPIO25 = input
    GpioCtrlRegs.GPBQSEL1.bit.GPIO35 = 0;        // XINT1 Synch to SYSCLKOUT only

    GpioCtrlRegs.GPBMUX1.bit.GPIO36 = 0;  // GPIO25 = GPIO25
    GpioCtrlRegs.GPBDIR.bit.GPIO36 = 0;   // GPIO25 = input
    GpioCtrlRegs.GPBQSEL1.bit.GPIO36 = 0;     // XINT1 Synch to SYSCLKOUT only

    GpioCtrlRegs.GPAPUD.bit.GPIO6   = 1; // disable pull ups
    GpioCtrlRegs.GPAMUX1.bit.GPIO6  = 0; // choose GPIO for mux option
    GpioCtrlRegs.GPADIR.bit.GPIO6   = 1; // set as output
    GpioDataRegs.GPASET.bit.GPIO6 = 1;

    EDIS;

    InitSpiaGpio();
    spi_fifo_init();     // Initialize the SPI FIFO
    ///------------ADIS16488-------------



    Stop();

    My_Komand1.Arrietir = Arr_Off;  // по умолчанию снимаем с арретира???

    CpuTimer0Regs.TCR.bit.TSS = 0;  // start timer0 - запускаем таймер времени с дискретом 1мс

    CurrentTime = CpuTimer0.InterruptCount;
    while ((CpuTimer0.InterruptCount - CurrentTime)<100); //задержка 100мс
    GpioDataRegs.GPBCLEAR.bit.GPIO51 = 1;//interface //подача питания на видеоплату и плату интерфейса
    CurrentTime = CpuTimer0.InterruptCount;
    while ((CpuTimer0.InterruptCount - CurrentTime)<300);
    GpioDataRegs.GPBSET.bit.GPIO53 = 1;//верхний
    CurrentTime = CpuTimer0.InterruptCount;
    while ((CpuTimer0.InterruptCount - CurrentTime)<300);
    GpioDataRegs.GPBSET.bit.GPIO54 = 1;//нижний
    CurrentTime = CpuTimer0.InterruptCount;
    while ((CpuTimer0.InterruptCount - CurrentTime)<300);


    ///------------ADIS16488-------------
    sdata = 0x8003;
    spi_xmit(sdata);
    while(SpiaRegs.SPIFFRX.bit.RXFFST !=1) { }
    rdata = SpiaRegs.SPIRXBUF;

    CurrentTime = CpuTimer0.InterruptCount;
    do
    {
        sdata = 0x0000;
        spi_xmit(sdata);
        while(SpiaRegs.SPIFFRX.bit.RXFFST !=1) { }
        rdata = SpiaRegs.SPIRXBUF;
    }while ((rdata!=3)&&((CpuTimer0.InterruptCount - CurrentTime)<200));

    //sdata = 0x8C17; //100 Гц(2460/(23+1))  для ADIS16488
    sdata = 0x8C2A;  ///100Гц (4250/(42+1))  для ADIS16497
    spi_xmit(sdata);
    while(SpiaRegs.SPIFFRX.bit.RXFFST !=1) { }
    rdata = SpiaRegs.SPIRXBUF;
    delay_loop();
    //sdata = 0x8D03;
    sdata = 0x8D00;
    spi_xmit(sdata);
    while(SpiaRegs.SPIFFRX.bit.RXFFST !=1) { }
    rdata = SpiaRegs.SPIRXBUF;
    delay_loop();

    sdata = 0x8000;
    spi_xmit(sdata);
    while(SpiaRegs.SPIFFRX.bit.RXFFST !=1) { }
    rdata = SpiaRegs.SPIRXBUF;

    CurrentTime = CpuTimer0.InterruptCount;
    do
    {
        sdata = 0x0000;
        spi_xmit(sdata);
        while(SpiaRegs.SPIFFRX.bit.RXFFST !=1) { }
        rdata = SpiaRegs.SPIRXBUF;
    }while ((rdata!=0)&&((CpuTimer0.InterruptCount - CurrentTime)<200));

    Xint1CountOld = Xint1Count;
    ///------------ADIS16488-------------

    while(1)
    {
        ///работа по прерыванию без BRF
        if (Xint1Count != Xint1CountOld)
        {
            if (abs(Xint1Count - Xint1CountOld)>1) ErrorOfCount_DATA_CNT++;
            Xint1CountOld = Xint1Count;

            sdata = 0x1200;
            spi_xmit(sdata);
            while(SpiaRegs.SPIFFRX.bit.RXFFST !=1) { }
            rdata = SpiaRegs.SPIRXBUF;
            ////delay_loop();
            //sdata = 0x8D03;
            sdata = 0x1000;
            spi_xmit(sdata);
            while(SpiaRegs.SPIFFRX.bit.RXFFST !=1) { }
            rdata = SpiaRegs.SPIRXBUF;
            ////delay_loop();
            ///BRFData->X_GYRO_OUT = rdata;

            sdata = 0x1600;
            spi_xmit(sdata);
            while(SpiaRegs.SPIFFRX.bit.RXFFST !=1) { }
            rdata = SpiaRegs.SPIRXBUF;
            ////delay_loop();
            BRFData->X_GYRO_OUT = rdata;
            sdata = 0x1400;
            spi_xmit(sdata);
            while(SpiaRegs.SPIFFRX.bit.RXFFST !=1) { }
            rdata = SpiaRegs.SPIRXBUF;
            ////delay_loop();
            BRFData->X_GYRO_LOW = rdata;

            sdata = 0x1A00;
            spi_xmit(sdata);
            while(SpiaRegs.SPIFFRX.bit.RXFFST !=1) { }
            rdata = SpiaRegs.SPIRXBUF;
            ////delay_loop();
            BRFData->Y_GYRO_OUT = rdata;
            sdata = 0x1800;
            spi_xmit(sdata);
            while(SpiaRegs.SPIFFRX.bit.RXFFST !=1) { }
            rdata = SpiaRegs.SPIRXBUF;
            ////delay_loop();
            BRFData->Y_GYRO_LOW = rdata;

            sdata = 0x0E00;
            spi_xmit(sdata);
            while(SpiaRegs.SPIFFRX.bit.RXFFST !=1) { }
            rdata = SpiaRegs.SPIRXBUF;
            ////delay_loop();
            BRFData->Z_GYRO_OUT = rdata;
            sdata = 0x2800;
            spi_xmit(sdata);
            while(SpiaRegs.SPIFFRX.bit.RXFFST !=1) { }
            rdata = SpiaRegs.SPIRXBUF;
            ////delay_loop();
            BRFData->Z_GYRO_LOW = rdata;
            sdata = 0x2900;
            spi_xmit(sdata);
            while(SpiaRegs.SPIFFRX.bit.RXFFST !=1) { }
            rdata = SpiaRegs.SPIRXBUF;
            ////delay_loop();
            BRFData->TEMP_OUT = rdata;
            TempOfADIS = BRFData->TEMP_OUT/177 + 25;

            sdata = 0x3000;
            spi_xmit(sdata);
            while(SpiaRegs.SPIFFRX.bit.RXFFST !=1) { }
            rdata = SpiaRegs.SPIRXBUF;
            ////delay_loop();
            BRFData->DATA_CNT = rdata;
            sdata = 0x2E00;
            spi_xmit(sdata);
            while(SpiaRegs.SPIFFRX.bit.RXFFST !=1) { }
            rdata = SpiaRegs.SPIRXBUF;
            ////delay_loop();
            ///BRFData->Y_GYRO_LOW = rdata;

            sdata = 0x3000;
            spi_xmit(sdata);
            while(SpiaRegs.SPIFFRX.bit.RXFFST !=1) { }
            rdata = SpiaRegs.SPIRXBUF;
            ////delay_loop();
            //BRFData->CRC_UPR = rdata;
            PressureInPa = rdata;
            //PressureInPa = (int32)rdata*4;
            sdata = 0x2E00;
            spi_xmit(sdata);
            while(SpiaRegs.SPIFFRX.bit.RXFFST !=1) { }
            rdata = SpiaRegs.SPIRXBUF;
            BRFData->CRC_LWR = rdata;
            ////delay_loop();
            ///BRFData->Y_GYRO_LOW = rdata;


            X_GyroInt32 = ((int32)((((Uint32)BRFData->X_GYRO_OUT)<<16)+((Uint32)BRFData->X_GYRO_LOW)));
            Y_GyroInt32 = ((int32)((((Uint32)BRFData->Y_GYRO_OUT)<<16)+((Uint32)BRFData->Y_GYRO_LOW)));
            Z_GyroInt32 = ((int32)((((Uint32)BRFData->Z_GYRO_OUT)<<16)+((Uint32)BRFData->Z_GYRO_LOW)));

            //для ADIS16488
            DeltaXSek = DeltaXSek + X_GyroInt32/910/100;   //0.02/65536(ед. разряда в градусах), 0.02/65536*60*60 (ед. разряда в секундах)  /100 - частота   1/число = ед.разряда
            DeltaYSek = DeltaYSek + Y_GyroInt32/910/100;
            DeltaZSek = DeltaZSek + Z_GyroInt32/910/100;

            X_Gyro = ((int32)((((Uint32)BRFData->X_GYRO_OUT)<<16)+((Uint32)BRFData->X_GYRO_LOW)))/910/60;
            Y_Gyro = ((int32)((((Uint32)BRFData->Y_GYRO_OUT)<<16)+((Uint32)BRFData->Y_GYRO_LOW)))/910/60;
            Z_Gyro = ((int32)((((Uint32)BRFData->Z_GYRO_OUT)<<16)+((Uint32)BRFData->Z_GYRO_LOW)))/910/60;



            //для ADIS16497
            //DeltaXSek = DeltaXSek + X_GyroInt32/728/200;   //0.025/65536(ед. разряда в градусах), 0.025/65536*60*60 (ед. разряда в секундах)
            //DeltaYSek = DeltaYSek + Y_GyroInt32/728/200;
            //DeltaZSek = DeltaZSek + Z_GyroInt32/728/200;

            //X_Gyro = ((int32)((((Uint32)BRFData->X_GYRO_OUT)<<16)+((Uint32)BRFData->X_GYRO_LOW)))/728/60;
            //Y_Gyro = ((int32)((((Uint32)BRFData->Y_GYRO_OUT)<<16)+((Uint32)BRFData->Y_GYRO_LOW)))/728/60;
            //Z_Gyro = ((int32)((((Uint32)BRFData->Z_GYRO_OUT)<<16)+((Uint32)BRFData->Z_GYRO_LOW)))/728/60;


            My_Otvet32.BAROM_OUT_Hi = (PressureInPa>>8) & 0xFF;
            My_Otvet32.BAROM_OUT_Lo = PressureInPa & 0xFF;

            My_Otvet32.TEMP_OUT_Hi = (BRFData->TEMP_OUT>>8) & 0xFF;
            My_Otvet32.TEMP_OUT_Lo = BRFData->TEMP_OUT & 0xFF;

            My_Otvet32.X_GYRO_OUT_Hi = (BRFData->X_GYRO_OUT>>8) & 0xFF;
            My_Otvet32.X_GYRO_OUT_Lo = BRFData->X_GYRO_OUT & 0xFF;
            My_Otvet32.X_GYRO_LOW_Hi = (BRFData->X_GYRO_LOW>>8) & 0xFF;
            My_Otvet32.X_GYRO_LOW_Lo = BRFData->X_GYRO_LOW & 0xFF;

            My_Otvet32.Y_GYRO_OUT_Hi = (BRFData->Y_GYRO_OUT>>8) & 0xFF;
            My_Otvet32.Y_GYRO_OUT_Lo = BRFData->Y_GYRO_OUT & 0xFF;
            My_Otvet32.Y_GYRO_LOW_Hi = (BRFData->Y_GYRO_LOW>>8) & 0xFF;
            My_Otvet32.Y_GYRO_LOW_Lo = BRFData->Y_GYRO_LOW & 0xFF;

            My_Otvet32.Z_GYRO_OUT_Hi = (BRFData->Z_GYRO_OUT>>8) & 0xFF;
            My_Otvet32.Z_GYRO_OUT_Lo = BRFData->Z_GYRO_OUT & 0xFF;
            My_Otvet32.Z_GYRO_LOW_Hi = (BRFData->Z_GYRO_LOW>>8) & 0xFF;
            My_Otvet32.Z_GYRO_LOW_Lo = BRFData->Z_GYRO_LOW & 0xFF;
        }

        if (Flags.Switch_Otvet_COM32)
        {
            Flags.Switch_Otvet_COM32 = 0;
            SwitchKomand32_Rx();
            SwitchKomand32_Tx();

            switch(Komand_Com32.Komand_Arr)
            {
            case ARR_open:
                My_Komand1.Arrietir = Arr_Off;
                FirstKommandToArretirIsHappened = true;
                break;
            case ARR_close:
                My_Komand1.Arrietir = Arr_On;
                FirstKommandToArretirIsHappened = true;
                break;
            case ARR_do_nothing:
                break;
            case Drv_On_Nignii:
                GpioDataRegs.GPBSET.bit.GPIO54 = 1;
                break;
            case Drv_Off_Nignii:
                // GpioDataRegs.GPBCLEAR.bit.GPIO54 = 1;
                break;
            case Drv_On_verhniy:
                GpioDataRegs.GPBSET.bit.GPIO53 = 1;
                break;
            case Drv_Off_verhniy:
                //GpioDataRegs.GPBCLEAR.bit.GPIO53 = 1;
                break;
            }

        }

        if (FirstKommandToArretirIsHappened) Arrietir(My_Komand1.Arrietir, &My_Otvet32);

        if(GpioDataRegs.GPBDAT.bit.GPIO53==1){My_Otvet32.Status_Arr=(My_Otvet32.Status_Arr&0x0F)|0x20;}
        if(GpioDataRegs.GPBDAT.bit.GPIO53==0){My_Otvet32.Status_Arr=My_Otvet32.Status_Arr&0x1F;}
        if(GpioDataRegs.GPBDAT.bit.GPIO54==1){My_Otvet32.Status_Arr=(My_Otvet32.Status_Arr&0x2F)|0x10;}
        if(GpioDataRegs.GPBDAT.bit.GPIO54==0){My_Otvet32.Status_Arr&=0x2F;}

        AppCounter++;
        //if (!(AppCounter % 1000000)) printf("Количество прерываний таймера = %ld \n",  CpuTimer0.InterruptCount);
        //if (!(CpuTimer0.InterruptCount % 1000)) printf("Количество прерываний таймера = %ld \n",  CpuTimer0.InterruptCount);
    }
}



interrupt void cpu_timer0_isr(void)
{
    CpuTimer0.InterruptCount++;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

interrupt void SCIB_RX_isr(void)
{
    Bukva = SciaRegs.SCIRXBUF.all;
    SCIB_RX_32(Bukva, My_Com_32.Start1,  My_Com_32.Start2,  My_Com_32.finish);
    PieCtrlRegs.PIEACK.all |= 0x100;
}

//
// adca1_isr - Read Temperature ISR
//

unsigned int adcTimer = 0;
unsigned int average = 0;

interrupt void adca1_isr(void)
{
    average = AdcaResultRegs.ADCRESULT0;
    //  sensorSample = AdcaResultRegs.ADCRESULT0;
    //  sensorTemp = GetTemperatureC(sensorSample);
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}


void SCIB_RX_32(Uint16 bukva,unsigned char start1, unsigned char start2, unsigned char finish)
{
    unsigned char bukva1;
    unsigned char i;
    My_Com_32.Bukva=bukva;
    if(bukva==start1)
    {
        bukva1=start1;
    }
    if(bukva==start2)
    {
        if(bukva1==start1)
        {
            if (My_Com_32.u>1)
            {
                bukva1=0;
            }
            else
            {
                My_Com_32.flagMess=1; My_Com_32.u=1;bukva1=0;
            }
        }

        else{bukva1=0;}
    }
    if(My_Com_32.flagMess==1)
    {
        if (My_Com_32.u==1)
        {
            My_Com_32.Sklad[0]=start1;
            My_Com_32.Sklad[1]=start2;
            bukva1=0;
            My_Com_32.u+=1;
        }
        else
        {
            My_Com_32.Sklad[ My_Com_32.u]=My_Com_32.Bukva;

            if (My_Com_32.Sklad[ My_Com_32.u]==start1)
            {
                if (My_Com_32.Sklad[ My_Com_32.u-1]!=1){bukva1=0;}
            }
            My_Com_32.u+=1;
        }
    }
    if(My_Com_32.u==32)
    {
        My_Com_32.u=0;
        My_Com_32.flagMess=0;
        My_Com_32.Bukva=0;
        bukva1=0;
        My_Com_32.flagViv=0;
        My_Com_32.checksum=My_Com_32.Sklad[0];

        //контрольная сумма
        for (My_Com_32.index=1;My_Com_32.index<(32-1);My_Com_32.index++)
        {
            My_Com_32.checksum=My_Com_32.checksum^My_Com_32.Sklad[My_Com_32.index];
        }

        if((My_Com_32.Sklad[32-1])==(My_Com_32.checksum))
        {

            for (i=0;i<32;i++){My_Com_32.message1[i]=My_Com_32.Sklad[i];}
            Flags.Switch_Otvet_COM32=1;
        }
    }
}

void My_Com_Tx32(Uint8 lenght, Uint8 *mass)
{
    int i;
    for (i=0;i<lenght;i++)
    {
        while ( SciaRegs.SCICTL2.bit.TXEMPTY == 0);
        SciaRegs.SCITXBUF.all = mass[i];
    }
}


void SwitchKomand32_Rx(void)
{
    switch(My_Com_32.message1[2]) //код команды
    {
    case 0xA3:
        Komand_Com32.Komand_Pitanie = My_Com_32.message1[3];
        Komand_Com32.Komand_Arr = My_Com_32.message1[4];
        Komand_Com32.Tek_Regim_Raboti = My_Com_32.message1[5];
        Komand_Com32.Mems_Regim = My_Com_32.message1[6];
        Komand_Com32.reserv1 = My_Com_32.message1[7];
        Komand_Com32.reserv2 = My_Com_32.message1[8];
        Komand_Com32.reserv3 = My_Com_32.message1[9];
        Komand_Com32.reserv4 = My_Com_32.message1[10];
        Komand_Com32.reserv5 = My_Com_32.message1[11];
        Komand_Com32.reserv6 = My_Com_32.message1[12];
        Komand_Com32.reserv7 = My_Com_32.message1[13];
        Komand_Com32.reserv8 = My_Com_32.message1[14];
        Komand_Com32.reserv9 = My_Com_32.message1[15];
        Komand_Com32.reserv10 = My_Com_32.message1[16];
        Komand_Com32.reserv11 = My_Com_32.message1[17];
        Komand_Com32.reserv12 = My_Com_32.message1[18];
        Komand_Com32.reserv13 = My_Com_32.message1[19];
        Komand_Com32.reserv14 = My_Com_32.message1[20];
        Komand_Com32.reserv15 = My_Com_32.message1[21];
        Komand_Com32.reserv16 = My_Com_32.message1[22];
        Komand_Com32.reserv17 = My_Com_32.message1[23];
        Komand_Com32.reserv18 = My_Com_32.message1[24];
        Komand_Com32.reserv19 = My_Com_32.message1[25];
        Komand_Com32.reserv20 = My_Com_32.message1[26];
        Komand_Com32.reserv21 = My_Com_32.message1[27];
        Komand_Com32.reserv22 = My_Com_32.message1[28];
        Komand_Com32.reserv23 = My_Com_32.message1[29];
        break;
    case 0xA2:
        break;
    default:
        break;
    }
}


void SwitchKomand32_Tx(void)
{
    unsigned char i;
    My_Com_32_Tx[0]=My_Com_32.Start1;
    My_Com_32_Tx[1]=0xA3;
    My_Com_32_Tx[2]=My_Com_32.Start2;
    My_Com_32_Tx[3]=0;
    My_Com_32_Tx[4]=My_Otvet32.Status_Arr;

    My_Com_32_Tx[10] = My_Otvet32.X_GYRO_OUT_Hi;
    My_Com_32_Tx[11] = My_Otvet32.X_GYRO_OUT_Lo;
    My_Com_32_Tx[12] = My_Otvet32.X_GYRO_LOW_Hi;
    My_Com_32_Tx[13] = My_Otvet32.X_GYRO_LOW_Lo;
    My_Com_32_Tx[14] = My_Otvet32.Y_GYRO_OUT_Hi;
    My_Com_32_Tx[15] = My_Otvet32.Y_GYRO_OUT_Lo;
    My_Com_32_Tx[16] = My_Otvet32.Y_GYRO_LOW_Hi;
    My_Com_32_Tx[17] = My_Otvet32.Y_GYRO_LOW_Lo;
    My_Com_32_Tx[18] = My_Otvet32.Z_GYRO_OUT_Hi;
    My_Com_32_Tx[19] = My_Otvet32.Z_GYRO_OUT_Lo;
    My_Com_32_Tx[20] = My_Otvet32.Z_GYRO_LOW_Hi;
    My_Com_32_Tx[21] = My_Otvet32.Z_GYRO_LOW_Lo;

    My_Com_32_Tx[22] = My_Otvet32.TEMP_OUT_Hi;
    My_Com_32_Tx[23] = My_Otvet32.TEMP_OUT_Lo;
    My_Com_32_Tx[24] = My_Otvet32.BAROM_OUT_Hi;
    My_Com_32_Tx[25] = My_Otvet32.BAROM_OUT_Lo;


    /*
        My_Com_32_Tx[34] = My_Otvet32.TEMP_OUT_Hi;
        My_Com_32_Tx[35] = My_Otvet32.TEMP_OUT_Lo;
        My_Com_32_Tx[36] = My_Otvet32.BAROM_OUT_Hi;
        My_Com_32_Tx[37] = My_Otvet32.BAROM_OUT_Lo;
     */

    My_Com_32_Tx[30] = My_Com_32.finish;
    My_Com_32_Tx[31]=My_Com_32_Tx[0];
    for (i=1;i<(32-1);i++) { My_Com_32_Tx[31]= My_Com_32_Tx[31]^My_Com_32_Tx[i];}
    My_Com_Tx32(32, My_Com_32_Tx);

    /*
      My_Com_32_Tx[62] = My_Com_32.finish;
      My_Com_32_Tx[63]=My_Com_32_Tx[0];
      for (i=1;i<(64-1);i++) { My_Com_32_Tx[63] ^= My_Com_32_Tx[i];}
      My_Com_Tx32(64, My_Com_32_Tx);
     */
}
