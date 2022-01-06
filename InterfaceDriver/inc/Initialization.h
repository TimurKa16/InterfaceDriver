
#include "main2.h"
#include "DeviceNumber.h"



void InitI2c(void)
{
    I2caRegs.I2CMDR.bit.IRS = 0;    // Reset the I2C module
    // I2C slave address register
    I2caRegs.I2CSAR.all = TMP100_SLAVE;
    // I2C Prescale Register
    I2caRegs.I2CPSC.all = 14;       // Internal I2C module clock = SYSCLK/(PSC +1)
    // = 10 MHz

    I2caRegs.I2CCLKL = 95;          // Tmaster = (PSC +1)[ICCL + 5 + ICCH + 5] / 150MHz
    I2caRegs.I2CCLKH = 95;          // Tmaster =  15 [ICCL + ICCH + 10] / 150 MHz

    I2caRegs.I2CMDR.bit.IRS = 1;    // Take I2C out of reset
}

void InitInterrupts()
{
    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.XINT1_INT = &xint1_isr;
    EDIS;    // This is needed to disable write to EALLOW protected registers

    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;          // Enable the PIE block
    PieCtrlRegs.PIEIER1.bit.INTx4 = 1;          // Enable PIE Group 1 INT4
    PieCtrlRegs.PIEIER1.bit.INTx5 = 1;          // Enable PIE Group 1 INT5
    IER |= M_INT1;                              // Enable CPU INT1
    //    EINT;                                       // Enable Global Interrupts
    // Map ISR functions
    //
    EALLOW;
    PieVectTable.ADCA1_INT = &adca1_isr; //function for ADCA interrupt 1
    PieVectTable.TIMER0_INT = &CpuTimerIsr;
    PieCtrlRegs.PIEIER3.bit.INTx1 = 1;  // Enable PWM11INT in PIE group 3
    PieVectTable.SCIA_RX_INT=SciRxIsr;
    PieCtrlRegs.PIEIER9.bit.INTx1 = 1;
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;
    EDIS;

    IER |= M_INT3; // Enable group 3 interrupts
    IER |= M_INT1; // Enable group 1 interrupts
    IER |= M_INT9;
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    EDIS;
}

void InitEveryGpioWeNeed()
{
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

    GpioCtrlRegs.GPAPUD.bit.GPIO24   = 1; // disable pull ups
    GpioCtrlRegs.GPAMUX2.bit.GPIO24  = 0; // choose GPIO for mux option
    GpioCtrlRegs.GPADIR.bit.GPIO24  = 1; // set as output
    GpioDataRegs.GPASET.bit.GPIO24 = 1;

    GpioCtrlRegs.GPAPUD.bit.GPIO25   = 1; // disable pull ups
    GpioCtrlRegs.GPAMUX2.bit.GPIO25  = 0; // choose GPIO for mux option
    GpioCtrlRegs.GPADIR.bit.GPIO25  = 1; // set as output
    GpioDataRegs.GPASET.bit.GPIO25 = 1;


    GpioCtrlRegs.GPAPUD.bit.GPIO28   = 0; // disable pull ups
    GpioCtrlRegs.GPAMUX2.bit.GPIO28  = 1; // choose GPIO for mux option

    GpioCtrlRegs.GPAPUD.bit.GPIO29   = 0; // disable pull ups
    GpioCtrlRegs.GPAMUX2.bit.GPIO29  = 1; // choose GPIO for mux option
    GpioCtrlRegs.GPAQSEL2.bit.GPIO28 = 3;

    GpioCtrlRegs.GPBPUD.bit.GPIO48   = 1; // enable pull ups
    GpioCtrlRegs.GPBMUX2.bit.GPIO48  = 0; // choose GPIO for mux option
    GpioCtrlRegs.GPBDIR.bit.GPIO48  = 0; // set as input
    GpioCtrlRegs.GPBQSEL2.bit.GPIO48 = 3; // Asynch input

    GpioCtrlRegs.GPBPUD.bit.GPIO49   = 1; // enable pull ups
    GpioCtrlRegs.GPBMUX2.bit.GPIO49  = 0; // choose GPIO for mux option
    GpioCtrlRegs.GPBDIR.bit.GPIO49  = 0; // set as input
    GpioCtrlRegs.GPBQSEL2.bit.GPIO49 = 3; // Asynch input

    GpioCtrlRegs.GPBPUD.bit.GPIO51   = 1; // disable pull ups
    GpioCtrlRegs.GPBMUX2.bit.GPIO51  = 0; // choose GPIO for mux option
    GpioCtrlRegs.GPBDIR.bit.GPIO51  = 1; // set as output

    GpioCtrlRegs.GPBPUD.bit.GPIO52   = 1; // disable pull ups
    GpioCtrlRegs.GPBMUX2.bit.GPIO52  = 0; // choose GPIO for mux option
    GpioCtrlRegs.GPBDIR.bit.GPIO52  = 1; // set as output
    GpioDataRegs.GPBCLEAR.bit.GPIO52 = 1;

    GpioCtrlRegs.GPBPUD.bit.GPIO53   = 1; // disable pull ups
    GpioCtrlRegs.GPBMUX2.bit.GPIO53  = 0; // choose GPIO for mux option
    GpioCtrlRegs.GPBDIR.bit.GPIO53   = 1; // set as output
    GpioDataRegs.GPBCLEAR.bit.GPIO53 = 1;

    GpioCtrlRegs.GPBPUD.bit.GPIO54   = 1; // disable pull ups
    GpioCtrlRegs.GPBMUX2.bit.GPIO54  = 0; // choose GPIO for mux option
    GpioCtrlRegs.GPBDIR.bit.GPIO54   = 1; // set as output
    GpioDataRegs.GPBCLEAR.bit.GPIO54 = 1;

    GpioCtrlRegs.GPBPUD.bit.GPIO55   = 1; // disable pull ups
    GpioCtrlRegs.GPBMUX2.bit.GPIO55  = 0; // choose GPIO for mux option
    GpioCtrlRegs.GPBDIR.bit.GPIO55   = 1; // set as output
    GpioDataRegs.GPBCLEAR.bit.GPIO55 = 1;

    EDIS;

    //------**TMP100
    EALLOW;

    GpioCtrlRegs.GPBMUX1.all = 0;           // GPIO47 ... GPIO32 = General Purpose I/O
    GpioCtrlRegs.GPBMUX1.bit.GPIO32 = 1;    // GPIO32 = I2C - SDA
    GpioCtrlRegs.GPBMUX1.bit.GPIO33 = 1;    // GPIO33 = I2C - SCL

    GpioCtrlRegs.GPBPUD.bit.GPIO32 = 0;    // Enable pull-up for GPIO32 (SDAA)
    GpioCtrlRegs.GPBPUD.bit.GPIO33 = 0;    // Enable pull-up for GPIO33 (SCLA)

    GpioCtrlRegs.GPBQSEL1.bit.GPIO32 = 3;  // Asynch input GPIO32 (SDAA)
    GpioCtrlRegs.GPBQSEL1.bit.GPIO33 = 3;  // Asynch input GPIO33 (SCLA)

    EDIS;
    //------**TMP100

    GpioDataRegs.GPBCLEAR.bit.GPIO51 = 1;
    GpioDataRegs.GPBCLEAR.bit.GPIO52 = 1;
    GpioDataRegs.GPBCLEAR.bit.GPIO53 = 1;
    GpioDataRegs.GPBCLEAR.bit.GPIO54 = 1;
    GpioDataRegs.GPBCLEAR.bit.GPIO55 = 1;
}

void InitXint()
{
    EALLOW;
    InputXbarRegs.INPUT4SELECT = 35;      //Set XINT1 source to GPIO-pin
    EDIS;

    XintRegs.XINT1CR.bit.POLARITY = 1;
    XintRegs.XINT1CR.bit.ENABLE = 1;            // Enable XINT1
}

void InitAdc()
{

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

    AdcbRegs.ADCSOC5CTL.bit.CHSEL    = 5;     // B5
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

    EALLOW;
    //
    // start ePWM
    //
    EPwm1Regs.ETSEL.bit.SOCAEN = 1;  //enable SOCA
    EPwm1Regs.TBCTL.bit.CTRMODE = 0; //unfreeze, and enter up count mode

    EDIS;
}

void spi_fifo_init()
{
    SpiaRegs.SPIFFTX.all = 0xE040;
    SpiaRegs.SPIFFRX.all = 0x2044;
    SpiaRegs.SPIFFCT.all = 0x0;

    InitSpi();
}

void InitSpia()
{
    InitSpiaGpio();
    spi_fifo_init();
}

void InitSystem()
{
    InitSysCtrl();
#ifdef _FLASH
    // Call Flash Initialization to setup flash waitstates
    // This function must reside in RAM
    InitFlash();
#endif

    InitPieCtrl();

    IER = 0x0000;
    IFR = 0x0000;

    InitPieVectTable();

    InitCpuTimers();    // basic setup CPU Timer0, 1 and 2

    ///ConfigCpuTimer(&CpuTimer0, 150, DELAY); // CPU - Timer0 at 100 milliseconds
    ConfigCpuTimer(&CpuTimer0, 200, 1000); // CPU - Timer0 точность 1 мс
    InitGpio(); // Skipped for this example
}
