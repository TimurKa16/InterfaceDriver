#include "F28x_Project.h"

#define RESOLUTION_12BIT   0 //12-bit resolution
#define RESOLUTION_16BIT   1 //16-bit resolution (not supported for all variants)

// Function Prototypes
#define SIGNAL_SINGLE          0 //single-ended channel conversions (12-bit mode only)
#define SIGNAL_DIFFERENTIAL    1 //differential pair channel conversions

void scib_echoback_init(void)
{

       SciaRegs.SCICCR.all = 0x0007;   // 1 stop bit,  No loopback
                                        // No parity,8 char bits,
                                        // async mode, idle-line protocol
        SciaRegs.SCICTL1.all = 0x0003;  // enable TX, RX, internal SCICLK,
        SciaRegs.SCICTL2.bit.RXBKINTENA = 1;
        SciaRegs.SCIHBAUD.all = 0x0000;
        SciaRegs.SCILBAUD.all = 0x0036;
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
void ConfigureEPWM(void)
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
