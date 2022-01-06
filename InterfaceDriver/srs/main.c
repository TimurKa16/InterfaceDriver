// Обновление:
// Если посылка SCI не приходит 5 секунд, устройство рестартует

#include "Initialization.h"




char sciIsReceived = 0;

void main(void)
{
    InitVariables();
    InitSystem();
    InitInterrupts();
    InitEveryGpioWeNeed();
    InitXint();
    InitI2c();
    InitEpwm();
    InitAdc();
    InitSci();
    InitSpia();
    StartCpuTimer();

    Stop();

    sciIsReceived = 0;
    while (!sciIsReceived)
        CheckSci();

    ReadTmp100();
    StartDrivers();
    StartMems();

    while(1)
    {
        ReadMems();

        CheckHeat();
        CheckArretirCommand();
        CheckSci();
        CheckArretirStatus();
        CheckTmp100();

    }
}


// Прерывания

interrupt void CpuTimerIsr(void)
{
    timer.arretirTimer++;
    timer.tmpTimer++;
    timer.ds1624Timer++;
    CpuTimer0.InterruptCount++;
    timer.cpuTimer++;
    timer.heatTimer++;
    timer.workingTimer++;
    memsTimer++;

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

// TODO SciRxIsr
interrupt void SciRxIsr(void)
{
    sciIsReceived = 1;

    sciRxData = SciaRegs.SCIRXBUF.all;
    SciRx(sciRxData);

    PieCtrlRegs.PIEACK.all |= 0x100;
}

interrupt void adca1_isr(void)
{
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}






void InitVariables(void)
{
    sciTxCommand.arretirStatus = ArretirStatus.Closed;
    xintCount = 0;
    sciRxCommand.arretirCommand = ArretirCommand.Stop;
}




void SciRx(Uint16 sciRxBuffer)
{
    unsigned char i;
    unsigned char checksum = 0;

    if ((sciRxBufferArray[1] == SciCommand.HisAddress) && (sciRxBufferArray[0] == SciCommand.Beginning))
    {
        if (sciRxBufferCounter < (SciCommand.MessageLength - 1))
        {
            sciRxBufferArray[sciRxBufferCounter] = sciRxBuffer;
            sciRxBufferCounter++;
        }
        else if (sciRxBufferCounter == SciCommand.MessageLength - 1)
        {
            sciRxBufferArray[sciRxBufferCounter] = sciRxBuffer;

            if (sciRxBufferArray[SciCommand.MessageLength - 2] == SciCommand.End)
            {
                checksum = 0;
                for (i = 0; i < SciCommand.MessageLength - 1; i++)
                    checksum = sciRxBufferArray[i] ^ checksum;

                if (sciRxBufferArray[SciCommand.MessageLength - 1] == checksum)
                {
                    for (i = 0; i < SciCommand.MessageLength; i++)
                        sciRxArray[i] = sciRxBufferArray[i];
                    sciRxCommand.rxMassageIsCorrect = 1;
                }
            }

            for (i = 0; i < SciCommand.MessageLength; i++)
                sciRxBufferArray[i] = 0;
            sciRxBufferCounter = 0;
        }
    }
    else if ((sciRxBufferCounter == 0) && (sciRxBuffer == SciCommand.Beginning))
    {
        sciRxBufferArray[sciRxBufferCounter] = sciRxBuffer;
        sciRxBufferCounter++;
    }
    else if ((sciRxBufferCounter == 1) && (sciRxBuffer == SciCommand.HisAddress))
    {
        sciRxBufferArray[sciRxBufferCounter] = sciRxBuffer;
        sciRxBufferCounter++;
    }
    else
    {
        for (i = 0; i < SciCommand.MessageLength; i++)
            sciRxBufferArray[i] = 0;
        sciRxBufferCounter = 0;
    }

    PieCtrlRegs.PIEACK.all|=0x100;

}

void SciSendData(unsigned char lenght, unsigned char *arr)
{
    int i;
    for (i = 0; i < lenght; i++)
    {
        while ( SciaRegs.SCICTL2.bit.TXEMPTY == 0);
        SciaRegs.SCITXBUF.all = arr[i];
    }
}

// TODO SciParseRxData
void SciParseRxData(void)
{
    sciRxCommand.myAddress = sciRxArray[2];

    if(sciRxCommand.myAddress == SciCommand.MyAddress)
    {
        sciRxCommand.operatingMode = sciRxArray[5];

        if (sciRxCommand.operatingMode == SciOperatingMode.Normal)
            SciParseNormal();
        else
            SciParseReload();

        sciRxCommand.rxMassageIsCorrect = 0;

    }
}

void SciPrepareTxData(void)
{
    unsigned char i;
    memset(&sciTxArray, 0, sizeof(sciTxArray));

    sciTxArray[0] = SciCommand.Beginning;
    sciTxArray[1] = SciCommand.MyAddress;
    sciTxArray[2] = SciCommand.HisAddress;

    if (sciRxCommand.operatingMode == SciOperatingMode.Normal)
        SciPrepareNormal();
    else
        SciPrepareReload();

    sciTxArray[SciCommand.MessageLength - 2] = SciCommand.End;
    sciTxArray[SciCommand.MessageLength - 1] = sciTxArray[0];
    for (i = 1; i < (SciCommand.MessageLength - 1); i++)
        sciTxArray[SciCommand.MessageLength - 1] = sciTxArray[SciCommand.MessageLength - 1] ^ sciTxArray[i];


    heatStatusToSendNew = heatStatus;
    if (heatStatusToSendOld == 1)
        if (heatStatusToSendOld != heatStatusToSendNew)
            heatStatusWasSent = 1;
}


// TODO SciParseNormal
void SciParseNormal()
{

    sciRxCommand.power = sciRxArray[3];

    if (heatFlag == 2)
    {
        if (sciRxArray[4] != 0)
            sciRxCommand.arretirCommand = sciRxArray[4];
        else
            sciRxCommand.arretirCommand = ArretirCommand.Stop;
    }
    else if (heatFlag == 1)
        sciRxCommand.arretirCommand = ArretirCommand.Open;

    sciRxCommand.operatingMode = sciRxArray[5];
    stabilizationFlag = sciRxArray[6];
    sciRxCommand.temperatureDS1624Top = sciRxArray[7];
    sciRxCommand.topDrvHeatStatus = sciRxArray[8];
    sciRxCommand.temperatureDS1624Bot = sciRxArray[9];
    sciRxCommand.botDrvHeatStatus = sciRxArray[10];

    if (sciRxCommand.temperatureDS1624Top > 128)
        temperatureDS1624TopNew = (sciRxCommand.temperatureDS1624Top - 256);
    else
        temperatureDS1624TopNew = sciRxCommand.temperatureDS1624Top;

    if (sciRxCommand.temperatureDS1624Bot > 128)
        temperatureDS1624Bot = (sciRxCommand.temperatureDS1624Bot - 256);
    else
        temperatureDS1624Bot = sciRxCommand.temperatureDS1624Bot;

    if (stableTimeFlag == 1)
    {
        if (abs(temperatureDS1624TopNew - temperatureDS1624Top) < 10)
            temperatureDS1624Top = temperatureDS1624TopNew;
    }
    else
        temperatureDS1624Top = temperatureDS1624TopNew;

}

/*
    Плата видео принимает посылки по 64 байта (32 слова)
    Дробит на 3 посылки и передает
    Первая часть: 22 байта
    Вторая часть: 22 байта
    Третья часть: 20 байт
 */

void SciParseReload()
{
    if (sciRxCommand.operatingMode == SciOperatingMode.BeginReload)
        SciParseBeginReload();

    else if ((sciRxCommand.operatingMode == SciOperatingMode.RecieveFirstPart) &&
            (sciRxCommand.operatingMode != sciTxCommand.operatingMode))
        SciParseRecieveFirstPart();

    else if ((sciRxCommand.operatingMode == SciOperatingMode.RecieveSecondPart) &&
            (sciRxCommand.operatingMode != sciTxCommand.operatingMode))
        SciParseRecieveSecondPart();

    else if ((sciRxCommand.operatingMode == SciOperatingMode.RecieveThirdPart) &&
            (sciRxCommand.operatingMode != sciTxCommand.operatingMode))
        SciParseRecieveThirdPart();

    else if ((sciRxCommand.operatingMode == SciOperatingMode.EndReload) &&
            (sciRxCommand.operatingMode != sciTxCommand.operatingMode))
        SciParseEndReload();

    sciTxCommand.numberOfPacket0 = sciRxCommand.numberOfPacket0;
    sciTxCommand.numberOfPacket1 = sciRxCommand.numberOfPacket1;
}



void SciParseBeginReload()
{
    int i = 0;
    sciRxCommand.firmwareLength0 = sciRxArray[3];
    sciRxCommand.firmwareLength1 = sciRxArray[4];
    sciRxCommand.numberOfPacket0 = sciRxArray[6];
    sciRxCommand.numberOfPacket1 = sciRxArray[7];

    sciRxCommand.firmwareLength = (Uint32)(((Uint32)sciRxCommand.firmwareLength1 << 8) | (Uint32)sciRxCommand.firmwareLength0);

    firmwareLength = sciRxCommand.firmwareLength / 2;
    firmwareCounter = 0;
    for (i = 0; i < 20000; i++)
        firmwareBuffer[i] = 0;

    sciTxCommand.operatingMode = SciOperatingMode.ReadyToReload;
}

void SciParseRecieveFirstPart()
{
    int i = 0;
    sciRxCommand.numberOfPacket0 = sciRxArray[6];
    sciRxCommand.numberOfPacket1 = sciRxArray[7];
    sciRxCommand.numberOfPacket = sciRxCommand.numberOfPacket0 | (sciRxCommand.numberOfPacket1 << 8);
    firmwareCounter = sciRxCommand.numberOfPacket * 64 / 2; // Считает от последней метки с 0-го

    for (i = 8; i < 30; i += 2)
    {
        if (firmwareCounter < firmwareLength)
        {
            firmwareBuffer[firmwareCounter] = (sciRxArray[i + 1] << 8) | sciRxArray[i];
            firmwareCounter++;
        }
        else
            break;
    }

    sciTxCommand.operatingMode = SciOperatingMode.FirstPartIsRecieved;

}

void SciParseRecieveSecondPart()
{
    int i = 0;
    sciRxCommand.numberOfPacket0 = sciRxArray[6];
    sciRxCommand.numberOfPacket1 = sciRxArray[7];
    sciRxCommand.numberOfPacket = sciRxCommand.numberOfPacket0 | (sciRxCommand.numberOfPacket1 << 8);
    firmwareCounter = sciRxCommand.numberOfPacket * 64 / 2 + 11; // Считает от последней метки с 11-го

    for (i = 8; i < 30; i += 2)
    {
        if (firmwareCounter < firmwareLength)
        {
            firmwareBuffer[firmwareCounter] = (sciRxArray[i + 1] << 8) | sciRxArray[i];
            firmwareCounter++;
        }
        else
            break;
    }

    sciTxCommand.operatingMode = SciOperatingMode.SecondPartIsRecieved;
}

void SciParseRecieveThirdPart()

{
    int i = 0;

    sciRxCommand.numberOfPacket0 = sciRxArray[6];
    sciRxCommand.numberOfPacket1 = sciRxArray[7];
    sciRxCommand.numberOfPacket = sciRxCommand.numberOfPacket0 | (sciRxCommand.numberOfPacket1 << 8);
    firmwareCounter = sciRxCommand.numberOfPacket * 64 / 2 + 22; // Считает от последней метки с 22-го

    for (i = 8; i < 28; i += 2)
    {
        if (firmwareCounter < firmwareLength)
        {
            firmwareBuffer[firmwareCounter] = (sciRxArray[i + 1] << 8) | sciRxArray[i];
            firmwareCounter++;
        }
        else
            break;
    }

    sciTxCommand.operatingMode = SciOperatingMode.ThirdPartIsRecieved;

}

void SciParseEndReload()
{
    int i = 0;
    firmwareSum = 1;
    for (i = 0; i < firmwareLength; i++)
        firmwareSum += (unsigned short)firmwareBuffer[i];

    sciTxCommand.operatingMode = SciOperatingMode.ReloadIsEnded;
}


void SciPrepareNormal()
{

    sciTxCommand.operatingMode = SciOperatingMode.Normal;

    sciTxArray[3] = sciTxCommand.operatingMode;
    sciTxArray[4] = sciTxCommand.arretirStatus;

    sciTxArray[5] = sciTxCommand.xGyroHighLow;
    sciTxArray[6] = sciTxCommand.xGyroHighHigh;

    sciTxArray[7] = sciTxCommand.yGyroHighLow;
    sciTxArray[8] = sciTxCommand.yGyroHighHigh;

    sciTxArray[9] = sciTxCommand.zGyroHighLow;
    sciTxArray[10] = sciTxCommand.zGyroHighHigh;

    sciTxArray[17] = sciTxCommand.temperature;
    sciTxArray[18] = sciTxCommand.memsTemperature;
    sciTxArray[19] = sciTxCommand.pressureHigh;
    sciTxArray[20] = sciTxCommand.pressureLow;
    sciTxArray[21] = sciTxCommand.memsPressureHigh;
    sciTxArray[22] = sciTxCommand.memsPressureLow;
    sciTxArray[23] = heatStatus;
    sciTxArray[24] = hollStatus;
}


void SciPrepareReload()
{
    sciTxArray[3] = sciTxCommand.operatingMode;
    sciTxArray[4] = sciTxCommand.numberOfPacket0;
    sciTxArray[5] = sciTxCommand.numberOfPacket1;

    if (sciRxCommand.operatingMode == SciOperatingMode.EndReload)
    {
        sciTxArray[6] = firmwareSum & 0xFF;
        sciTxArray[7] = (firmwareSum >> 8) & 0xFF;
    }
}





void Wait(long delay)
{
    for (timer.cpuTimer = 0; timer.cpuTimer < delay; );
}

void DriversOn()
{
    Wait(1000);
    GpioDataRegs.GPBSET.bit.GPIO52 = 1;
    Wait(2000);
    GpioDataRegs.GPBSET.bit.GPIO53 = 1;
    Wait(3000);
    GpioDataRegs.GPBSET.bit.GPIO54 = 1;
}

void DriversOff()
{
    GpioDataRegs.GPBCLEAR.bit.GPIO54 = 1;                       // выкл привод
    Wait(1000);
    GpioDataRegs.GPBCLEAR.bit.GPIO53 = 1;                       // выкл привод
    Wait(1000);
    GpioDataRegs.GPBCLEAR.bit.GPIO52 = 1;                         // выкл контроллеры
}





void StartCpuTimer()
{
    CpuTimer0Regs.TCR.bit.TSS = 0;  // start timer0 - запускаем таймер времени с дискретом 1мс
}

void ReadTmp100()
{
    timer.tmpTimer = 0;

    while(temperature == -127 || temperature == 0)
    {
        I2caRegs.I2CCNT     = 1;    // read 2 byte temperature
        I2caRegs.I2CMDR.all = 0x6C20;
        CounterForI2C = 0;
        while ((I2caRegs.I2CSTR.bit.RRDY == 0)&&(CounterForI2C<10000)) CounterForI2C++;   // wait for 1st byte
        if (I2caRegs.I2CSTR.bit.RRDY != 0)
        {
            temperature = I2caRegs.I2CDRR.all << 8;         // read upper 8 Bit (integers)
            temperature = temperature/256;
        }
        Wait(1);
        if (timer.tmpTimer > 100)
            break;
    }
}

void StartDrivers()
{
    GpioDataRegs.GPBSET.bit.GPIO52 = 1; // включить платы драйверов
    Wait(2000);

    if(temperature != -127)
    {
//        if (temperature < 0)
            if (temperature < 40)
        {

            GpioDataRegs.GPBSET.bit.GPIO51 = 1;   //  вкл обогрев верх
            Wait(300);
            GpioDataRegs.GPBSET.bit.GPIO55 = 1;
            Wait(300);
            heatStatus = 1;
            heatFlag = 1;
            heatStatusToSendOld = 1;
            heatStatusToSendNew = 1;

            timer.heatTimer = 0;
        }
    }

    Wait(2000);
    GpioDataRegs.GPBSET.bit.GPIO53 = 1;

    if(sciRxCommand.rxMassageIsCorrect);
    {
        sciRxCommand.rxMassageIsCorrect = 0;
        SciParseRxData();
        SciPrepareTxData();
        SciSendData(SciCommand.MessageLength, sciTxArray);
    }
    timer.ds1624Timer = 0;
    while(temperatureDS1624Top == -127 || temperatureDS1624Top == 0)
    {
        if(sciRxCommand.rxMassageIsCorrect);
        {
            sciRxCommand.rxMassageIsCorrect = FALSE;
            SciParseRxData();
            SciPrepareTxData();
        }
        if (timer.ds1624Timer > 100)
            break;
    }

    Wait(1000);
    if (heatStatus == 2)
        GpioDataRegs.GPBSET.bit.GPIO54 = 1; // если heatStatus == 1, то вкл gpio55 (доп питание)
    // если heatStatus == 0, то вкл gpio54 (осн питание)

    Wait(300);
}

void StartMems()
{
    ///------------ADIS16488-------------
    spiTxData = 0x8003;
    spi_xmit(spiTxData);
    while(SpiaRegs.SPIFFRX.bit.RXFFST !=1) { }
    spiRxData = SpiaRegs.SPIRXBUF;

    CurrentTime = CpuTimer0.InterruptCount;
    do
    {
        spiTxData = 0x0000;
        spi_xmit(spiTxData);
        while(SpiaRegs.SPIFFRX.bit.RXFFST !=1) { }
        spiRxData = SpiaRegs.SPIRXBUF;
    }while ((spiRxData!=3)&&((CpuTimer0.InterruptCount - CurrentTime)<200));


    //    spiTxData = 0x8C02; //100 Гц(2460/(23+1))  для ADIS16488
    spiTxData = 0x8C2A;  ///100 Гц (4250/(42+1))  для ADIS16497
    spi_xmit(spiTxData);
    while(SpiaRegs.SPIFFRX.bit.RXFFST !=1) { }
    spiRxData = SpiaRegs.SPIRXBUF;
    delay_loop();

    spiTxData = 0x8D00;
    spi_xmit(spiTxData);
    while(SpiaRegs.SPIFFRX.bit.RXFFST !=1) { }
    spiRxData = SpiaRegs.SPIRXBUF;
    delay_loop();

    spiTxData = 0x8000;
    spi_xmit(spiTxData);
    while(SpiaRegs.SPIFFRX.bit.RXFFST !=1) { }
    spiRxData = SpiaRegs.SPIRXBUF;

    CurrentTime = CpuTimer0.InterruptCount;
    do
    {
        spiTxData = 0x0000;
        spi_xmit(spiTxData);
        while(SpiaRegs.SPIFFRX.bit.RXFFST !=1) { }
        spiRxData = SpiaRegs.SPIRXBUF;
    }while ((spiRxData!=0)&&((CpuTimer0.InterruptCount - CurrentTime)<200));

    Xint1CountOld = xintCount;
}

void ReadMems()
{
    if (xintCount != Xint1CountOld)
    {

        if (abs(xintCount - Xint1CountOld)>1) errorCounter++;
        Xint1CountOld = xintCount;

        spiTxData = 0x4000;//x deltAngle low
        spi_xmit(spiTxData);
        while(SpiaRegs.SPIFFRX.bit.RXFFST !=1) { }
        spiRxData = SpiaRegs.SPIRXBUF;
        spiTxData = 0x4200;//x deltAngle high
        spi_xmit(spiTxData);
        while(SpiaRegs.SPIFFRX.bit.RXFFST !=1) { }
        spiRxData = SpiaRegs.SPIRXBUF;

        spiTxData = 0x4400;//y deltAngle low
        spi_xmit(spiTxData);
        while(SpiaRegs.SPIFFRX.bit.RXFFST !=1) { }
        spiRxData = SpiaRegs.SPIRXBUF;
        memsData->xDeltaAngleLow = spiRxData;
        spiTxData = 0x4600;//y deltAngle high
        spi_xmit(spiTxData);
        while(SpiaRegs.SPIFFRX.bit.RXFFST !=1) { }
        spiRxData = SpiaRegs.SPIRXBUF;
        memsData->xDeltaAngleHigh = spiRxData;

        spiTxData = 0x4800; //z deltAngle low
        spi_xmit(spiTxData);
        while(SpiaRegs.SPIFFRX.bit.RXFFST !=1) { }
        spiRxData = SpiaRegs.SPIRXBUF;
        memsData->yDeltaAngleLow = spiRxData;
        spiTxData = 0x4A00;//z deltAngle high
        spi_xmit(spiTxData);
        while(SpiaRegs.SPIFFRX.bit.RXFFST !=1) { }
        spiRxData = SpiaRegs.SPIRXBUF;
        memsData->yDeltaAngleHigh = spiRxData;

        spiTxData = 0x2800;//time stamp
        spi_xmit(spiTxData);
        while(SpiaRegs.SPIFFRX.bit.RXFFST !=1) { }
        spiRxData = SpiaRegs.SPIRXBUF;
        memsData->zDeltaAngleLow = spiRxData;
        spiTxData = 0x0E00;//temperature
        spi_xmit(spiTxData);
        while(SpiaRegs.SPIFFRX.bit.RXFFST !=1) { }
        spiRxData = SpiaRegs.SPIRXBUF;
        memsData->zDeltaAngleHigh = spiRxData;

        spiTxData = 0x0400; //data cnt
        spi_xmit(spiTxData);
        while(SpiaRegs.SPIFFRX.bit.RXFFST !=1) { }
        spiRxData = SpiaRegs.SPIRXBUF;
        memsData->timeStamp = spiRxData;

        spiTxData = 0x2900;//nothing
        spi_xmit(spiTxData);
        while(SpiaRegs.SPIFFRX.bit.RXFFST !=1) { }
        spiRxData = SpiaRegs.SPIRXBUF;
        memsData->temperature = spiRxData / 177 + 25;
        spiTxData = 0x2E00; //nothing
        spi_xmit(spiTxData);
        while(SpiaRegs.SPIFFRX.bit.RXFFST !=1) { }
        spiRxData = SpiaRegs.SPIRXBUF;
        memsData->dataCount = spiRxData;


        xGyroInt32 = ((int32)((((Uint32)memsData->xDeltaAngleHigh) << 16) + ((Uint32)memsData->xDeltaAngleLow)));
        yGyroInt32 = ((int32)((((Uint32)memsData->yDeltaAngleHigh) << 16 ) + ((Uint32)memsData->yDeltaAngleLow)));
        zGyroInt32 = ((int32)((((Uint32)memsData->zDeltaAngleHigh) << 16 ) + ((Uint32)memsData->zDeltaAngleLow)));

        if (xGyroInt32 == -65536)
            xGyroInt32 = 0;
        else if (xGyroInt32 == 65536)
            xGyroInt32 = 0;



        if (stabilizationFlag == TRUE && oldStabilizationFlag == FALSE)
        {
            deltaXSec = 0;
            deltaYSec = 0;
            deltaZSec = 0;
            oldStabilizationFlag = TRUE;
        }
        else if (stabilizationFlag == FALSE && oldStabilizationFlag == TRUE)
        {
            deltaXSec = 0;
            oldStabilizationFlag = FALSE;
        }

        if (stabilizationFlag == FALSE)
        {

            deltaXSec = deltaXSec + (float32)xGyroInt32 * 720.0 * 3600.0 / 2147483647.0 + xCoef;
            deltaYSec = deltaYSec + (float32)yGyroInt32 * 720.0 * 3600.0 / 2147483647.0 + yCoef;
            deltaZSec = deltaZSec + (float32)zGyroInt32 * 720.0 * 3600.0 / 2147483647.0 + zCoef;


            memsXSum += (float32)xGyroInt32 * 720.0 * 3600.0 / 2147483647.0;
            memsYSum += (float32)yGyroInt32 * 720.0 * 3600.0 / 2147483647.0;
            memsZSum += (float32)zGyroInt32 * 720.0 * 3600.0 / 2147483647.0;

            memsCounter++;

            if (memsCounter % 1000 == 0)
            {
                xCoef = - memsXSum / 1000;
                yCoef = - memsYSum / 1000;
                zCoef = - memsZSum / 1000;
                memsXSum = 0;
                memsYSum = 0;
                memsZSum = 0;
            }
        }

        if (stabilizationFlag == TRUE)
        {
            deltaXSec = deltaXSec + (float32)xGyroInt32 * 720.0 * 3600.0 / 2147483647.0 + xCoef;
            deltaYSec = deltaYSec + (float32)yGyroInt32 * 720.0 * 3600.0 / 2147483647.0 + yCoef;
            deltaZSec = deltaZSec + (float32)zGyroInt32 * 720.0 * 3600.0 / 2147483647.0 + zCoef;
        }
       
        sciTxCommand.memsPressureHigh = (memsData->pressure>>8) & 0xFF;
        sciTxCommand.memsPressureLow = memsData->pressure & 0xFF;

        sciTxCommand.memsTemperature = memsData->temperature;
        sciTxCommand.temperature = temperature;

        sciTxCommand.pressureHigh = (pressure >> 8) & 0xFF;
        sciTxCommand.pressureLow = pressure & 0xFF;

        xLong = deltaXSec;
        yLong = deltaYSec;
        zLong = deltaZSec;

        sciTxCommand.xGyroHighHigh = ((int32)xLong >> (4 + 8)) & 0xFF;
        sciTxCommand.xGyroHighLow = ((int32)xLong >> 4) & 0xFF;

        sciTxCommand.yGyroHighHigh = ((int32)yLong >> (4 + 8)) & 0xFF;
        sciTxCommand.yGyroHighLow = ((int32)yLong >> 4) & 0xFF;

        sciTxCommand.zGyroHighHigh = ((int32)zLong >> (4 + 8)) & 0xFF;
        sciTxCommand.zGyroHighLow = ((int32)zLong >> 4) & 0xFF;


    }
}

void delay_loop()
{
    long i;
    for (i = 0; i < 1000000; i++) {}
}

void spi_xmit(Uint16 a)
{
    SpiaRegs.SPITXBUF = a;
}

interrupt void xint1_isr(void)
{
    xintCount++;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

void CheckHeat()
{
    if (heatFlag == 1)
    {
        if (timer.heatTimer > 240000 || temperatureDS1624Top > 50 || temperatureDS1624Bot > 30)     // Если время прошло #времянагрева
        {
            GpioDataRegs.GPASET.bit.GPIO24 = 1;
            GpioDataRegs.GPASET.bit.GPIO25 = 1;

            GpioDataRegs.GPBCLEAR.bit.GPIO51 = 1;
            GpioDataRegs.GPBCLEAR.bit.GPIO55 = 1;

            GpioDataRegs.GPBCLEAR.bit.GPIO54 = 1;                       // выкл привод
            Wait(1000);
            GpioDataRegs.GPBCLEAR.bit.GPIO53 = 1;                       // выкл привод
            Wait(1000);

            heatStatus = 2;

            if (heatStatusWasSent == 1)
            {
                Wait(300);
                GpioDataRegs.GPBCLEAR.bit.GPIO52 = 1;
                DriversOn();
                heatFlag = 2;
                timer.heatTimer = 0;
            }
        }
    }
}


void CheckSci()
{
    if (sciRxCommand.rxMassageIsCorrect == TRUE)
    {
        SciParseRxData();
        SciPrepareTxData();
        SciSendData(SciCommand.MessageLength, sciTxArray);
    }
    if(sciTxCommand.operatingMode == SciOperatingMode.ReloadIsEnded)
    {
        firmwareLengthInRam = firmwareLength;
        int i = 0;
        firmwareSumInRam = 0;
        for (i = 0; i < firmwareLength; i++)
            firmwareSumInRam = firmwareSumInRam ^ firmwareBuffer[i];

        JUMP_TO_FLASHAPI;
    }

    if (!sciIsReceived)
    {

        if (timer.workingTimer > 5000)
        {
            DriversOff();
            
            // Restart device
            EALLOW;
            WdRegs.WDCR.all = 0x0028;
            EDIS;

            EALLOW;
            WdRegs.WDCR.all = 0x0000;
            EDIS;
        }
    }

}

void CheckArretirCommand()
{
    if (sciRxCommand.arretirCommand == 0x00)
    {
        sciRxCommand.arretirCommand = ArretirCommand.Stop;
    }

    else if (sciRxCommand.arretirCommand == ArretirCommand.RestartDrivers)
    {
        DriversOff();
        DriversOn();
    }

    else if (sciRxCommand.arretirCommand == ArretirCommand.TurnOnTopDriver)
        GpioDataRegs.GPBSET.bit.GPIO53 = 1;

    else if (sciRxCommand.arretirCommand == ArretirCommand.TurnOffTopDriver)
        GpioDataRegs.GPBCLEAR.bit.GPIO53 = 1;

    else if (sciRxCommand.arretirCommand == ArretirCommand.TurnOnBottomDriver)
    {
        if (GpioDataRegs.GPBDAT.bit.GPIO55 == 0)
            GpioDataRegs.GPBSET.bit.GPIO54 = 1;
    }

    else if (sciRxCommand.arretirCommand == ArretirCommand.TurnOffBottomDriver)
        GpioDataRegs.GPBCLEAR.bit.GPIO54 = 1;
}

void CheckArretirStatus(void)
{
    StatusArrietir(sciRxCommand.arretirCommand, &sciTxCommand);

    Arrietir(sciRxCommand.arretirCommand, &sciTxCommand, heatStatus);

    if(GpioDataRegs.GPBDAT.bit.GPIO53==1)
        sciTxCommand.arretirStatus=(sciTxCommand.arretirStatus&0x0F)|0x20;
    if(GpioDataRegs.GPBDAT.bit.GPIO53==0)
        sciTxCommand.arretirStatus=sciTxCommand.arretirStatus&0x1F;
    if(GpioDataRegs.GPBDAT.bit.GPIO54==1)
        sciTxCommand.arretirStatus=(sciTxCommand.arretirStatus&0x2F)|0x10;
    if(GpioDataRegs.GPBDAT.bit.GPIO54==0)
        sciTxCommand.arretirStatus&=0x2F;

    hollStatus = (GpioDataRegs.GPADAT.bit.GPIO24 << 3) | (GpioDataRegs.GPADAT.bit.GPIO25 << 2) | (GpioDataRegs.GPBDAT.bit.GPIO48 << 1) | GpioDataRegs.GPBDAT.bit.GPIO49;
}

void CheckTmp100()
{
    if ((CpuTimer0.InterruptCount - timerForTmp100) > 6000)
    {
        timerForTmp100 = CpuTimer0.InterruptCount;
        ReadTmp100();

        if (timer.workingTimer > 15000)
            stableTimeFlag = 1;
    }
}

