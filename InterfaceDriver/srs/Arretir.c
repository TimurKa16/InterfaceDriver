#include "user_structures.h"
#include "F28x_Project.h"
#include "DeviceNumber.h"

void ToArretirPwm();
void FromArretirPwm();
extern void Wait(long delay);
char arretirCounter = 0;
unsigned long arretirWorkingTime = 0;
char arretirTimerFlag = 0;
char waitCommandState = 0;
char oldArretirCommand = 2;
extern char arretirStatusClosing;

extern ArretirCommandEnum ArretirCommand;
extern TimerStruct timer;


bool inArretir = 1;
bool notInArretir = 1;

// Доработка по функциям движения
void GoToArretir()
{
    GpioDataRegs.GPASET.bit.GPIO24 = 1;
    GpioDataRegs.GPASET.bit.GPIO25 = 1;
}

void Stop()
{
    GpioDataRegs.GPACLEAR.bit.GPIO24 = 1; //бит движения
    GpioDataRegs.GPACLEAR.bit.GPIO25 = 1; //бит направления
}

void GoFromArretir()
{
    GpioDataRegs.GPASET.bit.GPIO24 = 1;
    GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;
}

void StatusArrietir(Uint8 arretirCommand, SciTxCommandStruct * sciTxCommand)
{
    if (DEVICE_NUMBER == INTERFACE_201006 || DEVICE_NUMBER == INTERFACE_201015)
    {
        inArretir = GpioDataRegs.GPBDAT.bit.GPIO48;
        notInArretir = GpioDataRegs.GPBDAT.bit.GPIO49;
    }
    else
    {
        notInArretir = GpioDataRegs.GPBDAT.bit.GPIO48;
        inArretir = GpioDataRegs.GPBDAT.bit.GPIO49;
    }

    if (!arretirStatusClosing)
    {
        if((!notInArretir) && inArretir)
            sciTxCommand->arretirStatus = (sciTxCommand->arretirStatus & 0xF0) | 0x03;

        if(notInArretir && (!inArretir))
            sciTxCommand->arretirStatus = (sciTxCommand->arretirStatus & 0xF0) | 0x01;
    }
    else if (arretirStatusClosing)
        sciTxCommand->arretirStatus = (sciTxCommand->arretirStatus & 0xF0) | 0x02;
}

bool InArretirOld = 1;
bool NotInArretirOld = 1;

void Arrietir(Uint8 arretirCommand, SciTxCommandStruct * sciTxCommand, char heatFlag)
{


    if (DEVICE_NUMBER == INTERFACE_201006 || DEVICE_NUMBER == INTERFACE_201015)
    {
        inArretir = GpioDataRegs.GPBDAT.bit.GPIO48;
        notInArretir = GpioDataRegs.GPBDAT.bit.GPIO49;
    }
    else
    {
        notInArretir = GpioDataRegs.GPBDAT.bit.GPIO48;
        inArretir = GpioDataRegs.GPBDAT.bit.GPIO49;

    }

    

        if ((arretirCommand == ArretirCommand.Close && oldArretirCommand == ArretirCommand.Open) || (arretirCommand == ArretirCommand.Open && oldArretirCommand == ArretirCommand.Close))
            oldArretirCommand = ArretirCommand.Stop;


        if (arretirCommand == ArretirCommand.Close && oldArretirCommand != ArretirCommand.Close)
        {
            if (arretirTimerFlag == TRUE)
            {
                if (timer.arretirTimer > 500)
                {
                    if (inArretir || !notInArretir)
                    {
                        ToArretirPwm();
                    }
                    else
                    {
                        oldArretirCommand = ArretirCommand.Close;
                        Stop();
                        arretirWorkingTime = 0;
                        arretirTimerFlag = FALSE;
                        arretirStatusClosing = FALSE;
                    }
                }
            }
            else
            {
                timer.arretirTimer = 0;
                arretirTimerFlag = TRUE;
                arretirStatusClosing = TRUE;
            }
        }
        else if (arretirCommand == ArretirCommand.Open && oldArretirCommand != ArretirCommand.Open)
        {
            if (notInArretir || !inArretir)
                FromArretirPwm();
            else
            {
                //arretirError = 0;
                arretirWorkingTime = 0;
                oldArretirCommand = ArretirCommand.Open;
                Stop();
            }
        }
        else if (arretirCommand == ArretirCommand.Stop)
        {
            oldArretirCommand = ArretirCommand.Stop;
            Stop();
            arretirWorkingTime = 0;
        }
        else if ((arretirCommand == ArretirCommand.Open) && (arretirStatusClosing == TRUE))
        {
            arretirStatusClosing = FALSE;
        }
}

void ToArretirPwm()
{
    GoFromArretir();
}
void FromArretirPwm()
{
    GoToArretir();
}
