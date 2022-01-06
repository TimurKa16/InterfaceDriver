#include "user_structures.h"
#include "F28x_Project.h"

void GoToArretir()
{
    GpioDataRegs.GPACLEAR.bit.GPIO24 = 1;
    GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;
}

void Stop()
{
    GpioDataRegs.GPASET.bit.GPIO24 = 1; //бит движения
    GpioDataRegs.GPASET.bit.GPIO25 = 1; //бит направления
}

void GoFromArretir()
{
    GpioDataRegs.GPACLEAR.bit.GPIO24 = 1;
    GpioDataRegs.GPASET.bit.GPIO25 = 1;
}

void Arrietir(Uint8 komand, OutsideMessage_t * OutsideMessageLocal)
{
  bool NotInArretir = GpioDataRegs.GPBDAT.bit.GPIO48; //gpio1
  bool InArretir = GpioDataRegs.GPBDAT.bit.GPIO49; //gpio2

    if((!NotInArretir) && InArretir)
        OutsideMessageLocal->Status_Arr = (OutsideMessageLocal->Status_Arr & 0xF0) | 0x03;

    if(NotInArretir && (!InArretir))
        OutsideMessageLocal->Status_Arr = (OutsideMessageLocal->Status_Arr & 0xF0) | 0x01;

    if((!NotInArretir) && (!InArretir))
        OutsideMessageLocal->Status_Arr = (OutsideMessageLocal->Status_Arr & 0xF0) | 0x02;

    if (komand)
    {
        if (InArretir) GoFromArretir();/*GoToArretir();*/
        else  Stop();
    }else
    {
        if (NotInArretir) GoToArretir();/*GoFromArretir();*/
        else Stop();
    }
}
