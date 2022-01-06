#ifndef USER_STRUCTURES_H_
#define USER_STRUCTURES_H_

#include "F28x_Project.h"

typedef unsigned char Uint8;

typedef struct ComPort_struct
{
    Uint8   Bukva;
    Uint8   Bukva1;
    Uint8   index;
    Uint8   i;
    Uint8   flagMess;
    Uint8   u;
    Uint8   Start1;
    Uint8   Start2;
    Uint8   finish;
    Uint8   Komand;
    Uint8   reserv;
    Uint8   flagViv;
    Uint8   checksum;
    Uint8   Sklad[32];
    Uint8   message1[32];
}ComPort_t;

typedef struct OutsideKomands_struct
{
    Uint8   Addres_Rx;
    Uint8   Komand_Arr;
    Uint8   Komand_Pitanie;
    Uint8   Tek_Regim_Raboti;
    Uint8   Mems_Regim;
    Uint8   reserv1;
    Uint8   reserv2;
    Uint8   reserv3;
    Uint8   reserv4;
    Uint8   reserv5;
    Uint8   reserv6;
    Uint8   reserv7;
    Uint8   reserv8;
    Uint8   reserv9;
    Uint8   reserv10;
    Uint8   reserv11;
    Uint8   reserv12;
    Uint8   reserv13;
    Uint8   reserv14;
    Uint8   reserv15;
    Uint8   reserv16;
    Uint8   reserv17;
    Uint8   reserv18;
    Uint8   reserv19;
    Uint8   reserv20;
    Uint8   reserv21;
    Uint8   reserv22;
    Uint8   reserv23;
}OutsideKomands_t;

typedef struct InsideKomands_struct
{
    Uint8   SwitchDriver;
    Uint8   SwitchVideo;
    Uint8   SwitchReset;
    Uint8   Status;
    Uint16  Freq;
    Uint16  Target;
    Uint16  HighLimit;
    Uint16  LowLimit;
    Uint16  Accuracity;
    Uint16  velocity1;
    Uint16  Arrietir;
}InsideKomands_t;
/*
typedef struct OutsideMessage_struct
{
    Uint8   Addres_Tx;
    Uint8   Status_Pitanie;
    Uint8   Status_Arr;
    Uint8   Status_Tek_Regim_Raboti;
    Uint8   Dat_Vip;
    Uint8   Dat_Korpus;
    Uint8   Status_Regim_Raboti_Mems;
    Uint8   reserv1;
    Uint8   Mems_X_V_Low;
    Uint8   Mems_X_V_High;
    Uint8   Mems_Y_V_Low;
    Uint8   Mems_Y_V_High;
    Uint8   Mems_Z_V_Low;
    Uint8   Mems_Z_V_High;
    Uint8   Mems_X_a_Low;
    Uint8   Mems_X_a_High;
    Uint8   Mems_Y_a_Low;
    Uint8   Mems_Y_a_High;
    Uint8   Mems_Z_a_Low;
    Uint8   Mems_Z_a_High;
    Uint8   Pitanie_Vhod;
    Uint8   Pitanie_Dr1_Cifra;
    Uint8   Pitanie_Dr2_Cifra;
    Uint8   Pitanie_Dr1_Sil;
    Uint8   Pitanie_Dr2_Sil;
    Uint8   Pitanie_Plis;
    Uint8   Pitanie_Arr;
    Uint8   Pitanie_Nagrev;
}OutsideMessage_t;
 */

typedef struct OutsideMessage_struct
{
    ///    Uint8   StartOfMessage;                 //1

    Uint8   Addres_Tx;                      //1
    Uint8   Status_Pitanie;                 //2
    Uint8   Status_Arr;                     //3
    Uint8   Status_Tek_Regim_Raboti;        //4
    Uint8   Dat_Vip;                        //5
    Uint8   Dat_Korpus;                     //6
    Uint8   Status_Regim_Raboti_Mems;       //7
    Uint8   reserv1;                        //8
    Uint8   X_GYRO_OUT_Hi;                  //9
    Uint8   X_GYRO_OUT_Lo;                  //10
    Uint8   X_GYRO_LOW_Hi;                  //11
    Uint8   X_GYRO_LOW_Lo;                  //12
    Uint8   Y_GYRO_OUT_Hi;                  //13
    Uint8   Y_GYRO_OUT_Lo;                  //14
    Uint8   Y_GYRO_LOW_Hi;                  //15
    Uint8   Y_GYRO_LOW_Lo;                  //16
    Uint8   Z_GYRO_OUT_Hi;                  //17
    Uint8   Z_GYRO_OUT_Lo;                  //18
    Uint8   Z_GYRO_LOW_Hi;                  //19
    Uint8   Z_GYRO_LOW_Lo;                  //20
    Uint8   X_ACCL_OUT_Hi;                  //21
    Uint8   X_ACCL_OUT_Lo;                  //22
    Uint8   X_ACCL_LOW_Hi;                  //23
    Uint8   X_ACCL_LOW_Lo;                  //24
    Uint8   Y_ACCL_OUT_Hi;                  //25
    Uint8   Y_ACCL_OUT_Lo;                  //26
    Uint8   Y_ACCL_LOW_Hi;                  //27
    Uint8   Y_ACCL_LOW_Lo;                  //28
    Uint8   Z_ACCL_OUT_Hi;                  //29
    Uint8   Z_ACCL_OUT_Lo;                  //30
    Uint8   Z_ACCL_LOW_Hi;                  //31
    Uint8   Z_ACCL_LOW_Lo;                  //32
    Uint8   TEMP_OUT_Hi;                    //33
    Uint8   TEMP_OUT_Lo;                    //34
    Uint8   BAROM_OUT_Hi;                   //35
    Uint8   BAROM_OUT_Lo;                   //36
    Uint8   Humidity_Hi;                    //37
    Uint8   Humidity_Lo;                    //38
    Uint8   Pitanie_Vhod;                   //39
    Uint8   Pitanie_Dr1_Cifra;              //40
    Uint8   Pitanie_Dr2_Cifra;              //41
    Uint8   Pitanie_Dr1_Sil;                //42
    Uint8   Pitanie_Dr2_Sil;                //43
    Uint8   Pitanie_Plis;                   //44
    Uint8   Pitanie_Arr;                    //45
    Uint8   Pitanie_Nagrev;                 //46

    Uint8   adcPressure_OUT_Hi;              //47
    Uint8   adcPressure_OUT_Lo;             //48

    Uint8   Reserv[16];                     //49 - 64
}OutsideMessage_t;




typedef struct StatusFlags_struct
{
    Uint8   Switch_tx_COM;
    Uint8   Switch_tx_COM2;
    Uint8   Switch_tx_COM32;
    Uint8   Switch_rx_COM;
    Uint8   Switch_status;
    Uint8   index;
    Uint8   i;
    Uint8   Switch_Otvet_COM;
    Uint8   Switch_Otvet_COM32;
    Uint8   u;
    Uint8   Start;
    Uint8   finish;
    Uint8   Komand;
    Uint8   flagTx;
    Uint8   flagViv;
    Uint8   checksum;
    Uint8   On_Driver1;
    Uint8   On_Driver2;
    Uint8   On_Driver3;
    Uint8   Switch_Arr;
}StatusFlags_t;

typedef struct Counters_struct
{
    Uint8   DelayPlis;
    Uint8   DelayDriver1;
    Uint8   DelayDriver2;
    Uint8   reserv;
    Uint16  DelayDriver3;
}Counters_t;




//пока не используется
typedef struct Komand_3_Tx_struct
{
    Uint8   Vkl_Regim_Nagreva:1;
    Uint8   reserv:7;
}Komand_3_Tx_t;

//пока не используется
typedef struct Komand_2_Tx_struct
{
    Uint8   reserv:1;
    Uint8   CU:2;
    Uint8   Uderg:1;
    Uint8   Stabil:1;
    Uint8   Vkl_Most:2;
}Komand_2_Tx_t;



typedef struct BRFData_struct
{
    Uint16  BURST_ID;
    Uint16  SYS_E_FLAG;
    int16   TEMP_OUT;
    Uint16  X_GYRO_LOW;
    Uint16  X_GYRO_OUT;
    Uint16  Y_GYRO_LOW;
    Uint16  Y_GYRO_OUT;
    Uint16  Z_GYRO_LOW;
    Uint16  Z_GYRO_OUT;
    Uint16  X_ACCL_LOW;
    Uint16  X_ACCL_OUT;
    Uint16  Y_ACCL_LOW;
    Uint16  Y_ACCL_OUT;
    Uint16  Z_ACCL_LOW;
    Uint16  Z_ACCL_OUT;
    Uint16  DATA_CNT;
    Uint16  CRC_LWR;
    Uint16  CRC_UPR;
} BRFData_t;





#endif
