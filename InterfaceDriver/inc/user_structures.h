#ifndef USER_STRUCTURES_H_
#define USER_STRUCTURES_H_

#include "F28x_Project.h"

typedef unsigned char Uint8;
typedef char int8;

typedef struct
{
    const unsigned char Beginning;
    const unsigned char MyAddress;
    const unsigned char HisAddress;
    const unsigned char End;
    const unsigned char MessageLength;
} SciCommandEnum;


typedef struct
{
    Uint8 myAddress;
    Uint8 Addres_Rx;
    Uint8 arretirCommand;
    Uint8 power;
    Uint8 operatingMode;
    Uint8 memsMode;
    int8  temperatureDS1624Top;
    Uint8 topDrvHeatStatus;
    int8  temperatureDS1624Bot;
    Uint8 botDrvHeatStatus;

    unsigned short   firmwareLength0;
    unsigned short   firmwareLength1;
    unsigned short   firmwareLength;

    unsigned short   numberOfPacket0;
    unsigned short   numberOfPacket1;
    unsigned short   numberOfPacket;
    unsigned short   oldNumberOfPacket;
    unsigned char rxMassageIsCorrect;
}SciRxCommandStruct;

typedef struct
{
    Uint8   myAddress;
    Uint8   Status_Pitanie;
    Uint8   arretirStatus;
    Uint8  operatingMode;
    Uint8   xGyroHighHigh;
    Uint8   xGyroHighLow;
    Uint8   xGyroLowHigh;
    Uint8   xGyroLowLow;
    Uint8   yGyroHighHigh;
    Uint8   yGyroHighLow;
    Uint8   yGyroLowHigh;
    Uint8   yGyroLowLow;
    Uint8   zGyroHighHigh;
    Uint8   zGyroHighLow;
    Uint8   zGyroLowHigh;
    Uint8   zGyroLowLow;
    int8   memsTemperature;
    int8   temperature;
    Uint8   memsPressureHigh;
    Uint8   memsPressureLow;
    Uint8   memsHumidityHigh;
    Uint8   memsHumidityLow;
    Uint8   heatCommand;
    Uint8   pressureHigh;
    Uint8   pressureLow;

    unsigned char numberOfPacket0;
    unsigned char numberOfPacket1;
    unsigned int numberOfPacket;
}SciTxCommandStruct;







typedef struct
{
    Uint16  id;
    Uint16  sysFlag;
    int16   temperature;
    int16   pressure;
    Uint16  xGyroLow;
    Uint16  xGyroHigh;
    Uint16  yGyroLow;
    Uint16  yGyroHigh;
    Uint16  zGyroLow;
    Uint16  zGyroHigh;
    Uint16  xAccLow;
    Uint16  xAccHigh;
    Uint16  yAccLow;
    Uint16  yAccHigh;
    Uint16  zAccHigh;
    Uint16  zAccLow;
    Uint16  dataCount;
    Uint16  crcLow;
    Uint16  crcUpper;
    Uint16 xDeltaAngleHigh;
    Uint16 xDeltaAngleLow;
    Uint16 yDeltaAngleHigh;
    Uint16 yDeltaAngleLow;
    Uint16 zDeltaAngleHigh;
    Uint16 zDeltaAngleLow;
    Uint16 timeStamp;
} MemsDataStruct;

typedef struct
{
    const char Closed;
    const char Closing;
    const char Opened;

} ArretirStatusEnum;


typedef struct
{
    const char Open;
    const char Close;
    const char Stop;
    const char RestartDrivers;
    const char TurnOnTopDriver;
    const char TurnOffTopDriver;
    const char TurnOnBottomDriver;
    const char TurnOffBottomDriver;
} ArretirCommandEnum;


typedef struct
{
    const char Normal;
    const char BeginReload;
    const char EndReload;
    const char RecieveFirstPart;
    const char RecieveSecondPart;
    const char RecieveThirdPart;

    const char NotReadyToReload;
    const char ReadyToReload;
    const char ReloadIsEnded;
    const char FirstPartIsRecieved;
    const char SecondPartIsRecieved;
    const char ThirdPartIsRecieved;

} SciOperatingModeEnum;


typedef struct
{
    unsigned long arretirTimer;
    unsigned long tmpTimer;
    unsigned long ds1624Timer;
    unsigned long cpuTimer;
    unsigned long heatTimer;
    unsigned long workingTimer;
} TimerStruct;





#ifndef TRUE
#define TRUE 1
#define FALSE 0
#endif



#endif
