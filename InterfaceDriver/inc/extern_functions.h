/*
 * extern_functions.h
 *
 *  Created on: 09.07.2019
 *      Author: Arthur
 */

#ifndef __EXTERN_FUNCTIONS_H__
#define __EXTERN_FUNCTIONS_H__


extern void ConfigureADC(void);

extern void InitEpwm(void);
extern void SetupADCEpwm(void);

interrupt void adca1_isr(void);

__interrupt void CpuTimerIsr(void);
// Globals
interrupt void SciRxIsr(void);
void SciParseRxData(void);
void SciRx(Uint16 bukva);
void SciPrepareTxData(void);

void My_Com_Tx(unsigned char flagswitch,unsigned char *mass);
void My_Com_Tx2(unsigned char flagswitch,unsigned char *mass);
void SwitchKomand(unsigned char flagswitch,unsigned char *mass);
extern void InitSci(void);

extern void StatusArrietir(Uint8 command, SciTxCommandStruct * sciTxCommand);
extern void Arrietir(Uint8 komand, SciTxCommandStruct * sciTxCommand, char heatStatus);
extern void Stop();

#endif
