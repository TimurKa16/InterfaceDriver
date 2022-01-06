/*
 * extern_functions.h
 *
 *  Created on: 09.07.2019
 *      Author: Arthur
 */

#ifndef __EXTERN_FUNCTIONS_H__
#define __EXTERN_FUNCTIONS_H__


extern void ConfigureADC(void);

extern void ConfigureEPWM(void);
extern void SetupADCEpwm(void);

interrupt void adca1_isr(void);

__interrupt void cpu_timer0_isr(void);
// Globals
interrupt void SCIB_RX_isr(void);
void SwitchKomand32_Rx(void);
void SCIB_RX_32(Uint16 bukva,unsigned char start1, unsigned char start2, unsigned char finish);
void SwitchKomand32_Tx(void);

void My_Com_Tx(unsigned char flagswitch,unsigned char *mass);
void My_Com_Tx2(unsigned char flagswitch,unsigned char *mass);
void SwitchKomand(unsigned char flagswitch,unsigned char *mass);
extern void scib_echoback_init(void);

extern void Arrietir(Uint8 komand, OutsideMessage_t * OutsideMessageLocal);
extern void Stop();

#endif
