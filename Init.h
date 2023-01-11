/*
 * init.h
 *
 *  Created on: 2021�~6��11��
 *      Author: Johnson
 */

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C" {
#endif

#ifndef INIT_H_
#define INIT_H_

extern void InitGPIO(void);
extern void InitPIE(void);
extern void InitADC(void);
extern void InitADCSOC(void);
extern void InitCPUTimer(void);
extern void InitEPWM(unsigned int period);



#endif /* INIT_H_ */
