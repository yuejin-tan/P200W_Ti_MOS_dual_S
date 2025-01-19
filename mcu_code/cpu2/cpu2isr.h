/*
 * cpu2isr.h
 *
 *  Created on: 2023年8月16日
 *      Author: t
 */

#ifndef MCU_CODE_CPU2_CPU2ISR_H_
#define MCU_CODE_CPU2_CPU2ISR_H_

#include "stdint.h"

interrupt void NOTUSED_ISR_CPU2_TYJ(void);

interrupt void ipc_isr_cpu2(void);

extern volatile uint64_t timeStampCpu2;
extern volatile uint16_t cpu2mainIsrTick;

#endif /* MCU_CODE_CPU2_CPU2ISR_H_ */
