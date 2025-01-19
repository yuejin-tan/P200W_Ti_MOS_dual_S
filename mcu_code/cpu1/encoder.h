/*
 * encoder.h
 *
 *  Created on: 2024年9月7日
 *      Author: t
 */

#ifndef MCU_CODE_CPU1_ENCODER_H_
#define MCU_CODE_CPU1_ENCODER_H_

#include "F28x_Project.h"
#include "stdint.h"
#include "math.h"

#define EQEP_TYJDEV_REGS1 EQep1Regs
#define EQEP_TYJDEV_BASE1 EQEP1_BASE

#define EQEP_TYJDEV_REGS2 EQep2Regs
#define EQEP_TYJDEV_BASE2 EQEP2_BASE

struct encoder_struct {
    float raw2u16_ratio;
    float spdCalc_ratio;
};

#pragma FUNC_ALWAYS_INLINE(encoder_init)
static inline void encoder_init(struct encoder_struct* hEncoder, uint32_t ppr)
{
    hEncoder->raw2u16_ratio = 65536.0f / (ppr * 4);

    // default UPEVENT DIV = 4
    hEncoder->spdCalc_ratio = (float)(200e6 * 2.0 * M_PI) / ppr;
}

#pragma FUNC_ALWAYS_INLINE(encoder_u16Read)
static inline uint16_t encoder_u16Read(struct encoder_struct* hEncoder, uint32_t qposcnt)
{
    return qposcnt * hEncoder->raw2u16_ratio;
}

#pragma FUNC_ALWAYS_INLINE(encoder_lowSpdCalc)
static inline float encoder_lowSpdCalc(struct encoder_struct* hEncoder, uint16_t qcprdlat)
{
    if (qcprdlat == 0)
    {
        // 防止除零
        qcprdlat = 1;
    }
    return hEncoder->spdCalc_ratio / qcprdlat;
}

#endif /* MCU_CODE_CPU1_ENCODER_H_ */
