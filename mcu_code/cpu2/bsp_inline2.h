/*
 * bsp_inline2.h
 *
 *  Created on: 2023年10月13日
 *      Author: t
 */

#ifndef SHARE_CPU2_BSP_INLINE2_H_
#define SHARE_CPU2_BSP_INLINE2_H_

#include "F28x_Project.h"
#include "stdint.h"

 // timer 0

#pragma FUNC_ALWAYS_INLINE(bsp_tim0_polling_OF)
static inline int16_t bsp_tim0_polling_OF(void)
{
    return CpuTimer0Regs.TCR.bit.TIF;
}

#pragma FUNC_ALWAYS_INLINE(bsp_tim0_clearFlg_OF)
static inline void bsp_tim0_clearFlg_OF(void)
{
    CpuTimer0Regs.TCR.bit.TIF = 1;
}

// timer 1

#pragma FUNC_ALWAYS_INLINE(bsp_tim1_polling_OF)
static inline int16_t bsp_tim1_polling_OF(void)
{
    return CpuTimer1Regs.TCR.bit.TIF;
}

#pragma FUNC_ALWAYS_INLINE(bsp_tim1_clearFlg_OF)
static inline void bsp_tim1_clearFlg_OF(void)
{
    CpuTimer1Regs.TCR.bit.TIF = 1;
}

// LED D10 BLUE
#pragma FUNC_ALWAYS_INLINE(bsp_LED_D10_BLUE_CTRL)
static inline void bsp_LED_D10_BLUE_CTRL(int sta)
{
    if (sta == 1)
    {
        GpioDataRegs.GPACLEAR.bit.GPIO31 = 1;
    }
    else if (sta == 0)
    {
        GpioDataRegs.GPASET.bit.GPIO31 = 1;
    }
    else
    {
        GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;
    }
}

#endif /* SHARE_CPU2_BSP_INLINE2_H_ */
