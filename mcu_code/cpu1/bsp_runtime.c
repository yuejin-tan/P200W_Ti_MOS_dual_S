/*
 * bsp_runtime.c
 *
 *  Created on: 2023年10月12日
 *      Author: t
 */

#include "bsp_runtime.h"
#include "bsp_inline.h"

#include "F28x_Project.h"
#include "stdint.h"

#include "cdb_cpu1.h"
#include "scd_inc.h"

#include "proj.h"

void scd_call_in_mainLoop()
{
    volatile struct SCI_REGS* scd_SciRegs = &SciaRegs;

    while ((*scd_SciRegs).SCIFFRX.bit.RXFFST > 0)
    {
        volatile uint16_t temp = (*scd_SciRegs).SCIRXBUF.all & 0xffu;
        SCD_Rev1Byte(&scd_1, temp);
    }

    while ((*scd_SciRegs).SCIFFTX.bit.TXFFST < 15)
    {
        (*scd_SciRegs).SCITXBUF.all = scd_send1Byte(&scd_1);
    }

    return;
}


void infLoopMain()
{
    for (;;)
    {
        mainLoopProcess();

#ifdef _SCD_ENABLE
        scd_call_in_mainLoop();
        cdb_ack_cpu1();

#endif

        if (bsp_tim1_polling_OF())
        {
            // tim 1 100ms间隔已到
            // 清空标志位
            bsp_tim1_clearFlg_OF();

            bsp_LED_D9_RED_CTRL(2);
        }

    }
}

