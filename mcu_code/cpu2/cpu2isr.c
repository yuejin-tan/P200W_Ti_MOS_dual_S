/*
 * cpu2isr.c
 *
 *  Created on: 2023年8月16日
 *      Author: t
 */


#include "F2837xD_Ipc_drivers.h"
#include "F28x_Project.h"
#include "cpu2isr.h"

#include "cpu2funcs.h"
#include "cdb_cpu2.h"

#include "topCfg.h"

 // Unused ISR
#pragma CODE_SECTION(NOTUSED_ISR_CPU2_TYJ, "ramfuncs2");
interrupt void NOTUSED_ISR_CPU2_TYJ(void)
{
    asm("      ESTOP0");
    for (;;);
}

#pragma DATA_SECTION(timeStampCpu2, "cpu2bss")
volatile uint64_t timeStampCpu2;

#pragma DATA_SECTION(cpu2mainIsrTick, "cpu2bss")
volatile uint16_t cpu2mainIsrTick;

#pragma CODE_SECTION(ipc_isr_cpu2, "ramfuncs2");
interrupt void ipc_isr_cpu2(void)
{
    volatile struct IPC_REGS_CPU2* IpcRegCpu2 = (void*)&IpcRegs;
    IpcRegCpu2->IPCACK.bit.IPC0 = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1; //清除中断向量表中的标志位

    timeStampCpu2 = getIpcTimeStamp();

#ifdef _SCD_ENABLE
    cdb_dlog_mIsr(&cdb1, CDB_BUFF_ADDR);
#endif

    cpu2mainIsrTick = getIpcTimeStamp() - timeStampCpu2;
}
