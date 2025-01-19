/*
 * boot.c
 *
 *  Created on: 2023年8月15日
 *      Author: t
 */

#include "F2837xD_Ipc_drivers.h"
#include "F28x_Project.h"
#include "can.h"

#include "cdb_cpu2.h"

#include "bsp_init2.h"
#include "cpu2funcs.h"
#include "cpu2isr.h"

#include "algo_code_config.h"

#pragma CODE_SECTION(CAN_initModule2, "ramfuncs2");
void CAN_initModule2(uint32_t base)
{
    // Place CAN controller in init state, regardless of previous state.  This
    // will put controller in idle, and allow the message object RAM to be
    // programmed.
    HWREGH(base + CAN_O_CTL) |= ((uint16_t)CAN_CTL_INIT |
        (uint16_t)CAN_INIT_PARITY_DISABLE);

    // Initialize the message RAM before using it.
    CAN_initRAM(base);

    // Force module to reset state
    HWREGH(base + CAN_O_CTL) |= CAN_CTL_SWR;

    DELAY_US2(1U);

    // Enable write access to the configuration registers
    HWREGH(base + CAN_O_CTL) |= CAN_CTL_CCE;
}

#pragma CODE_SECTION(CAN_setBitTiming2, "ramfuncs2");
void CAN_setBitTiming2(uint32_t base, uint16_t prescaler,
    uint16_t prescalerExtension, uint16_t tSeg1, uint16_t tSeg2, uint16_t sjw)
{
    uint16_t savedInit;
    uint32_t bitReg;

    // To set the bit timing register, the controller must be placed in init
    // mode (if not already), and also configuration change bit enabled.
    // State of the init bit should be saved so it can be restored at the end.
    savedInit = HWREGH(base + CAN_O_CTL);
    HWREGH(base + CAN_O_CTL) = savedInit | CAN_CTL_INIT | CAN_CTL_CCE;

    // Set the bit fields of the bit timing register
    bitReg = (uint32_t)((uint32_t)prescaler & CAN_BTR_BRP_M);
    bitReg |= (uint32_t)(((uint32_t)sjw << CAN_BTR_SJW_S) & CAN_BTR_SJW_M);
    bitReg |= (uint32_t)(((uint32_t)tSeg1 << CAN_BTR_TSEG1_S) &
        CAN_BTR_TSEG1_M);
    bitReg |= (uint32_t)(((uint32_t)tSeg2 << CAN_BTR_TSEG2_S) &
        CAN_BTR_TSEG2_M);
    bitReg |= (uint32_t)(((uint32_t)prescalerExtension << CAN_BTR_BRPE_S) &
        CAN_BTR_BRPE_M);

    HWREG_BP(base + CAN_O_BTR) = bitReg;

    //
    // Clear the config change bit, and restore the init bit.
    //
    savedInit &= ~((uint16_t)CAN_CTL_CCE);

    HWREGH(base + CAN_O_CTL) = savedInit;
}

#pragma CODE_SECTION(CAN_setBitRate2, "ramfuncs2");
void CAN_setBitRate2(uint32_t base, uint32_t clockFreq, uint32_t bitRate, uint16_t bitTime)
{
    uint16_t brp;
    uint16_t tPhase;
    uint16_t phaseSeg2;
    uint16_t tSync = 1U;
    uint16_t tProp = 2U;
    uint16_t tSeg1;
    uint16_t tSeg2;
    uint16_t sjw;
    uint16_t prescaler;
    uint16_t prescalerExtension;

    // Calculate bit timing values
    brp = (uint16_t)(clockFreq / (bitRate * bitTime));
    tPhase = bitTime - (tSync + tProp);
    if ((tPhase / 2U) <= 8U)
    {
        phaseSeg2 = tPhase / 2U;
    }
    else
    {
        phaseSeg2 = 8U;
    }
    tSeg1 = ((tPhase - phaseSeg2) + tProp) - 1U;
    tSeg2 = phaseSeg2 - 1U;
    if (phaseSeg2 > 4U)
    {
        sjw = 3U;
    }
    else
    {
        sjw = tSeg2;
    }
    prescalerExtension = ((brp - 1U) / 64U);
    prescaler = ((brp - 1U) % 64U);

    // Set the calculated timing parameters
    CAN_setBitTiming2(base, prescaler, prescalerExtension, tSeg1, tSeg2, sjw);
}

#pragma CODE_SECTION(CAN_setupMessageObject2, "ramfuncs2");
void CAN_setupMessageObject2(uint32_t base, uint32_t objID, uint32_t msgID,
    CAN_MsgFrameType frame, CAN_MsgObjType msgType,
    uint32_t msgIDMask, uint32_t flags, uint16_t msgLen)
{
    uint32_t cmdMaskReg = 0U;
    uint32_t maskReg = 0U;
    uint32_t arbReg = 0U;
    uint32_t msgCtrl = 0U;

    // Wait for busy bit to clear
    while ((HWREGH(base + CAN_O_IF1CMD) & CAN_IF1CMD_BUSY) == CAN_IF1CMD_BUSY)
    {
    }

    switch (msgType)
    {
        // Transmit message object.
    case CAN_MSG_OBJ_TYPE_TX:
    {
        // Set message direction to transmit.
        arbReg = CAN_IF1ARB_DIR;
        break;
    }

    // Remote frame receive remote, with auto-transmit message object.
    case CAN_MSG_OBJ_TYPE_RXTX_REMOTE:
    {
        // Set message direction to Tx for remote receivers.
        arbReg = CAN_IF1ARB_DIR;

        // Set this object to auto answer if a matching identifier is seen.
        msgCtrl = (uint32_t)((uint32_t)CAN_IF1MCTL_RMTEN |
            (uint32_t)CAN_IF1MCTL_UMASK);

        break;
    }

    // Transmit remote request message object (CAN_MSG_OBJ_TYPE_TX_REMOTE)
    // or Receive message object (CAN_MSG_OBJ_TYPE_RX).
    default:
    {
        // Set message direction to read.
        arbReg = 0U;

        break;
    }
    }

    // Set values based on Extended Frame or Standard Frame
    if (frame == CAN_MSG_FRAME_EXT)
    {
        // Configure the Mask Registers for 29 bit Identifier mask.
        if ((flags & CAN_MSG_OBJ_USE_ID_FILTER) == CAN_MSG_OBJ_USE_ID_FILTER)
        {
            maskReg = msgIDMask & CAN_IF1MSK_MSK_M;
        }

        // Set the 29 bit version of the Identifier for this message
        // object. Mark the message as valid and set the extended ID bit.
        arbReg |= (msgID & CAN_IF1ARB_ID_M) | CAN_IF1ARB_MSGVAL |
            CAN_IF1ARB_XTD;
    }
    else
    {
        // Configure the Mask Registers for 11 bit Identifier mask.
        if ((flags & CAN_MSG_OBJ_USE_ID_FILTER) == CAN_MSG_OBJ_USE_ID_FILTER)
        {
            maskReg = ((msgIDMask << CAN_IF1ARB_STD_ID_S) &
                CAN_IF1ARB_STD_ID_M);
        }

        // Set the 11 bit version of the Identifier for this message
        // object. The lower 18 bits are set to zero. Mark the message as
        // valid.
        arbReg |= ((msgID << CAN_IF1ARB_STD_ID_S) & CAN_IF1ARB_STD_ID_M) |
            CAN_IF1ARB_MSGVAL;
    }

    // If the caller wants to filter on the extended ID bit then set it.
    maskReg |= (flags & CAN_MSG_OBJ_USE_EXT_FILTER);

    // The caller wants to filter on the message direction field.
    maskReg |= (flags & CAN_MSG_OBJ_USE_DIR_FILTER);

    // If any filtering is requested, set the UMASK bit to use mask register
    if (((flags & CAN_MSG_OBJ_USE_ID_FILTER) |
        (flags & CAN_MSG_OBJ_USE_DIR_FILTER) |
        (flags & CAN_MSG_OBJ_USE_EXT_FILTER)) != 0U)
    {
        msgCtrl |= CAN_IF1MCTL_UMASK;
    }

    // Set the data length for the transfers. This is applicable only for
    // Tx mailboxes. For Rx mailboxes, dlc is updated on receving a frame.
    if ((msgType == CAN_MSG_OBJ_TYPE_TX) ||
        (msgType == CAN_MSG_OBJ_TYPE_RXTX_REMOTE))
    {
        msgCtrl |= ((uint32_t)msgLen & CAN_IF1MCTL_DLC_M);
    }

    // If this is a single transfer or the last mailbox of a FIFO, set EOB bit.
    // If this is not the last entry in a FIFO, leave the EOB bit as 0.
    if ((flags & CAN_MSG_OBJ_FIFO) == 0U)
    {
        msgCtrl |= CAN_IF1MCTL_EOB;
    }

    // Enable transmit interrupts if they should be enabled.
    msgCtrl |= (flags & CAN_MSG_OBJ_TX_INT_ENABLE);

    // Enable receive interrupts if they should be enabled.
    msgCtrl |= (flags & CAN_MSG_OBJ_RX_INT_ENABLE);

    // Set the Control, Arb, and Mask bit so that they get transferred to the
    // Message object.
    cmdMaskReg |= CAN_IF1CMD_ARB;
    cmdMaskReg |= CAN_IF1CMD_CONTROL;
    cmdMaskReg |= CAN_IF1CMD_MASK;
    cmdMaskReg |= CAN_IF1CMD_DIR;

    // Write out the registers to program the message object.
    HWREG_BP(base + CAN_O_IF1MSK) = maskReg;
    HWREG_BP(base + CAN_O_IF1ARB) = arbReg;
    HWREG_BP(base + CAN_O_IF1MCTL) = msgCtrl;

    // Transfer data to message object RAM
    HWREG_BP(base + CAN_O_IF1CMD) =
        cmdMaskReg | (objID & CAN_IF1CMD_MSG_NUM_M);
}

#pragma CODE_SECTION(cpu2initCanB, "ramfuncs2");
void cpu2initCanB(void)
{
    EALLOW;
    // 启动CAN的时钟
    CpuSysRegs.PCLKCR10.bit.CAN_B = 1;
    // CpuSysRegs.PCLKCR10.bit.CAN_B = 1;
    EDIS;

    CAN_initModule2(CANB_BASE);
    // 500k
    // CAN_setBitRate2(CANB_BASE, 200ul * 1000ul * 1000ul, 500ul * 1000ul, 16);
    // CAN_setBitRate2(CANB_BASE, 200ul * 1000ul * 1000ul, 500ul * 1000ul, 20);
    // 1M
    CAN_setBitRate2(CANB_BASE, 200ul * 1000ul * 1000ul, 1000ul * 1000ul, 20);

    // Initialize the receive message object used for receiving CAN messages.
    // 加入过滤
    // 1# dump指令邮箱
    CAN_setupMessageObject2(CANB_BASE, 1, 0x1, CAN_MSG_FRAME_STD,
        CAN_MSG_OBJ_TYPE_RX, 0x7ffu, CAN_MSG_OBJ_USE_ID_FILTER | CAN_MSG_OBJ_NO_FLAGS, 8);
    // 2# trig配置指令邮箱
    CAN_setupMessageObject2(CANB_BASE, 2, 0x2, CAN_MSG_FRAME_STD,
        CAN_MSG_OBJ_TYPE_RX, 0x7ffu, CAN_MSG_OBJ_USE_ID_FILTER | CAN_MSG_OBJ_NO_FLAGS, 8);
    // 3# 内存修改16b指令邮箱
    CAN_setupMessageObject2(CANB_BASE, 3, 0x3, CAN_MSG_FRAME_STD,
        CAN_MSG_OBJ_TYPE_RX, 0x7ffu, CAN_MSG_OBJ_USE_ID_FILTER | CAN_MSG_OBJ_NO_FLAGS, 8);
    // 4# 内存修改32b指令邮箱
    CAN_setupMessageObject2(CANB_BASE, 4, 0x4, CAN_MSG_FRAME_STD,
        CAN_MSG_OBJ_TYPE_RX, 0x7ffu, CAN_MSG_OBJ_USE_ID_FILTER | CAN_MSG_OBJ_NO_FLAGS, 8);
    // 5# 内存修改状态反馈邮箱
    CAN_setupMessageObject2(CANB_BASE, 5, 0x5, CAN_MSG_FRAME_STD,
        CAN_MSG_OBJ_TYPE_TX, 0, CAN_MSG_OBJ_NO_FLAGS, 8);
    // 6# 内存读取指令邮箱
    CAN_setupMessageObject2(CANB_BASE, 6, 0x6, CAN_MSG_FRAME_STD,
        CAN_MSG_OBJ_TYPE_RX, 0x7ffu, CAN_MSG_OBJ_USE_ID_FILTER | CAN_MSG_OBJ_NO_FLAGS, 8);

    // Initialize the transmit message object used for sending CAN messages.
    // CAN_setupMessageObject(CANB_BASE, 8, 0x8, CAN_MSG_FRAME_STD,
    //     CAN_MSG_OBJ_TYPE_TX, 0, CAN_MSG_OBJ_NO_FLAGS, 8);
    for (int ii = 8;ii < 16;ii++)
    {
        CAN_setupMessageObject2(CANB_BASE, ii, ii, CAN_MSG_FRAME_STD,
            CAN_MSG_OBJ_TYPE_TX, 0, CAN_MSG_OBJ_NO_FLAGS, 8);
    }

    // CAN_setupMessageObject(CANB_BASE, 3, 0x12,
    //     CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_RX, 0x12,
    //     (CAN_MSG_OBJ_USE_ID_FILTER | CAN_MSG_OBJ_NO_FLAGS), 8);

    // start
    CAN_startModule(CANB_BASE);

    return;
}

#pragma CODE_SECTION(cpu2initPIE, "ramfuncs2");
void cpu2initPIE()
{
    // 魔改 InitPieCtrl();

    // Disable Interrupts at the CPU level
    DINT;

    // Disable the PIE
    PieCtrlRegs.PIECTRL.bit.ENPIE = 0;

    // Clear all PIEIER registers:
    PieCtrlRegs.PIEIER1.all = 0;
    PieCtrlRegs.PIEIER2.all = 0;
    PieCtrlRegs.PIEIER3.all = 0;
    PieCtrlRegs.PIEIER4.all = 0;
    PieCtrlRegs.PIEIER5.all = 0;
    PieCtrlRegs.PIEIER6.all = 0;
    PieCtrlRegs.PIEIER7.all = 0;
    PieCtrlRegs.PIEIER8.all = 0;
    PieCtrlRegs.PIEIER9.all = 0;
    PieCtrlRegs.PIEIER10.all = 0;
    PieCtrlRegs.PIEIER11.all = 0;
    PieCtrlRegs.PIEIER12.all = 0;

    // Clear all PIEIFR registers:
    PieCtrlRegs.PIEIFR1.all = 0;
    PieCtrlRegs.PIEIFR2.all = 0;
    PieCtrlRegs.PIEIFR3.all = 0;
    PieCtrlRegs.PIEIFR4.all = 0;
    PieCtrlRegs.PIEIFR5.all = 0;
    PieCtrlRegs.PIEIFR6.all = 0;
    PieCtrlRegs.PIEIFR7.all = 0;
    PieCtrlRegs.PIEIFR8.all = 0;
    PieCtrlRegs.PIEIFR9.all = 0;
    PieCtrlRegs.PIEIFR10.all = 0;
    PieCtrlRegs.PIEIFR11.all = 0;
    PieCtrlRegs.PIEIFR12.all = 0;

    IER = 0x0000;
    IFR = 0x0000;

    // 魔改 InitPieVectTable();
    Uint16 i;
    Uint32* Dest = (void*)&PieVectTable;

    // Do not write over first 3 32-bit locations (these locations are
    // initialized by Boot ROM with boot variables)
    Dest = Dest + 3;

    EALLOW;
    for (i = 0; i < 221; i++)
    {
        *Dest++ = (Uint32)NOTUSED_ISR_CPU2_TYJ;
    }
    EDIS;

    //
    // Enable the PIE Vector Table
    //
    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;

    EALLOW;
    PieVectTable.IPC0_INT = &ipc_isr_cpu2; //function for IPC0(1.13)
    EDIS;

    //Enable group 1 interrupts
    IER |= M_INT1;

    // enable PIE interrupt
    PieCtrlRegs.PIEIER1.bit.INTx13 = 1;

    // Enables PIE to drive a pulse into the CPU
    PieCtrlRegs.PIEACK.all = 0xFFFF;

    EINT; // Enable Global interrupt INTM
    return;
}

#pragma CODE_SECTION(cpu2InitSysCtrl, "ramfuncs2");
void cpu2InitSysCtrl()
{
    // Turn on all peripherals basic
    EALLOW;

    CpuSysRegs.PCLKCR0.bit.CLA1 = 1;
    CpuSysRegs.PCLKCR0.bit.DMA = 1;
    CpuSysRegs.PCLKCR0.bit.CPUTIMER0 = 1;
    CpuSysRegs.PCLKCR0.bit.CPUTIMER1 = 1;
    CpuSysRegs.PCLKCR0.bit.CPUTIMER2 = 1;

    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;

    CpuSysRegs.PCLKCR2.bit.EPWM1 = 0;
    CpuSysRegs.PCLKCR2.bit.EPWM2 = 0;
    CpuSysRegs.PCLKCR2.bit.EPWM3 = 0;
    CpuSysRegs.PCLKCR2.bit.EPWM4 = 0;
    CpuSysRegs.PCLKCR2.bit.EPWM5 = 0;
    CpuSysRegs.PCLKCR2.bit.EPWM6 = 0;
    CpuSysRegs.PCLKCR2.bit.EPWM7 = 0;
    CpuSysRegs.PCLKCR2.bit.EPWM8 = 0;
    CpuSysRegs.PCLKCR2.bit.EPWM9 = 0;
    CpuSysRegs.PCLKCR2.bit.EPWM10 = 0;
    CpuSysRegs.PCLKCR2.bit.EPWM11 = 0;
    CpuSysRegs.PCLKCR2.bit.EPWM12 = 0;

    CpuSysRegs.PCLKCR3.bit.ECAP1 = 0;
    CpuSysRegs.PCLKCR3.bit.ECAP2 = 0;
    CpuSysRegs.PCLKCR3.bit.ECAP3 = 0;
    CpuSysRegs.PCLKCR3.bit.ECAP4 = 0;
    CpuSysRegs.PCLKCR3.bit.ECAP5 = 0;
    CpuSysRegs.PCLKCR3.bit.ECAP6 = 0;

    CpuSysRegs.PCLKCR4.bit.EQEP1 = 0;
    CpuSysRegs.PCLKCR4.bit.EQEP2 = 0;
    CpuSysRegs.PCLKCR4.bit.EQEP3 = 0;

    CpuSysRegs.PCLKCR6.bit.SD1 = 0;
    CpuSysRegs.PCLKCR6.bit.SD2 = 0;

    CpuSysRegs.PCLKCR7.bit.SCI_A = 0;
    CpuSysRegs.PCLKCR7.bit.SCI_B = 0;
    CpuSysRegs.PCLKCR7.bit.SCI_C = 0;
    CpuSysRegs.PCLKCR7.bit.SCI_D = 0;

    CpuSysRegs.PCLKCR8.bit.SPI_A = 0;
    CpuSysRegs.PCLKCR8.bit.SPI_B = 0;
    CpuSysRegs.PCLKCR8.bit.SPI_C = 0;

    CpuSysRegs.PCLKCR9.bit.I2C_A = 0;
    CpuSysRegs.PCLKCR9.bit.I2C_B = 0;

    CpuSysRegs.PCLKCR10.bit.CAN_A = 0;
    CpuSysRegs.PCLKCR10.bit.CAN_B = 0;

    CpuSysRegs.PCLKCR11.bit.McBSP_A = 0;
    CpuSysRegs.PCLKCR11.bit.McBSP_B = 0;


    CpuSysRegs.PCLKCR13.bit.ADC_A = 0;
    CpuSysRegs.PCLKCR13.bit.ADC_B = 0;
    CpuSysRegs.PCLKCR13.bit.ADC_C = 0;
    CpuSysRegs.PCLKCR13.bit.ADC_D = 0;

    CpuSysRegs.PCLKCR14.bit.CMPSS1 = 0;
    CpuSysRegs.PCLKCR14.bit.CMPSS2 = 0;
    CpuSysRegs.PCLKCR14.bit.CMPSS3 = 0;
    CpuSysRegs.PCLKCR14.bit.CMPSS4 = 0;
    CpuSysRegs.PCLKCR14.bit.CMPSS5 = 0;
    CpuSysRegs.PCLKCR14.bit.CMPSS6 = 0;
    CpuSysRegs.PCLKCR14.bit.CMPSS7 = 0;
    CpuSysRegs.PCLKCR14.bit.CMPSS8 = 0;

    CpuSysRegs.PCLKCR16.bit.DAC_A = 0;
    CpuSysRegs.PCLKCR16.bit.DAC_B = 0;
    CpuSysRegs.PCLKCR16.bit.DAC_C = 0;

    // 配置LSPCLK为200MHz
    ClkCfgRegs.LOSPCP.bit.LSPCLKDIV = 0;

    EDIS;

    return;
}

#pragma CODE_SECTION(cpu2InitTimers, "ramfuncs2");
void cpu2InitTimers()
{
    // tim_0 100ms 主循环低优先级任务，滴答计数

    // Initialize timer period, 200MHz * 1000us -> 1ms
    CpuTimer0Regs.PRD.all = (200ul * 1000ul * 100ul - 1);
    // Initialize pre-scale counter to divide by 1 (SYSCLKOUT):
    CpuTimer0Regs.TPR.all = 0;
    CpuTimer0Regs.TPRH.all = 0;

    CpuTimer0Regs.TCR.all = 0;
    // Make sure timer is stopped:
    CpuTimer0Regs.TCR.bit.TSS = 1;
    // Reload all counter register with period value:
    CpuTimer0Regs.TCR.bit.TRB = 1;

    // tim_1 1.5ms cdb轮询任务，滴答计数

    // Initialize timer period, 200MHz * 1500us -> 1.5ms
    CpuTimer1Regs.PRD.all = (200ul * 1500ul - 1);
    // Initialize pre-scale counter to divide by 1 (SYSCLKOUT):
    CpuTimer1Regs.TPR.all = 0;
    CpuTimer1Regs.TPRH.all = 0;

    CpuTimer1Regs.TCR.all = 0;
    // Make sure timer is stopped:
    CpuTimer1Regs.TCR.bit.TSS = 1;
    // Reload all counter register with period value:
    CpuTimer1Regs.TCR.bit.TRB = 1;
    return;
}

#pragma CODE_SECTION(cpu2InitCla, "ramfuncs2");
void cpu2InitCla()
{

    // 两个CLA使用一样的配置和内存布局
    memcpy_tyj((void*)0x8000ul, (void*)0xc000ul, (uint32_t)(4096ul));

    EALLOW;

    // Initialize and wait for CLA1ToCPUMsgRAM
    MemCfgRegs.MSGxINIT.bit.INIT_CLA1TOCPU = 1;
    while (MemCfgRegs.MSGxINITDONE.bit.INITDONE_CLA1TOCPU != 1) {};

    // Initialize and wait for CPUToCLA1MsgRAM
    MemCfgRegs.MSGxINIT.bit.INIT_CPUTOCLA1 = 1;
    while (MemCfgRegs.MSGxINITDONE.bit.INITDONE_CPUTOCLA1 != 1) {};

    // program space
    MemCfgRegs.LSxMSEL.bit.MSEL_LS0 = 1;
    MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS0 = 1;

    // data space
    MemCfgRegs.LSxMSEL.bit.MSEL_LS1 = 1;
    MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS1 = 0;

    EDIS;

    // Compute all CLA task vectors
    // On Type-1 CLAs the MVECT registers accept full 16-bit task addresses as
    // opposed to offsets used on older Type-0 CLAs
    EALLOW;
    // Cla1Regs.MVECT1 = (uint16_t)(&Cla1Task1_ad2s1210Read);

    // Enable the IACK instruction to start a task on CLA in software
    // for all  8 CLA tasks. Also, globally enable all 8 tasks (or a
    // subset of tasks) by writing to their respective bits in the MIER register

    // Cla1Regs.MCTL.bit.IACKE = 1;
    // Cla1Regs.MIER.all = 0x0;
    // Cla1Regs.MIER.bit.INT1 = 1;

    // Configure the vectors for the end-of-task interrupt for all 8 tasks
    // PieVectTable.CLA1_1_INT = &cla1Isr1;

    // 不使用中断机制
    PieCtrlRegs.PIEIER11.all = 0x0;
    // IER |= (M_INT11);

    EDIS;

    return;
}

#pragma CODE_SECTION(bsp_init_all2, "ramfuncs2");
void bsp_init_all2()
{
    DINT;

    // cla2 配置
    cpu2InitCla();
    // 定时器
    cpu2InitTimers();
    // CanB
    cpu2initCanB();
    // 清空IPC
    IpcRegs.IPCCLR.all = IpcRegs.IPCFLG.all;
    // 中断
    cpu2initPIE();

#ifdef _SCD_ENABLE
    // cdb调试支持
    cdb_init(&cdb1);
#endif

#ifdef _DEBUG
    ERTM;
    // Enable Global realtime interrupt DBGM
#endif

}

#pragma CODE_SECTION(bsp_start2, "ramfuncs2");
void bsp_start2()
{
    // 启动 tim_0 主循环低优先级任务滴答计时器
    CpuTimer0Regs.TCR.bit.TSS = 0;

    // 启动 tim_1 cdb轮询任务滴答计时器
    CpuTimer1Regs.TCR.bit.TSS = 0;

    return;
}
