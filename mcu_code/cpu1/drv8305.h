/*
 * drv8305.h
 *
 *  Created on: 2024年11月17日
 *      Author: t
 */

#ifndef MCU_CODE_CPU1_DRV8305_H_
#define MCU_CODE_CPU1_DRV8305_H_

#include "F28x_Project.h"
#include "stdint.h"

#define DRV8305_READ_CMD 0X8000UL
#define DRV8305_WRITE_CMD 0X0000UL
#define DRV8305_ADDR_UTIL(ADDR) (ADDR<<11)

struct DRV8305_struct
{
    uint16_t DRV8305RegReadVals[16];
    uint16_t readCnt;
    uint16_t ii;
};

// 配置参数

#pragma FUNC_ALWAYS_INLINE(drv8305_init)
static inline void drv8305_init(struct DRV8305_struct* hDRV8305)
{
    // 确定SPI号
    volatile struct SPI_REGS* SpixRegs = &SpiaRegs;

    // pin58 SPISIMO pin
    GPIO_SetupPinMux(58, GPIO_MUX_CPU1, 6);
    GPIO_SetupPinOptions(58, GPIO_OUTPUT, GPIO_PULLUP | GPIO_ASYNC);
    // pin59 SPISOMI pin
    GPIO_SetupPinMux(59, GPIO_MUX_CPU1, 6);
    GPIO_SetupPinOptions(59, GPIO_INPUT, GPIO_PULLUP | GPIO_ASYNC);
    // pin60 SPICLK pin
    GPIO_SetupPinMux(60, GPIO_MUX_CPU1, 6);
    GPIO_SetupPinOptions(60, GPIO_OUTPUT, GPIO_PULLUP | GPIO_ASYNC);
    // pin61 cs pin
    GPIO_SetupPinMux(61, GPIO_MUX_CPU1, 6);
    GPIO_SetupPinOptions(61, GPIO_OUTPUT, GPIO_PULLUP | GPIO_ASYNC);

    // spi外设配置
    // 打开 SPI-B 时钟
    EALLOW;
    CpuSysRegs.PCLKCR8.bit.SPI_A = 1;

    // 确保复位
    SpixRegs->SPICCR.bit.SPISWRESET = 0;
    // rising edge, no delay
    SpixRegs->SPICCR.bit.CLKPOLARITY = 0;
    SpixRegs->SPICTL.bit.CLK_PHASE = 0;

    // 关闭环回测试模式
    SpixRegs->SPICCR.bit.SPILBK = 0;
    // 使用最大16bit数据位数
    SpixRegs->SPICCR.bit.SPICHAR = 16 - 1;

    // 关闭overrun中断
    SpixRegs->SPICTL.bit.OVERRUNINTENA = 0;
    // 主机模式
    SpixRegs->SPICTL.bit.MASTER_SLAVE = 1;
    // 开启传输
    SpixRegs->SPICTL.bit.TALK = 1;
    // 关闭中断
    SpixRegs->SPICTL.bit.SPIINTENA = 0;

    // 波特率2MHz
    SpixRegs->SPIBRR.bit.SPI_BIT_RATE = 99;

    // 使spi外设退出复位状态
    SpixRegs->SPICCR.bit.SPISWRESET = 1;
    EDIS;

    hDRV8305->readCnt = 0;
    hDRV8305->ii = 0;

    const uint16_t DRV8305RegCfgVals[] = {
        DRV8305_READ_CMD | DRV8305_ADDR_UTIL(0X1UL) | 0b00000000000ul,
        DRV8305_READ_CMD | DRV8305_ADDR_UTIL(0X2UL) | 0b00000000000ul,
        DRV8305_READ_CMD | DRV8305_ADDR_UTIL(0X3UL) | 0b00000000000ul,
        DRV8305_READ_CMD | DRV8305_ADDR_UTIL(0X4UL) | 0b00000000000ul,
        DRV8305_WRITE_CMD | DRV8305_ADDR_UTIL(0X5UL) | 0b01001000100ul,
        DRV8305_WRITE_CMD | DRV8305_ADDR_UTIL(0X6UL) | 0b01001000100ul,
        DRV8305_WRITE_CMD | DRV8305_ADDR_UTIL(0X7UL) | 0b01001000110ul,
        DRV8305_WRITE_CMD | DRV8305_ADDR_UTIL(0X9UL) | 0b00000000010ul,
        DRV8305_WRITE_CMD | DRV8305_ADDR_UTIL(0XaUL) | 0b00000000000ul,
        DRV8305_WRITE_CMD | DRV8305_ADDR_UTIL(0XbUL) | 0b00100001010ul,
        DRV8305_WRITE_CMD | DRV8305_ADDR_UTIL(0XcUL) | 0b00000110000ul,
    };

    const int allCnt = sizeof(DRV8305RegCfgVals) / sizeof(uint16_t);
    for (;hDRV8305->ii < allCnt;hDRV8305->ii++)
    {
        volatile uint16_t dummy = SpixRegs->SPIRXBUF;
        SpixRegs->SPITXBUF = DRV8305RegCfgVals[hDRV8305->ii];
        while (SpixRegs->SPISTS.bit.INT_FLAG == 0)
        {
            // wait for finish
        }
        hDRV8305->DRV8305RegReadVals[hDRV8305->ii] = SpixRegs->SPIRXBUF;
        hDRV8305->readCnt++;
        DELAY_US(1);
    }
    hDRV8305->ii = 0;

    return;
}

#pragma FUNC_ALWAYS_INLINE(drv8305_init2)
static inline void drv8305_init2(struct DRV8305_struct* hDRV8305)
{
    // 确定SPI号
    volatile struct SPI_REGS* SpixRegs = &SpibRegs;

    // pin63 SPISIMO pin
    GPIO_SetupPinMux(63, GPIO_MUX_CPU1, 15);
    GPIO_SetupPinOptions(63, GPIO_OUTPUT, GPIO_PULLUP | GPIO_ASYNC);
    // pin64 SPISOMI pin
    GPIO_SetupPinMux(64, GPIO_MUX_CPU1, 15);
    GPIO_SetupPinOptions(64, GPIO_INPUT, GPIO_PULLUP | GPIO_ASYNC);
    // pin65 SPICLK pin
    GPIO_SetupPinMux(65, GPIO_MUX_CPU1, 15);
    GPIO_SetupPinOptions(65, GPIO_OUTPUT, GPIO_PULLUP | GPIO_ASYNC);
    // pin66 cs pin
    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 15);
    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PULLUP | GPIO_ASYNC);

    // spi外设配置
    // 打开 SPI-B 时钟
    EALLOW;
    CpuSysRegs.PCLKCR8.bit.SPI_B = 1;

    // 确保复位
    SpixRegs->SPICCR.bit.SPISWRESET = 0;
    // rising edge, no delay
    SpixRegs->SPICCR.bit.CLKPOLARITY = 0;
    SpixRegs->SPICTL.bit.CLK_PHASE = 0;

    // 关闭环回测试模式
    SpixRegs->SPICCR.bit.SPILBK = 0;
    // 使用最大16bit数据位数
    SpixRegs->SPICCR.bit.SPICHAR = 16 - 1;

    // 关闭overrun中断
    SpixRegs->SPICTL.bit.OVERRUNINTENA = 0;
    // 主机模式
    SpixRegs->SPICTL.bit.MASTER_SLAVE = 1;
    // 开启传输
    SpixRegs->SPICTL.bit.TALK = 1;
    // 关闭中断
    SpixRegs->SPICTL.bit.SPIINTENA = 0;

    // 波特率2MHz
    SpixRegs->SPIBRR.bit.SPI_BIT_RATE = 99;

    // 使spi外设退出复位状态
    SpixRegs->SPICCR.bit.SPISWRESET = 1;
    EDIS;

    hDRV8305->readCnt = 0;
    hDRV8305->ii = 0;

    const uint16_t DRV8305RegCfgVals[] = {
        DRV8305_READ_CMD | DRV8305_ADDR_UTIL(0X1UL) | 0b00000000000ul,
        DRV8305_READ_CMD | DRV8305_ADDR_UTIL(0X2UL) | 0b00000000000ul,
        DRV8305_READ_CMD | DRV8305_ADDR_UTIL(0X3UL) | 0b00000000000ul,
        DRV8305_READ_CMD | DRV8305_ADDR_UTIL(0X4UL) | 0b00000000000ul,
        DRV8305_WRITE_CMD | DRV8305_ADDR_UTIL(0X5UL) | 0b01001000100ul,
        DRV8305_WRITE_CMD | DRV8305_ADDR_UTIL(0X6UL) | 0b01001000100ul,
        DRV8305_WRITE_CMD | DRV8305_ADDR_UTIL(0X7UL) | 0b01001000110ul,
        DRV8305_WRITE_CMD | DRV8305_ADDR_UTIL(0X9UL) | 0b00000000010ul,
        DRV8305_WRITE_CMD | DRV8305_ADDR_UTIL(0XaUL) | 0b00000000000ul,
        DRV8305_WRITE_CMD | DRV8305_ADDR_UTIL(0XbUL) | 0b00100001010ul,
        DRV8305_WRITE_CMD | DRV8305_ADDR_UTIL(0XcUL) | 0b00000110000ul,
    };

    const int allCnt = sizeof(DRV8305RegCfgVals) / sizeof(uint16_t);
    for (;hDRV8305->ii < allCnt;hDRV8305->ii++)
    {
        volatile uint16_t dummy = SpixRegs->SPIRXBUF;
        SpixRegs->SPITXBUF = DRV8305RegCfgVals[hDRV8305->ii];
        while (SpixRegs->SPISTS.bit.INT_FLAG == 0)
        {
            // wait for finish
        }
        hDRV8305->DRV8305RegReadVals[hDRV8305->ii] = SpixRegs->SPIRXBUF;
        hDRV8305->readCnt++;
        DELAY_US(1);
    }
    hDRV8305->ii = 0;

    return;
}

#pragma FUNC_ALWAYS_INLINE(drv8305_idleErrCheck)
static inline void drv8305_idleErrCheck(struct DRV8305_struct* hDRV8305)
{
    const uint16_t DRV8305RegReadTars[] = {
        DRV8305_READ_CMD | DRV8305_ADDR_UTIL(0X1UL) | 0b00000000000ul,
        DRV8305_READ_CMD | DRV8305_ADDR_UTIL(0X2UL) | 0b00000000000ul,
        DRV8305_READ_CMD | DRV8305_ADDR_UTIL(0X3UL) | 0b00000000000ul,
        DRV8305_READ_CMD | DRV8305_ADDR_UTIL(0X4UL) | 0b00000000000ul,
        DRV8305_READ_CMD | DRV8305_ADDR_UTIL(0X5UL) | 0b01001000100ul,
        DRV8305_READ_CMD | DRV8305_ADDR_UTIL(0X6UL) | 0b01001000100ul,
        DRV8305_READ_CMD | DRV8305_ADDR_UTIL(0X7UL) | 0b01001000110ul,
        DRV8305_READ_CMD | DRV8305_ADDR_UTIL(0X9UL) | 0b00000000010ul,
        DRV8305_READ_CMD | DRV8305_ADDR_UTIL(0XaUL) | 0b00000000000ul,
        DRV8305_READ_CMD | DRV8305_ADDR_UTIL(0XbUL) | 0b00100001010ul,
        DRV8305_READ_CMD | DRV8305_ADDR_UTIL(0XcUL) | 0b00000110000ul,
    };

    const int allCnt = sizeof(DRV8305RegReadTars) / sizeof(uint16_t);

    if (hDRV8305->ii >= allCnt)
    {
        return;
    }

    // 确定SPI号
    volatile struct SPI_REGS* SpixRegs = &SpiaRegs;

    SpixRegs->SPITXBUF = DRV8305RegReadTars[hDRV8305->ii];
    while (SpixRegs->SPISTS.bit.INT_FLAG == 0)
    {
        // wait for finish
    }
    hDRV8305->DRV8305RegReadVals[hDRV8305->ii] = SpixRegs->SPIRXBUF;
    hDRV8305->readCnt++;
    hDRV8305->ii++;
}

#pragma FUNC_ALWAYS_INLINE(drv8305_idleErrCheck2)
static inline void drv8305_idleErrCheck2(struct DRV8305_struct* hDRV8305)
{
    const uint16_t DRV8305RegReadTars[] = {
        DRV8305_READ_CMD | DRV8305_ADDR_UTIL(0X1UL) | 0b00000000000ul,
        DRV8305_READ_CMD | DRV8305_ADDR_UTIL(0X2UL) | 0b00000000000ul,
        DRV8305_READ_CMD | DRV8305_ADDR_UTIL(0X3UL) | 0b00000000000ul,
        DRV8305_READ_CMD | DRV8305_ADDR_UTIL(0X4UL) | 0b00000000000ul,
        DRV8305_READ_CMD | DRV8305_ADDR_UTIL(0X5UL) | 0b01001000100ul,
        DRV8305_READ_CMD | DRV8305_ADDR_UTIL(0X6UL) | 0b01001000100ul,
        DRV8305_READ_CMD | DRV8305_ADDR_UTIL(0X7UL) | 0b01001000110ul,
        DRV8305_READ_CMD | DRV8305_ADDR_UTIL(0X9UL) | 0b00000000010ul,
        DRV8305_READ_CMD | DRV8305_ADDR_UTIL(0XaUL) | 0b00000000000ul,
        DRV8305_READ_CMD | DRV8305_ADDR_UTIL(0XbUL) | 0b00100001010ul,
        DRV8305_READ_CMD | DRV8305_ADDR_UTIL(0XcUL) | 0b00000110000ul,
    };

    const int allCnt = sizeof(DRV8305RegReadTars) / sizeof(uint16_t);

    if (hDRV8305->ii >= allCnt)
    {
        return;
    }

    // 确定SPI号
    volatile struct SPI_REGS* SpixRegs = &SpibRegs;

    SpixRegs->SPITXBUF = DRV8305RegReadTars[hDRV8305->ii];
    while (SpixRegs->SPISTS.bit.INT_FLAG == 0)
    {
        // wait for finish
    }
    hDRV8305->DRV8305RegReadVals[hDRV8305->ii] = SpixRegs->SPIRXBUF;
    hDRV8305->readCnt++;
    hDRV8305->ii++;
}

#endif /* MCU_CODE_CPU1_DRV8305_H_ */
