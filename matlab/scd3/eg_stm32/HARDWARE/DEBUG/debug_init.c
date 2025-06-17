#include "scd_inc.h"

#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_usart.h"
#include "stm32f4xx_ll_dma.h"

const uint8_t APBPrescTable[8] = { 0, 0, 0, 0, 1, 2, 3, 4 };

#define USART_REV_BUFF_SIZE 4
#define USART_SEND_BUFF_SIZE 8

uint8_t scd_rev_2buffer[2][USART_REV_BUFF_SIZE];
uint8_t scd_send_2buffer[2][USART_SEND_BUFF_SIZE];

void uart1Init(uint32_t baud);

//初始化IO 串口1 DMA; baud:波特率
void DMAOfUart1_DBG_init(uint32_t baud)
{
    /* Init with LL driver */
    /* DMA controller clock enable */
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);

    /* USART1 DMA Init */

    /* USART1_TX Init */
    LL_DMA_SetChannelSelection(DMA2, LL_DMA_STREAM_7, LL_DMA_CHANNEL_4);
    LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_7, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
    LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_7, LL_DMA_PRIORITY_MEDIUM);
    LL_DMA_SetMode(DMA2, LL_DMA_STREAM_7, LL_DMA_MODE_CIRCULAR);
    LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_7, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_7, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_7, LL_DMA_PDATAALIGN_BYTE);
    LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_7, LL_DMA_MDATAALIGN_BYTE);
    LL_DMA_DisableFifoMode(DMA2, LL_DMA_STREAM_7);

    /* USART1_RX Init */
    LL_DMA_SetChannelSelection(DMA2, LL_DMA_STREAM_2, LL_DMA_CHANNEL_4);
    LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_2, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_2, LL_DMA_PRIORITY_MEDIUM);
    LL_DMA_SetMode(DMA2, LL_DMA_STREAM_2, LL_DMA_MODE_CIRCULAR);
    LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_2, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_2, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_2, LL_DMA_PDATAALIGN_BYTE);
    LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_2, LL_DMA_MDATAALIGN_BYTE);
    LL_DMA_DisableFifoMode(DMA2, LL_DMA_STREAM_2);

    /* DMA2_Stream2_IRQn interrupt configuration */
    NVIC_SetPriority(DMA2_Stream2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 4, 0));
    NVIC_EnableIRQ(DMA2_Stream2_IRQn);
    /* DMA2_Stream7_IRQn interrupt configuration */
    NVIC_SetPriority(DMA2_Stream7_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 4, 1));
    NVIC_EnableIRQ(DMA2_Stream7_IRQn);

    // 初始化 debug 协议
    scd_init_1();

    // fill the send buffer
    for (int j = 0; j < 2; j++)
    {
        for (int i = 0; i < USART_SEND_BUFF_SIZE; i++)
        {
            scd_send_2buffer[j][i] = scd_send1Byte(&scd_1);
        }
    }

    // rev 2 buff dma config
    LL_DMA_EnableDoubleBufferMode(DMA2, LL_DMA_STREAM_2);
    LL_DMA_SetPeriphAddress(DMA2, LL_DMA_STREAM_2, LL_USART_DMA_GetRegAddr(USART1));
    LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_2, (uint32_t)scd_rev_2buffer[0]);
    LL_DMA_SetMemory1Address(DMA2, LL_DMA_STREAM_2, (uint32_t)scd_rev_2buffer[1]);
    LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_2, USART_REV_BUFF_SIZE);
    LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_2);

    // send 2 buff dma config
    LL_DMA_EnableDoubleBufferMode(DMA2, LL_DMA_STREAM_7);
    LL_DMA_SetPeriphAddress(DMA2, LL_DMA_STREAM_7, LL_USART_DMA_GetRegAddr(USART1));
    LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_7, (uint32_t)scd_send_2buffer[0]);
    LL_DMA_SetMemory1Address(DMA2, LL_DMA_STREAM_7, (uint32_t)scd_send_2buffer[1]);
    LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_7, USART_SEND_BUFF_SIZE);
    LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_7);

    // 初始化串口
    uart1Init(baud);

    // enable usart dma trig
    LL_USART_EnableDMAReq_RX(USART1);
    LL_USART_EnableDMAReq_TX(USART1);

    // enable dma stream
    LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_2);
    LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_7);
}

void DMA2_Stream2_IRQHandler(void)
{
    // usart1 rx
    if (LL_DMA_IsActiveFlag_TC2(DMA2))
    {
        LL_DMA_ClearFlag_TC2(DMA2);
        if (LL_DMA_GetCurrentTargetMem(DMA2, LL_DMA_STREAM_2) == LL_DMA_CURRENTTARGETMEM0)
        {
            // at mem0, process mem1
            for (int i = 0; i < USART_REV_BUFF_SIZE; i++)
            {
                SCD_Rev1Byte(&scd_1, scd_rev_2buffer[1][i]);
            }
        }
        else
        {
            // at mem1, process mem0
            for (int i = 0; i < USART_REV_BUFF_SIZE; i++)
            {
                SCD_Rev1Byte(&scd_1, scd_rev_2buffer[0][i]);
            }
        }
    }
    else
    {
        while (1)
        {
            // error!
        }
    }
}

/**
  * @brief This function handles DMA2 stream7 global interrupt.
  */
void DMA2_Stream7_IRQHandler(void)
{
    // usart1 tx
    if (LL_DMA_IsActiveFlag_TC7(DMA2))
    {
        LL_DMA_ClearFlag_TC7(DMA2);
        if (LL_DMA_GetCurrentTargetMem(DMA2, LL_DMA_STREAM_7) == LL_DMA_CURRENTTARGETMEM0)
        {
            // at mem0, process mem1
            for (int i = 0; i < USART_SEND_BUFF_SIZE; i++)
            {
                scd_send_2buffer[1][i] = scd_send1Byte(&scd_1);
            }
        }
        else
        {
            // at mem1, process mem0
            for (int i = 0; i < USART_SEND_BUFF_SIZE; i++)
            {
                scd_send_2buffer[0][i] = scd_send1Byte(&scd_1);
            }
        }
    }
    else
    {
        while (1)
        {
            // error!
        }
    }
}
