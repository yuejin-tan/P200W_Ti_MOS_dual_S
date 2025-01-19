/*
 * bsp_inline.h
 *
 *  Created on: 2022年6月23日
 *      Author: tyj
 */

#ifndef INCX_BSP_INLINE_H_
#define INCX_BSP_INLINE_H_

#include "F28x_Project.h"
#include "stdint.h"

 // HW Fault
#pragma FUNC_ALWAYS_INLINE(bsp_GPIO_ifHWFaultSigSet)
static inline int bsp_GPIO_ifHWFaultSigSet()
{
    if (GpioDataRegs.GPADAT.bit.GPIO19)
    {
        return 0;
    }
    else
    {
        return 1;
    }
}

#pragma FUNC_ALWAYS_INLINE(bsp_GPIO_ifHWFaultSigSet2)
static inline int bsp_GPIO_ifHWFaultSigSet2()
{
    if (GpioDataRegs.GPEDAT.bit.GPIO139)
    {
        return 0;
    }
    else
    {
        return 1;
    }
}

// POWER EN
#pragma FUNC_ALWAYS_INLINE(bsp_POWER_EN_CH1)
static inline void bsp_POWER_EN_CH1(int level)
{
    // 高使能
    if (level)
    {
        GpioDataRegs.GPDSET.bit.GPIO124 = 1;
    }
    else
    {
        GpioDataRegs.GPDCLEAR.bit.GPIO124 = 1;
    }
}

#pragma FUNC_ALWAYS_INLINE(bsp_POWER_EN_CH2)
static inline void bsp_POWER_EN_CH2(int level)
{
    // 高使能
    if (level)
    {
        GpioDataRegs.GPASET.bit.GPIO26 = 1;
    }
    else
    {
        GpioDataRegs.GPACLEAR.bit.GPIO26 = 1;
    }
}

// 外部急停信号
#pragma FUNC_ALWAYS_INLINE(bsp_GPIO_ifStopSigSet)
static inline int bsp_GPIO_ifStopSigSet()
{
    if (GpioDataRegs.GPADAT.bit.GPIO14)
    {
        return 0;
    }
    else
    {
        return 1;
    }
}

// LED D9 RED
#pragma FUNC_ALWAYS_INLINE(bsp_LED_D9_RED_CTRL)
static inline void bsp_LED_D9_RED_CTRL(int sta)
{
    if (sta == 1)
    {
        GpioDataRegs.GPBCLEAR.bit.GPIO34 = 1;
    }
    else if (sta == 0)
    {
        GpioDataRegs.GPBSET.bit.GPIO34 = 1;
    }
    else
    {
        GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;
    }
}

// ePWM

#include "SVPWM.h"

// group1
#pragma FUNC_ALWAYS_INLINE(bsp_epwm_ch1_is_up_cnt)
static inline int16_t bsp_epwm_ch1_is_up_cnt()
{
    return EPwm1Regs.TBSTS.bit.CTRDIR;
}

#pragma FUNC_ALWAYS_INLINE(bsp_epwm_ch1_reg_set)
static inline void bsp_epwm_ch1_reg_set(struct SVPWM_struct* hSVPWM)
{
    // 由于控制频率翻倍，但单采单更，故可以从原始的结果上改
    {
        uint16_t cmpu = __min(__max(hSVPWM->epwmU, 0), vPWM_LOAD_VAL_I);
        uint16_t cmpu_b = cmpu >> 1;
        uint16_t cmpu_a = cmpu - cmpu_b;
        EPwm1Regs.CMPA.bit.CMPA = cmpu_a;
        EPwm1Regs.CMPB.bit.CMPB = cmpu_b;
    }
    {
        uint16_t cmpv = __min(__max(hSVPWM->epwmV, 0), vPWM_LOAD_VAL_I);
        uint16_t cmpv_b = cmpv >> 1;
        uint16_t cmpv_a = cmpv - cmpv_b;
        EPwm2Regs.CMPA.bit.CMPA = cmpv_a;
        EPwm2Regs.CMPB.bit.CMPB = cmpv_b;
    }
    {
        uint16_t cmpw = __min(__max(hSVPWM->epwmW, 0), vPWM_LOAD_VAL_I);
        uint16_t cmpw_b = cmpw >> 1;
        uint16_t cmpw_a = cmpw - cmpw_b;
        EPwm3Regs.CMPA.bit.CMPA = cmpw_a;
        EPwm3Regs.CMPB.bit.CMPB = cmpw_b;
    }
}

#pragma FUNC_ALWAYS_INLINE(bsp_epwm_ch1_reg_init)
static inline void bsp_epwm_ch1_reg_init()
{
    // 由于控制频率翻倍但单采单更，故可以从原始的结果上改
    int16_t new_cmp_val = vPWM_CMP_DEFAILT_VAL_I >> 1;
    EPwm1Regs.CMPA.bit.CMPA = new_cmp_val;
    EPwm1Regs.CMPB.bit.CMPB = new_cmp_val;
    EPwm2Regs.CMPA.bit.CMPA = new_cmp_val;
    EPwm2Regs.CMPB.bit.CMPB = new_cmp_val;
    EPwm3Regs.CMPA.bit.CMPA = new_cmp_val;
    EPwm3Regs.CMPB.bit.CMPB = new_cmp_val;
}

#define TYJ_AQSFRC_LOW 0b01
#define TYJ_AQSFRC_HIGH 0b10

#pragma FUNC_ALWAYS_INLINE(bsp_epwm_ch1_on)
static inline void bsp_epwm_ch1_on()
{
    // 开启 IGBT
    EPwm1Regs.AQCSFRC.bit.CSFA = 0;
    EPwm1Regs.AQCSFRC.bit.CSFB = 0;

    EPwm2Regs.AQCSFRC.bit.CSFA = 0;
    EPwm2Regs.AQCSFRC.bit.CSFB = 0;

    EPwm3Regs.AQCSFRC.bit.CSFA = 0;
    EPwm3Regs.AQCSFRC.bit.CSFB = 0;
}

#pragma FUNC_ALWAYS_INLINE(bsp_epwm_ch1_off)
static inline void bsp_epwm_ch1_off()
{
    // 关闭 IGBT
    EPwm1Regs.AQCSFRC.bit.CSFA = TYJ_AQSFRC_LOW;
    EPwm1Regs.AQCSFRC.bit.CSFB = TYJ_AQSFRC_HIGH;

    EPwm2Regs.AQCSFRC.bit.CSFA = TYJ_AQSFRC_LOW;
    EPwm2Regs.AQCSFRC.bit.CSFB = TYJ_AQSFRC_HIGH;

    EPwm3Regs.AQCSFRC.bit.CSFA = TYJ_AQSFRC_LOW;
    EPwm3Regs.AQCSFRC.bit.CSFB = TYJ_AQSFRC_HIGH;
}

#pragma FUNC_ALWAYS_INLINE(bsp_epwm_ch1_LSSC)
static inline void bsp_epwm_ch1_LSSC()
{
    // 开启 下桥 IGBT
    EPwm1Regs.AQCSFRC.bit.CSFA = TYJ_AQSFRC_LOW;
    EPwm1Regs.AQCSFRC.bit.CSFB = TYJ_AQSFRC_LOW;

    EPwm2Regs.AQCSFRC.bit.CSFA = TYJ_AQSFRC_LOW;
    EPwm2Regs.AQCSFRC.bit.CSFB = TYJ_AQSFRC_LOW;

    EPwm3Regs.AQCSFRC.bit.CSFA = TYJ_AQSFRC_LOW;
    EPwm3Regs.AQCSFRC.bit.CSFB = TYJ_AQSFRC_LOW;
}

// mode : 0@DOWN 1@UP; bit0 A, bit1 B, bit2 C
#pragma FUNC_ALWAYS_INLINE(bsp_epwm_ch1_LSSC2)
static inline void bsp_epwm_ch1_LSSC2(int16_t mode)
{
    if (mode & 0x1)
    {
        EPwm1Regs.AQCSFRC.bit.CSFA = TYJ_AQSFRC_HIGH;
        EPwm1Regs.AQCSFRC.bit.CSFB = TYJ_AQSFRC_HIGH;
    }
    else
    {
        EPwm1Regs.AQCSFRC.bit.CSFA = TYJ_AQSFRC_LOW;
        EPwm1Regs.AQCSFRC.bit.CSFB = TYJ_AQSFRC_LOW;
    }

    if (mode & 0x2)
    {
        EPwm2Regs.AQCSFRC.bit.CSFA = TYJ_AQSFRC_HIGH;
        EPwm2Regs.AQCSFRC.bit.CSFB = TYJ_AQSFRC_HIGH;
    }
    else
    {
        EPwm2Regs.AQCSFRC.bit.CSFA = TYJ_AQSFRC_LOW;
        EPwm2Regs.AQCSFRC.bit.CSFB = TYJ_AQSFRC_LOW;
    }

    if (mode & 0x4)
    {
        EPwm3Regs.AQCSFRC.bit.CSFA = TYJ_AQSFRC_HIGH;
        EPwm3Regs.AQCSFRC.bit.CSFB = TYJ_AQSFRC_HIGH;
    }
    else
    {
        EPwm3Regs.AQCSFRC.bit.CSFA = TYJ_AQSFRC_LOW;
        EPwm3Regs.AQCSFRC.bit.CSFB = TYJ_AQSFRC_LOW;
    }
}


// group2
#pragma FUNC_ALWAYS_INLINE(bsp_epwm_ch2_is_up_cnt)
static inline int16_t bsp_epwm_ch2_is_up_cnt()
{
    return EPwm4Regs.TBSTS.bit.CTRDIR;
}

#pragma FUNC_ALWAYS_INLINE(bsp_epwm_ch2_reg_set)
static inline void bsp_epwm_ch2_reg_set(struct SVPWM_struct* hSVPWM)
{
    // 由于控制频率翻倍，但单采单更，故可以从原始的结果上改
    {
        uint16_t cmpu = __min(__max(hSVPWM->epwmU, 0), vPWM_LOAD_VAL_I);
        uint16_t cmpu_b = cmpu >> 1;
        uint16_t cmpu_a = cmpu - cmpu_b;
        EPwm4Regs.CMPA.bit.CMPA = cmpu_a;
        EPwm4Regs.CMPB.bit.CMPB = cmpu_b;
    }
    {
        uint16_t cmpv = __min(__max(hSVPWM->epwmV, 0), vPWM_LOAD_VAL_I);
        uint16_t cmpv_b = cmpv >> 1;
        uint16_t cmpv_a = cmpv - cmpv_b;
        EPwm5Regs.CMPA.bit.CMPA = cmpv_a;
        EPwm5Regs.CMPB.bit.CMPB = cmpv_b;
    }
    {
        uint16_t cmpw = __min(__max(hSVPWM->epwmW, 0), vPWM_LOAD_VAL_I);
        uint16_t cmpw_b = cmpw >> 1;
        uint16_t cmpw_a = cmpw - cmpw_b;
        EPwm6Regs.CMPA.bit.CMPA = cmpw_a;
        EPwm6Regs.CMPB.bit.CMPB = cmpw_b;
    }
}

#pragma FUNC_ALWAYS_INLINE(bsp_epwm_ch2_reg_init)
static inline void bsp_epwm_ch2_reg_init()
{
    // 由于控制频率翻倍但单采单更，故可以从ch1的结果上改
    int16_t new_cmp_val = vPWM_CMP_DEFAILT_VAL_I >> 1;
    EPwm4Regs.CMPA.bit.CMPA = new_cmp_val;
    EPwm4Regs.CMPB.bit.CMPB = new_cmp_val;
    EPwm5Regs.CMPA.bit.CMPA = new_cmp_val;
    EPwm5Regs.CMPB.bit.CMPB = new_cmp_val;
    EPwm6Regs.CMPA.bit.CMPA = new_cmp_val;
    EPwm6Regs.CMPB.bit.CMPB = new_cmp_val;
}

#pragma FUNC_ALWAYS_INLINE(bsp_epwm_ch2_on)
static inline void bsp_epwm_ch2_on()
{
    // 开启 IGBT
    EPwm4Regs.AQCSFRC.bit.CSFA = 0;
    EPwm4Regs.AQCSFRC.bit.CSFB = 0;

    EPwm5Regs.AQCSFRC.bit.CSFA = 0;
    EPwm5Regs.AQCSFRC.bit.CSFB = 0;

    EPwm6Regs.AQCSFRC.bit.CSFA = 0;
    EPwm6Regs.AQCSFRC.bit.CSFB = 0;
}

#pragma FUNC_ALWAYS_INLINE(bsp_epwm_ch2_off)
static inline void bsp_epwm_ch2_off()
{
    // 关闭 IGBT
    EPwm4Regs.AQCSFRC.bit.CSFA = TYJ_AQSFRC_LOW;
    EPwm4Regs.AQCSFRC.bit.CSFB = TYJ_AQSFRC_HIGH;

    EPwm5Regs.AQCSFRC.bit.CSFA = TYJ_AQSFRC_LOW;
    EPwm5Regs.AQCSFRC.bit.CSFB = TYJ_AQSFRC_HIGH;

    EPwm6Regs.AQCSFRC.bit.CSFA = TYJ_AQSFRC_LOW;
    EPwm6Regs.AQCSFRC.bit.CSFB = TYJ_AQSFRC_HIGH;
}

#pragma FUNC_ALWAYS_INLINE(bsp_epwm_ch2_LSSC)
static inline void bsp_epwm_ch2_LSSC()
{
    // 开启 下桥 IGBT
    EPwm4Regs.AQCSFRC.bit.CSFA = TYJ_AQSFRC_LOW;
    EPwm4Regs.AQCSFRC.bit.CSFB = TYJ_AQSFRC_LOW;

    EPwm5Regs.AQCSFRC.bit.CSFA = TYJ_AQSFRC_LOW;
    EPwm5Regs.AQCSFRC.bit.CSFB = TYJ_AQSFRC_LOW;

    EPwm6Regs.AQCSFRC.bit.CSFA = TYJ_AQSFRC_LOW;
    EPwm6Regs.AQCSFRC.bit.CSFB = TYJ_AQSFRC_LOW;
}

// mode : 0@DOWN 1@UP; bit0 A, bit1 B, bit2 C
#pragma FUNC_ALWAYS_INLINE(bsp_epwm_ch2_LSSC2)
static inline void bsp_epwm_ch2_LSSC2(int16_t mode)
{
    if (mode & 0x1)
    {
        EPwm4Regs.AQCSFRC.bit.CSFA = TYJ_AQSFRC_HIGH;
        EPwm4Regs.AQCSFRC.bit.CSFB = TYJ_AQSFRC_HIGH;
    }
    else
    {
        EPwm4Regs.AQCSFRC.bit.CSFA = TYJ_AQSFRC_LOW;
        EPwm4Regs.AQCSFRC.bit.CSFB = TYJ_AQSFRC_LOW;
    }

    if (mode & 0x2)
    {
        EPwm5Regs.AQCSFRC.bit.CSFA = TYJ_AQSFRC_HIGH;
        EPwm5Regs.AQCSFRC.bit.CSFB = TYJ_AQSFRC_HIGH;
    }
    else
    {
        EPwm5Regs.AQCSFRC.bit.CSFA = TYJ_AQSFRC_LOW;
        EPwm5Regs.AQCSFRC.bit.CSFB = TYJ_AQSFRC_LOW;
    }

    if (mode & 0x4)
    {
        EPwm6Regs.AQCSFRC.bit.CSFA = TYJ_AQSFRC_HIGH;
        EPwm6Regs.AQCSFRC.bit.CSFB = TYJ_AQSFRC_HIGH;
    }
    else
    {
        EPwm6Regs.AQCSFRC.bit.CSFA = TYJ_AQSFRC_LOW;
        EPwm6Regs.AQCSFRC.bit.CSFB = TYJ_AQSFRC_LOW;
    }
}

// adc A

#pragma FUNC_ALWAYS_INLINE(bsp_get_CH1_Iw_adcRaw)
static inline int16_t bsp_get_CH1_Iw_adcRaw(void)
{
    return AdcaResultRegs.ADCRESULT0;
}

#pragma FUNC_ALWAYS_INLINE(bsp_get_CH1_Va_adcRaw)
static inline int16_t bsp_get_CH1_Va_adcRaw(void)
{
    return AdcaResultRegs.ADCRESULT1;
}

#pragma FUNC_ALWAYS_INLINE(bsp_get_CH1_Udc_adcRaw)
static inline int16_t bsp_get_CH1_Udc_adcRaw(void)
{
    return AdcaResultRegs.ADCRESULT2;
}

#pragma FUNC_ALWAYS_INLINE(bsp_get_CH2_Iw_adcRaw)
static inline int16_t bsp_get_CH2_Iw_adcRaw(void)
{
    return AdcaResultRegs.ADCRESULT3;
}

#pragma FUNC_ALWAYS_INLINE(bsp_get_CH2_Va_adcRaw)
static inline int16_t bsp_get_CH2_Va_adcRaw(void)
{
    return AdcaResultRegs.ADCRESULT4;
}

#pragma FUNC_ALWAYS_INLINE(bsp_get_CH2_Udc_adcRaw)
static inline int16_t bsp_get_CH2_Udc_adcRaw(void)
{
    return AdcaResultRegs.ADCRESULT5;
}


// adc B

#pragma FUNC_ALWAYS_INLINE(bsp_get_CH1_Iv_adcRaw)
static inline int16_t bsp_get_CH1_Iv_adcRaw(void)
{
    return AdcbResultRegs.ADCRESULT0;
}

#pragma FUNC_ALWAYS_INLINE(bsp_get_CH1_Vc_adcRaw)
static inline int16_t bsp_get_CH1_Vc_adcRaw(void)
{
    return AdcbResultRegs.ADCRESULT1;
}

#pragma FUNC_ALWAYS_INLINE(bsp_get_CH2_Iv_adcRaw)
static inline int16_t bsp_get_CH2_Iv_adcRaw(void)
{
    return AdcbResultRegs.ADCRESULT2;
}

#pragma FUNC_ALWAYS_INLINE(bsp_get_CH2_Vc_adcRaw)
static inline int16_t bsp_get_CH2_Vc_adcRaw(void)
{
    return AdcbResultRegs.ADCRESULT3;
}

// adc C

#pragma FUNC_ALWAYS_INLINE(bsp_get_CH1_Iu_adcRaw)
static inline int16_t bsp_get_CH1_Iu_adcRaw(void)
{
    return AdccResultRegs.ADCRESULT0;
}

#pragma FUNC_ALWAYS_INLINE(bsp_get_CH1_Vb_adcRaw)
static inline int16_t bsp_get_CH1_Vb_adcRaw(void)
{
    return AdccResultRegs.ADCRESULT1;
}

#pragma FUNC_ALWAYS_INLINE(bsp_get_CH2_Iu_adcRaw)
static inline int16_t bsp_get_CH2_Iu_adcRaw(void)
{
    return AdccResultRegs.ADCRESULT2;
}

#pragma FUNC_ALWAYS_INLINE(bsp_get_CH2_Vb_adcRaw)
static inline int16_t bsp_get_CH2_Vb_adcRaw(void)
{
    return AdccResultRegs.ADCRESULT3;
}

// adc D

#pragma FUNC_ALWAYS_INLINE(bsp_get_diff1_adcRaw)
static inline uint16_t bsp_get_diff1_adcRaw(void)
{
    return AdcdResultRegs.ADCRESULT0;
}

#pragma FUNC_ALWAYS_INLINE(bsp_get_ntc1_adcRaw)
static inline uint16_t bsp_get_ntc1_adcRaw(void)
{
    return AdcdResultRegs.ADCRESULT1;
}

#pragma FUNC_ALWAYS_INLINE(bsp_get_ntcRef1_adcRaw)
static inline uint16_t bsp_get_ntcRef1_adcRaw(void)
{
    return AdcdResultRegs.ADCRESULT2;
}

// adc sw trig
#pragma FUNC_ALWAYS_INLINE(bsp_adc_swTrig_ntc)
static inline void bsp_adc_swTrig_ntc(void)
{
    AdcdRegs.ADCSOCFRC1.all = 0b110u;
}

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

#endif /* INCX_BSP_INLINE_H_ */
