/**
 * @file scd_reg.cpp
 * @brief SCD注册文件
 * @author Tangent (498339337@qq.com)
 * @date 2022-04-08
 *
 * @copyright Copyright (c) 2022 @Tangent
 */

#include "scd_inc.h"

 /**
  * @brief 注册宏起始，尾缀标识具体实例名称
  */
#define SCD_REG_BEGIN(TYJ_SUFFIX)                                                                      \
    volatile unsigned char recBuff##TYJ_SUFFIX[2][SCD_REVBUFF_SIZE];                                   \
    volatile unsigned char printBuff##TYJ_SUFFIX[SCD_PRINTBUFF_SIZE];                                  \
    volatile unsigned char endBuff1##TYJ_SUFFIX[4] = {0x00, 0x00, 0x81, 0x7f};                         \
    volatile unsigned char endBuff2##TYJ_SUFFIX[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x82, 0x7f}; \
    volatile unsigned char endBuff3##TYJ_SUFFIX[4] = {0x00, 0x00, 0x83, 0x7f};                         \
    static const int _begin_index##TYJ_SUFFIX = __LINE__;                                              \
    const struct TYJ_UNIT_STRUCT tyj_unit_struct##TYJ_SUFFIX[] = {

  /**
   * @brief 注册宏，中间不可加入空行或注释
   * @param TYJ_NAME 需加入的参数名称
   * @param TYJ_TYPE 需加入的参数类型
   */
  #define SCD_REG_ADD(TYJ_NAME, TYJ_TYPE)     \
      {                                       \
          (void *)&(TYJ_NAME), TYJ_##TYJ_TYPE \
                                              \
      },

   /**
    * @brief 注册宏结尾，尾缀标识具体实例名称
    */
   #define SCD_REG_END(TYJ_SUFFIX)                                                            \
    }                                                                                      \
    ;                                                                                      \
    static const int _end_index##TYJ_SUFFIX = __LINE__;                                    \
    struct SCD_CTRL_STRUCT scd##TYJ_SUFFIX;                                                \
    void scd_init##TYJ_SUFFIX()                                                            \
    {                                                                                      \
        scd##TYJ_SUFFIX._unit_struct = tyj_unit_struct##TYJ_SUFFIX;                        \
        scd##TYJ_SUFFIX.structToSendTab[0] = 0;                                            \
        scd##TYJ_SUFFIX.structToSendTab[1] = 9999;                                         \
        scd##TYJ_SUFFIX._recBuff = recBuff##TYJ_SUFFIX;                                    \
        scd##TYJ_SUFFIX._printBuff = printBuff##TYJ_SUFFIX;                                \
        scd##TYJ_SUFFIX._endbyte1 = endBuff1##TYJ_SUFFIX;                                  \
        scd##TYJ_SUFFIX._endbyte2 = endBuff2##TYJ_SUFFIX;                                  \
        scd##TYJ_SUFFIX._endbyte3 = endBuff3##TYJ_SUFFIX;                                  \
        scd##TYJ_SUFFIX.structNum = _end_index##TYJ_SUFFIX - _begin_index##TYJ_SUFFIX - 1; \
        scd##TYJ_SUFFIX.bufNum = 0;                                                        \
        scd##TYJ_SUFFIX.bytesRec = 0;                                                      \
        scd##TYJ_SUFFIX.structToSend = 0;                                                  \
        scd##TYJ_SUFFIX.byteToSend = 0;                                                    \
        scd##TYJ_SUFFIX.tempBuff = 0;                                                      \
        scd##TYJ_SUFFIX.sco_protocol_num = 0;                                              \
        scd##TYJ_SUFFIX.sco_protocol_num_next = 0;                                         \
        scd##TYJ_SUFFIX.structToSend2 = 0;                                                 \
        scd##TYJ_SUFFIX.byteToSend2 = 0;                                                   \
        scd##TYJ_SUFFIX.byteToSend3 = 0;                                                   \
        scd##TYJ_SUFFIX.structToSend3 = 0;                                                 \
        scd##TYJ_SUFFIX.dumpTarget = 0;                                                    \
        scd##TYJ_SUFFIX.dumpNumCnt = 0;                                                    \
        scd##TYJ_SUFFIX.dumpPkgCnt = 0;                                                    \
        scd##TYJ_SUFFIX.ifContPkg = 0;                                                     \
        scd##TYJ_SUFFIX.isPrintBusyFlg = 0;                                                \
        scd##TYJ_SUFFIX.byteToPrint = 0;                                                   \
        scd##TYJ_SUFFIX.byteToSend4 = 0;                                                   \
    }

#include "F28x_Project.h"
#include "math.h"

#include "bsp_cfg.h"
#include "transform.h"
#include "thetaCal.h"
#include "SVPWM.h"
#include "pid.h"
#include "waveGen.h"
#include "samp.h"
#include "filter.h"
#include "speed_cal.h"
#include "comp.h"
#include "oeca.h"
#include "adrc.h"
#include "lms_anf.h"
#include "MRASwr.h"
#include "kalmanFilt_w.h"
#include "psi_lut.h"

#include "proj.h"

#include "cpu2isr.h"

    /**
     * @brief 注册宏1#scd的观测内容
     */
SCD_REG_BEGIN(_1)
SCD_REG_ADD(CH1_cur_mode, int16_t)
SCD_REG_ADD(CH2_cur_mode, int16_t)
SCD_REG_ADD(speed_mode, int16_t)
SCD_REG_ADD(speed_mode2, int16_t)
SCD_REG_ADD(torque_mode, int16_t)
SCD_REG_ADD(torque_mode2, int16_t)
SCD_REG_ADD(channel_mode, int16_t)
SCD_REG_ADD(channel_mode2, int16_t)
SCD_REG_ADD(CH1_angle_mode, int16_t)
SCD_REG_ADD(CH2_angle_mode, int16_t)
SCD_REG_ADD(CH1_ext_fcn, int16_t)
SCD_REG_ADD(CH2_ext_fcn, int16_t)
SCD_REG_ADD(thetaEnco_raw, uint16_t)
SCD_REG_ADD(thetaEnco_raw2, uint16_t)
SCD_REG_ADD(IProtectFlg_CH1, hex16)
SCD_REG_ADD(IProtectFlg_CH2, hex16)
SCD_REG_ADD(UdcProtectFlg_CH1, hex16)
SCD_REG_ADD(UdcProtectFlg_CH2, hex16)
SCD_REG_ADD(SPDProtectFlg, hex16)
SCD_REG_ADD(tempProtectFlg, hex16)
SCD_REG_ADD(isr_start_pwm_cnt, uint16_t)
SCD_REG_ADD(isr_end_pwm_cnt, uint16_t)
SCD_REG_ADD(isr_start_pwm_cnt2, uint16_t)
SCD_REG_ADD(isr_end_pwm_cnt2, uint16_t)
SCD_REG_ADD(cpu2mainIsrTick, uint16_t)
SCD_REG_ADD(targetRampVal, float)
SCD_REG_ADD(targetRampGrad, float)
SCD_REG_ADD(targetRampVal2, float)
SCD_REG_ADD(targetRampGrad2, float)
SCD_REG_ADD(targetThetaMTPA, float)
SCD_REG_ADD(targetThetaMTPA2, float)
SCD_REG_ADD(CH1_Udc, float)
SCD_REG_ADD(CH1_Idc, float)
SCD_REG_ADD(targetId_CH1, float)
SCD_REG_ADD(targetIq_CH1, float)
SCD_REG_ADD(CH1_Ifilt.d, float)
SCD_REG_ADD(CH1_Ifilt.q, float)
SCD_REG_ADD(CH1_Ifilt.abdq0, float)
SCD_REG_ADD(CH1_Utar.d, float)
SCD_REG_ADD(CH1_Utar.q, float)
SCD_REG_ADD(CH1_svpwm.Udc, float)
SCD_REG_ADD(targetThetaE_CH1, float)
SCD_REG_ADD(CH2_Udc, float)
SCD_REG_ADD(CH2_Idc, float)
SCD_REG_ADD(targetId_CH2, float)
SCD_REG_ADD(targetIq_CH2, float)
SCD_REG_ADD(CH2_Ifilt.d, float)
SCD_REG_ADD(CH2_Ifilt.q, float)
SCD_REG_ADD(CH2_Ifilt.abdq0, float)
SCD_REG_ADD(CH2_Utar.d, float)
SCD_REG_ADD(CH2_Utar.q, float)
SCD_REG_ADD(CH2_svpwm.Udc, float)
SCD_REG_ADD(targetThetaE_CH2, float)
SCD_REG_ADD(NTC_temp, float)
SCD_REG_ADD(omegaMfbk, float)
SCD_REG_ADD(omegaMfbk2, float)
SCD_REG_ADD(targetTe, float)
SCD_REG_ADD(targetTe2, float)
SCD_REG_ADD(targetIs, float)
SCD_REG_ADD(targetIs2, float)
SCD_REG_ADD(targetN, float)
SCD_REG_ADD(targetN2, float)
SCD_REG_ADD(targetOmegaM, float)
SCD_REG_ADD(targetOmegaM2, float)
SCD_REG_ADD(targetUdc, float)
SCD_REG_ADD(thetaEInc, float)
SCD_REG_ADD(thetaEInc2, float)
SCD_REG_ADD(CH1_angle_mode2, int16_t)
SCD_REG_ADD(rdc_inj_all, float)
SCD_REG_ADD(thetaEst, float)
SCD_REG_ADD(thetaEstFilt, float)
SCD_REG_ADD(LMSanfThetaM.W[0], float)
SCD_REG_ADD(LMSanfThetaM.W[1], float)
SCD_REG_ADD(LMSanfThetaM.W[2], float)
SCD_REG_ADD(LMSanfThetaM.W[3], float)
SCD_REG_ADD(LMSanfThetaM.W[4], float)
SCD_REG_ADD(LMSanfThetaM.W[5], float)
SCD_REG_ADD(CH1_Ld4PI, float)
SCD_REG_ADD(CH1_Lq4PI, float)
SCD_REG_ADD(CH1_thetaE_inter, float)
SCD_REG_ADD(CH1_thetaE_sensor, float)
SCD_REG_ADD(MRASwr.We, float)
SCD_REG_ADD(MRASwr.Rs, float)
SCD_REG_ADD(MRASwr.thetaOut, float)
SCD_REG_ADD(dbgErr_MARS, float)
SCD_REG_ADD(KFw.thetaOutPu, float)
SCD_REG_ADD(KFw.x_k[1], float)
SCD_REG_ADD(KFw.x_k[2], float)
SCD_REG_ADD(KFw.R_tr[0], float)
SCD_REG_ADD(KFw.R_tr[1], float)
SCD_REG_ADD(dbgErr_KFw, float)
SCD_REG_ADD(thetaEnco_raw_offset, uint16_t)
SCD_REG_ADD(thetaEnco_raw_offset2, uint16_t)
SCD_REG_ADD(adcOffset_init, function)
SCD_REG_END(_1)
