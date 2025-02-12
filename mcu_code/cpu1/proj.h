/*
 * proj.h
 *
 *  Created on: 2022年6月25日
 *      Author: tyj
 */

#ifndef INCX_PROJ_H_
#define INCX_PROJ_H_

void adcOffset_init(int32_t avgNum);

void ctrl_init();

void mainIsrProcess();

void mainIsrProcess2();

void mainLoopProcess();

extern struct SVPWM_struct CH1_svpwm;
extern struct ThataCal_struct CH1_thetaI;
extern struct ThataCal_struct CH1_thetaU;
extern struct Trans_struct CH1_Utar;

extern struct SVPWM_struct CH2_svpwm;
extern struct ThataCal_struct CH2_thetaI;
extern struct ThataCal_struct CH2_thetaU;
extern struct Trans_struct CH2_Utar;

extern struct ThataCal_struct thetaMTPA;

extern struct ThataCal_struct thetaMTPA2;

extern struct PIctrl_struct UdcPI;
extern struct PIctrl_struct PdcPI;

extern float Ld4Icomp;
extern float Lq4Icomp;
extern float Faif4Icomp;

extern float Ld4Icomp2;
extern float Lq4Icomp2;
extern float Faif4Icomp2;

extern struct ADRC_struct UdcAdrc;
extern uint16_t ADRC_mode;

extern struct PIctrl_struct omegaPI;

extern struct PIctrl_struct omegaPI2;

extern struct PIctrl_struct CH1_IdPI;
extern struct PIctrl_struct CH1_IqPI;

extern struct PIctrl_struct CH2_IdPI;
extern struct PIctrl_struct CH2_IqPI;

extern struct SpeedCal_struct omegaEcal;

extern struct SpeedCal_struct omegaEcal2;

extern struct protect_struct curProtect_CH1;
extern struct protect_struct UdcProtect_CH1;

extern struct protect_struct curProtect_CH2;
extern struct protect_struct UdcProtect_CH2;

extern struct protect_struct speedProtect;
extern struct protect_struct tempProtect;

extern struct LPF_Ord1_2_struct CH1_IdFilt;
extern struct LPF_Ord1_2_struct CH1_IqFilt;
extern struct LPF_Ord1_2_struct CH1_I0Filt;

extern struct LPF_Ord1_2_struct CH1_IdcFilt;
extern struct LPF_Ord1_2_struct CH1_UdcFilt;


extern struct LPF_Ord1_2_struct CH2_IdFilt;
extern struct LPF_Ord1_2_struct CH2_IqFilt;
extern struct LPF_Ord1_2_struct CH2_I0Filt;

extern struct LPF_Ord1_2_struct CH2_IdcFilt;
extern struct LPF_Ord1_2_struct CH2_UdcFilt;

extern struct LPF_Ord1_2_struct omegaFilt;

extern struct LPF_Ord1_2_struct omegaFilt2;

extern struct LPF_Ord1_2_struct PdcFilt;

extern struct LPF_Ord1_2_struct PdcFilt2;

extern struct Goertz_struct Goertz1;
extern struct Goertz_struct Goertz2;
extern struct OECA_struct OECA1;
extern struct LPF_Ord1_2_struct OECA_f1;

extern struct encoder_struct encoder1;
extern struct encoder_struct encoder2;
extern struct DRV8305_struct drv8305_1;
extern struct DRV8305_struct drv8305_2;

// 磁链标定与LUT补偿
extern struct LPF_Ord1_2_struct CH1_UTarFiltd;
extern struct LPF_Ord1_2_struct CH1_UTarFiltq;
extern struct Trans_struct CH1_UFilt;
extern struct Trans_struct CH1_psi;

extern float CH1_Ld4PI;
extern float CH1_Lq4PI;
extern float CH1_thetaE_inter;

// 位置谐波
extern struct LMSanf_struct LMSanfThetaM;
extern struct LPF_Ord1_2_struct CH1_IdFilt_2;
extern struct LPF_Ord1_2_struct CH1_IqFilt_2;
extern struct Trans_struct CH1_Ifilt2;

// 位置误差在线补偿
extern struct PIctrl_struct wPI;
extern struct PIctrl_struct RsPI;
extern struct MRASwr_struct MRASwr;
extern struct Trans_struct CH1_Psi;

extern struct KFw_struct KFw;

// bit0 注入谐波 bit1 补偿谐波 bit2 谐波辨识 bit3 在线补偿 bit4 位置切换
extern int16_t CH1_angle_mode2;

extern int16_t rdc_inj_raw;
extern int16_t rdc_comp_raw;

extern float rdc_inj_all;
extern float rdc_comp_all;
extern float thetaEst;
extern float thetaEst2;

extern float rdc_Acos_1;
extern float rdc_Asin_1;
extern float rdc_Acos_2;
extern float rdc_Asin_2;
extern float rdc_Acos_4;
extern float rdc_Asin_4;

// 采样值
extern int16_t CH1_Udc_raw;
extern int16_t CH1_Idc_raw;
extern int16_t CH1_Iu_raw;
extern int16_t CH1_Iv_raw;
extern int16_t CH1_Iw_raw;

extern int16_t CH1_samp_lThd;

extern int16_t CH1_Vu_raw;
extern int16_t CH1_Vv_raw;
extern int16_t CH1_Vw_raw;

extern int16_t CH2_Udc_raw;
extern int16_t CH2_Idc_raw;
extern int16_t CH2_Iu_raw;
extern int16_t CH2_Iv_raw;
extern int16_t CH2_Iw_raw;

extern int16_t CH2_samp_lThd;

extern int16_t CH2_Vu_raw;
extern int16_t CH2_Vv_raw;
extern int16_t CH2_Vw_raw;

extern uint16_t thetaEnco_raw;
extern uint16_t thetaEnco_raw_offset;

extern uint16_t thetaEnco_raw2;
extern uint16_t thetaEnco_raw_offset2;

extern int32_t NTC_raw;
extern int32_t NTC_ref_raw;

extern float NTC_Rref_R25;
extern float NTC_B;
extern float NTC_T0;
extern float NTC_temp;

extern float spdEnco;

extern float spdEnco2;

// SI 采样值、原始值
extern float CH1_Udc_SI;
extern float CH1_Idc_SI;

extern struct Trans_struct CH1_Ifbk;
extern struct Trans_struct CH1_Vfbk;
extern struct Trans_struct CH1_Ifilt;

extern float CH2_Udc_SI;
extern float CH2_Idc_SI;

extern struct Trans_struct CH2_Ifbk;
extern struct Trans_struct CH2_Vfbk;
extern struct Trans_struct CH2_Ifilt;

// 处理(滤波)后值
extern float CH1_Udc;
extern float CH1_Idc;

extern float CH2_Udc;
extern float CH2_Idc;

extern float Pdc;

extern float Pdc2;

extern float CH1_Udc_LThd;

extern float CH2_Udc_LThd;

// 转速环
extern float omegaMfbk;

extern float omegaMfbk2;

// 波形产生
extern int16_t targetWaveMode;
extern float targetWaveFreq;
extern float targetWaveAmp;
extern float targetWaveOffset;

extern int16_t targetWaveMode2;
extern float targetWaveFreq2;
extern float targetWaveAmp2;
extern float targetWaveOffset2;

// 斜坡给定
extern float targetRampVal;
extern float targetRampGrad;

extern float targetRampVal2;
extern float targetRampGrad2;

// 控制参数
extern float CH1_Iu_raw_offset;
extern float CH1_Iv_raw_offset;
extern float CH1_Iw_raw_offset;
extern float CH1_Idc_raw_offset;

extern float CH2_Iu_raw_offset;
extern float CH2_Iv_raw_offset;
extern float CH2_Iw_raw_offset;
extern float CH2_Idc_raw_offset;

extern int16_t CH1_angle_mode;
extern int16_t CH1_angle_mode2;

extern int16_t CH2_angle_mode;

extern int16_t CH1_cur_mode;
extern int16_t CH1_cur_mode2;

extern int16_t CH2_cur_mode;
extern int16_t CH2_cur_mode2;

extern uint16_t filtHWFaultCnt;

extern uint16_t filtHWFaultCnt2;

extern int16_t CH1_ext_fcn;
extern float db_Ithd_1;
extern float db_cmp_tick;
extern float db_cmp_vds;

extern int16_t CH2_ext_fcn;
extern float db2_Ithd_1;
extern float db2_cmp_tick;
extern float db2_cmp_vds;

extern int16_t CH1_cur_sta;
extern int16_t CH2_cur_sta;

extern int16_t speed_mode;
extern int16_t torque_mode;
extern int16_t channel_mode;

extern int16_t speed_mode2;
extern int16_t torque_mode2;
extern int16_t channel_mode2;

extern float targetTe;
extern float targetIs;
extern float targetThetaMTPA;

extern float targetTe2;
extern float targetIs2;
extern float targetThetaMTPA2;

extern float targetN;
extern float targetOmegaM;

extern float targetN2;
extern float targetOmegaM2;

extern float targetUdc;
extern float targetPdc;

extern float targetId_CH1;
extern float targetIq_CH1;
extern float targetThetaE_CH1;

extern float targetId_CH2;
extern float targetIq_CH2;
extern float targetThetaE_CH2;

extern float CH1_Ud_comp;
extern float CH1_Uq_comp;

extern float CH2_Ud_comp;
extern float CH2_Uq_comp;

extern float thetaEInc;

extern float thetaEInc2;

// 保护配置
extern int16_t IProtectFlg_CH1;
extern int16_t UdcProtectFlg_CH1;

extern int16_t IProtectFlg_CH2;
extern int16_t UdcProtectFlg_CH2;

extern int16_t SPDProtectFlg;
extern int16_t tempProtectFlg;

// 观测主中断执行时间
extern uint16_t isr_start_pwm_cnt;
extern uint16_t isr_end_pwm_cnt;
extern uint16_t isr_start_pwm_cnt2;
extern uint16_t isr_end_pwm_cnt2;

#endif /* INCX_PROJ_H_ */
