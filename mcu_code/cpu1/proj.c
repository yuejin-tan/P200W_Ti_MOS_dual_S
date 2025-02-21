/*
 * proj.c
 *
 *  Created on: 2022年6月25日
 *      Author: tyj
 */
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
#include "interp_tab.h"
#include "deadBandComp.h"
#include "oeca.h"
#include "adrc.h"
#include "lms_anf.h"
#include "MRASwr.h"
#include "kalmanFilt_w.h"
#include "psi_lut.h"

#include "proj.h"
#include "bsp_inline.h"
#include "protect.h"
#include "encoder.h"
#include "proj2.h"

#include "drv8305.h"

struct SVPWM_struct CH1_svpwm;
struct ThataCal_struct CH1_thetaI;
struct ThataCal_struct CH1_thetaU;
struct Trans_struct CH1_Utar;

struct SVPWM_struct CH2_svpwm;
struct ThataCal_struct CH2_thetaI;
struct ThataCal_struct CH2_thetaU;
struct Trans_struct CH2_Utar;

struct ThataCal_struct thetaMTPA;

struct ThataCal_struct thetaMTPA2;

struct PIctrl_struct UdcPI;
struct PIctrl_struct PdcPI;

float Ld4Icomp = MATLAB_PARA_Ld;
float Lq4Icomp = MATLAB_PARA_Lq;
float Faif4Icomp = MATLAB_PARA_faif;

float Ld4Icomp2 = DYNO_PARA_Ld;
float Lq4Icomp2 = DYNO_PARA_Lq;
float Faif4Icomp2 = DYNO_PARA_faif;

struct ADRC_struct UdcAdrc;
uint16_t ADRC_mode = 0;

struct PIctrl_struct omegaPI;

struct PIctrl_struct omegaPI2;

struct PIctrl_struct CH1_IdPI;
struct PIctrl_struct CH1_IqPI;

struct PIctrl_struct CH2_IdPI;
struct PIctrl_struct CH2_IqPI;

struct SpeedCal_struct omegaEcal;

struct SpeedCal_struct omegaEcal2;

struct protect_struct curProtect_CH1;
struct protect_struct UdcProtect_CH1;

struct protect_struct curProtect_CH2;
struct protect_struct UdcProtect_CH2;

struct protect_struct speedProtect;
struct protect_struct tempProtect;

struct LPF_Ord1_2_struct CH1_IdFilt;
struct LPF_Ord1_2_struct CH1_IqFilt;
struct LPF_Ord1_2_struct CH1_I0Filt;

struct LPF_Ord1_2_struct CH1_IdcFilt;
struct LPF_Ord1_2_struct CH1_UdcFilt;

struct LPF_Ord1_2_struct CH2_IdFilt;
struct LPF_Ord1_2_struct CH2_IqFilt;
struct LPF_Ord1_2_struct CH2_I0Filt;

struct LPF_Ord1_2_struct CH2_IdcFilt;
struct LPF_Ord1_2_struct CH2_UdcFilt;

struct LPF_Ord1_2_struct omegaFilt;

struct LPF_Ord1_2_struct omegaFilt2;

struct LPF_Ord1_2_struct PdcFilt;

struct LPF_Ord1_2_struct PdcFilt2;

struct Goertz_struct Goertz1;
struct Goertz_struct Goertz2;
struct OECA_struct OECA1;
struct LPF_Ord1_2_struct OECA_f1;

struct encoder_struct encoder1;
struct encoder_struct encoder2;
struct DRV8305_struct drv8305_1;
struct DRV8305_struct drv8305_2;

// 磁链标定与LUT补偿
struct LPF_Ord1_2_struct CH1_UTarFiltd;
struct LPF_Ord1_2_struct CH1_UTarFiltq;
struct Trans_struct CH1_Psi;

float CH1_Ld4PI = MATLAB_PARA_Ld;
float CH1_Lq4PI = MATLAB_PARA_Lq;
float CH1_thetaE_inter = 0;

// 位置谐波
struct LMSanf_struct LMSanfThetaM;
struct LPF_Ord1_2_struct CH1_IdFilt_2;
struct LPF_Ord1_2_struct CH1_IqFilt_2;
struct Trans_struct CH1_Ifilt2;

// 位置误差在线补偿
struct PIctrl_struct wPI;
struct PIctrl_struct RsPI;
struct MRASwr_struct MRASwr;

struct KFw_struct KFw;

// bit0 注入谐波 bit1 补偿谐波 bit2 谐波辨识 bit3 在线补偿 bit4 位置切换
int16_t CH1_angle_mode2 = 0;

int16_t rdc_inj_raw = 0;
int16_t rdc_comp_raw = 0;

uint16_t thetaRaw_inter = 0;

float CH1_thetaE_sensor = 0;
float dbgErr_MARS = 0;
float dbgErr_KFw = 0;
float dbgErr_Sensor = 0;

float rdc_inj_all = 0;
float rdc_comp_all = 0;
float thetaEst = 0;
float thetaEstFilt = 0;

float rdc_Acos_1 = -4.3545e-04;
float rdc_Asin_1 = 0.0020;
float rdc_Acos_2 = -0.0016;
float rdc_Asin_2 = 1.0957e-04;
float rdc_Acos_4 = -1.7133e-04;
float rdc_Asin_4 = 3.3302e-05;

float rdc_wNoise_gain = 0.002;

static const float wNoiseTabNeg1_1_256[256] = { 0.022116653008496634, -0.640208132384184, 0.8797824093988833, 0.18312599826983722, -0.011974623923470817, -0.9826759384646464, -0.9217573610370497, -0.9395350319381304, 0.30374941287540946, 0.2012828314394297, -0.5238432580743342, -0.12235932476274791, 0.34150817460360705, -0.7401708372506519, -0.011013206772932271, -0.9723971475939952, 0.6423128841556882, -0.2896848589044234, -0.12134428014742116, 0.3294624718997723, 0.04041388359881326, 0.8305363986269463, 0.5713714442865161, -0.7593177709535075, 0.7015175464144858, -0.4680312020839834, 0.5153955340872947, -0.6950992540871663, -0.4303199114348075, -0.46141684439508546, 0.4337691648825679, -0.13949328467452182, -0.2691823591376339, -0.5372776807791322, 0.046941321093037836, 0.4981886110666669, 0.43039304966643677, -0.14668732792464767, -0.9271076611340592, 0.727161109293833, 0.16454890714362036, -0.10337299081333873, 0.05822650795820827, 0.11354516913366441, -0.20024672597793702, -0.7237427553381941, -0.4225836187714682, -0.6589401651940212, -0.1296958069987162, 0.6164502409329675, -0.3572426651477645, 0.2657006726982616, 0.38615544249207345, 0.13197814225768667, 0.9061282755517719, -0.5133262490303905, 0.9080778257382589, 0.7020486463669948, -0.07795353310937503, 0.9684917002715996, 0.4873575556073628, -0.460208441365892, -0.7622427699264058, 0.9350812852448593, -0.4341870958084433, 0.2030223411054617, -0.3320221792357563, 0.7684202273009566, -0.8891536324298013, -0.1327115365484406, 0.1864484436625884, 0.6439307661265532, -0.9604651167538798, 0.6500329684522164, -0.4638412535102885, -0.6535099328375427, -0.43882548330927706, -0.10146276395361498, 0.5508181762904192, 0.6594641468863103, 0.39611681147703437, 0.14798216953501386, -0.32800353472520816, 0.959384679615229, 0.04134650122336048, -0.6581556554505961, -0.16223835169600376, -0.6564891437040925, -0.43305155403919926, 0.4190639285194291, -0.3181902113470867, 0.20369334760994295, -0.5919758153842842, -0.8588933358663866, 0.8338410808952581, 0.3527283392288909, 0.8330820088906032, -0.9750362747810353, 0.6750746142546795, 0.08740197254125337, -0.005967147958162222, -0.885205857501056, 0.5885206935837575, 0.22993144696545098, -0.4170215895081544, 0.7748689971670093, 0.4197193723221426, -0.14256808647730335, -0.7905959412449273, 0.030358551856308535, 0.21284237461364253, 0.3927850935329844, 0.41383472908342633, 0.40407856142376053, 0.4789469421484158, 0.33507323424366864, 0.3530941043077582, 0.9123008949290301, -0.5961599519807641, 0.8288936493190593, 0.059896594029305694, 0.515757453546049, -0.6636502227338104, 0.09604548651328537, 0.8619720767355401, 0.8195817894372823, -0.139394331528935, 0.09677913944104954, 0.559277157570383, -0.7714269315097098, -0.3086170652513369, -0.8183985864339851, 0.4040055830667635, 0.7073725108002658, -0.5766685604925623, -0.2543776453889859, 0.43690929568990766, -0.8648255729644478, -0.977684549449104, 0.14380796702805565, -0.6494375033064745, -0.247695522153067, -0.2490518363247567, -0.5385365027491396, -0.9440368669481118, 0.09577806363005781, 0.4144488545471168, 0.9648984820804987, -0.02244050451939672, 0.9847969162443371, -0.13175977143733197, 0.5567765471109141, -0.4725086781992307, -0.5458086410579057, 0.8698971503469157, -0.4781946329852338, 0.798110332639314, -0.0308723072596635, -0.4286816357760488, -0.3453574310948646, 0.9653603339625556, 0.9264396366680958, -0.8259564399120689, -0.5891371969660806, -0.24595238707848144, -0.08895339805549063, 0.4547985964027266, -0.012774410845214357, 0.8144003282374412, -0.26303324753736046, -0.18274570223047193, -0.5673378693043352, 0.40237699499971913, 0.9324505408722294, 0.6439665279154076, -0.7214430716081113, -0.45652701268992635, 0.014444589966541743, 0.3527856830770739, 0.21078490255046045, 0.7455882125198463, 0.011624684893885462, 0.3978692181417498, -0.5734404663618342, 0.6981370994376099, -0.5005860793156149, 0.44567516275204966, -0.12011874360142993, -0.0892333021681877, -0.3274184331588137, 0.8232497178611751, 0.14920219829274983, -0.38492359703182255, -0.19655744385169793, -0.6725664866820706, 0.016616806041311216, 0.3244569210011863, 0.5608053584831754, -0.6969540068965538, -0.21569276365340873, -0.13947080604603745, -0.22990844712898828, -0.31071236901229926, 0.0023640043617723094, -0.7279593660871562, 0.6692561883108463, -0.9135400623802927, -0.505967425142601, 0.17586443289821263, 0.23713924178867662, -0.3723690318252222, -0.6885613639515229, 0.1304122923926947, 0.9309781711906888, -0.125297657414718, -0.009249127427992443, 0.8432656265896683, 0.06997860393949717, -0.9772318355625633, -0.7231474644875509, -0.9608962945763215, -0.26630865743899856, 0.47247408403903113, 0.9992908871244597, 0.6566563816330573, -0.15636232898783486, 0.7889519228594613, 0.9263203562459497, -0.8373302604992434, -0.02838945422270145, -0.29014556205299713, -0.16696120292496297, -0.7427834349805023, -0.6845517440000628, -0.4183976314114892, -0.3792638549911098, -0.2942532209868278, 0.9470167761296471, -0.9708317142982315, -0.05738658949548903, 0.09905867882415809, 0.7039119961168261, 0.7797803374434709, 0.42813200919492256, 0.6783823993326572, 0.16161390869017644, 0.29837284329361924, 0.04804529856637174, -0.46376224596614657, 0.25304131691675646, -0.02207880355786007, -0.6336746890452283, 0.36272289061981433, 0.39984391744599135, 0.9348324124039504, 0.5983875527303222 };

enum ANGLE_MODE
{
    AM_manual = 0,
    AM_self_inc,
    AM_sync,
    AM_sync_comp,
};

enum CURRENT_MODE
{
    CM_error = -1,
    CM_stop = 0,
    CM_svpwm,
    CM_I_loop,
    CM_LSSC,
};

enum SPEED_MODE
{
    SM_err = -1,
    SM_stop = 0,
    SM_Is_ramp,
    SM_Te_ramp,
    SM_OECA_exp,
    SM_speed_loop_N,
    SM_speed_loop_ramp_N,
    SM_Pdc_loop,
    SM_Pdc_loop_ramp,
    SM_Udc_loop,
    SM_Udc_loop_ramp,
    SM_Udc_loop2,
    SM_Udc_loop_ramp2,
};

enum TORQUE_MODE
{
    TM_id0 = 0,
    TM_MTPA,
};

enum OUT_MODE
{
    OM_stop = 0,
    OM_CH1, // 1
    OM_free, // 2
};

enum WAVE_MODE
{
    WM_trig = 0,
    WM_sine,
};

// 采样值
int16_t CH1_Udc_raw = 0;
int16_t CH1_Idc_raw = (MATLAB_PARA_I_bus_sense_offset * MATLAB_PARA_adc_gain);
int16_t CH1_Iu_raw = (MATLAB_PARA_Isense_offset * MATLAB_PARA_adc_gain);
int16_t CH1_Iv_raw = (MATLAB_PARA_Isense_offset * MATLAB_PARA_adc_gain);
int16_t CH1_Iw_raw = (MATLAB_PARA_Isense_offset * MATLAB_PARA_adc_gain);

int16_t CH1_samp_lThd = 200;

int16_t CH1_Vu_raw = 0;
int16_t CH1_Vv_raw = 0;
int16_t CH1_Vw_raw = 0;

int16_t CH2_Udc_raw = 0;
int16_t CH2_Idc_raw = (MATLAB_PARA_I_bus_sense_offset * MATLAB_PARA_adc_gain);
int16_t CH2_Iu_raw = (MATLAB_PARA_Isense_offset * MATLAB_PARA_adc_gain);
int16_t CH2_Iv_raw = (MATLAB_PARA_Isense_offset * MATLAB_PARA_adc_gain);
int16_t CH2_Iw_raw = (MATLAB_PARA_Isense_offset * MATLAB_PARA_adc_gain);

int16_t CH2_samp_lThd = 200;

int16_t CH2_Vu_raw = 0;
int16_t CH2_Vv_raw = 0;
int16_t CH2_Vw_raw = 0;

uint16_t thetaEnco_raw = 0;
uint16_t thetaEnco_raw_offset = 5200;

uint16_t thetaEnco_raw2 = 0;
uint16_t thetaEnco_raw_offset2 = 2100;

int32_t NTC_raw = 0;
int32_t NTC_ref_raw = 0;

float NTC_Rref_R25 = 3000.0 / 5000.0;
float NTC_B = 3375;
float NTC_T0 = 298.15;
float NTC_temp = 25;

float spdEnco = 0;

float spdEnco2 = 0;

// SI 采样值、原始值
float CH1_Udc_SI = 0.0f;
float CH1_Idc_SI = 0.0f;

struct Trans_struct CH1_Ifbk;
struct Trans_struct CH1_Vfbk;
struct Trans_struct CH1_Ifilt;

float CH2_Udc_SI = 0.0f;
float CH2_Idc_SI = 0.0f;

struct Trans_struct CH2_Ifbk;
struct Trans_struct CH2_Vfbk;
struct Trans_struct CH2_Ifilt;

// 处理(滤波)后值
float CH1_Udc = 0.0f;
float CH1_Idc = 0.0f;

float CH2_Udc = 0.0f;
float CH2_Idc = 0.0f;

float Pdc = 0.0f;

float Pdc2 = 0.0f;

float CH1_Udc_LThd = 6.0f;

float CH2_Udc_LThd = 6.0f;

// 转速环
float omegaMfbk = 0.0f;

float omegaMfbk2 = 0.0f;

// 波形产生
int16_t targetWaveMode = 0;
float targetWaveFreq = 1.0f;
float targetWaveAmp = 1.0f;
float targetWaveOffset = 0;

int16_t targetWaveMode2 = 0;
float targetWaveFreq2 = 1.0f;
float targetWaveAmp2 = 1.0f;
float targetWaveOffset2 = 0;

// 斜坡给定
float targetRampVal = 0.0f;
float targetRampGrad = 1.0f;

float targetRampVal2 = 0.0f;
float targetRampGrad2 = 1.0f;

// 控制参数
float CH1_Idc_raw_offset = (MATLAB_PARA_I_bus_sense_offset * MATLAB_PARA_adc_gain);
float CH1_Iu_raw_offset = (MATLAB_PARA_Isense_offset * MATLAB_PARA_adc_gain);
float CH1_Iv_raw_offset = (MATLAB_PARA_Isense_offset * MATLAB_PARA_adc_gain);
float CH1_Iw_raw_offset = (MATLAB_PARA_Isense_offset * MATLAB_PARA_adc_gain);

float CH2_Idc_raw_offset = (MATLAB_PARA_I_bus_sense_offset * MATLAB_PARA_adc_gain);
float CH2_Iu_raw_offset = (MATLAB_PARA_Isense_offset * MATLAB_PARA_adc_gain);
float CH2_Iv_raw_offset = (MATLAB_PARA_Isense_offset * MATLAB_PARA_adc_gain);
float CH2_Iw_raw_offset = (MATLAB_PARA_Isense_offset * MATLAB_PARA_adc_gain);

int16_t CH1_angle_mode = 0;

int16_t CH2_angle_mode = 0;

int16_t CH1_cur_mode = 0;
int16_t CH1_cur_mode2 = 0;

int16_t CH2_cur_mode = 0;
int16_t CH2_cur_mode2 = 0;

uint16_t filtHWFaultCnt = 0;

uint16_t filtHWFaultCnt2 = 0;

// bit 0 1: udc var
// bit 1 2: Udq comp with fbk val
// bit 2 4: deadBand comp
// bit 3 8: Udq comp with LUT (ch1 only)
// bit 4 16: PI Para auto (ch1 only)
int16_t CH1_ext_fcn = 0;
float db_Ithd_1 = 1.0f / MATLAB_PARA_db_Ithd;
// 注意，由于是强行兼容的，故死区补偿的参数需*2
float db_cmp_tick = MATLAB_PARA_tick_db_comp * 2;
float db_cmp_vds = MATLAB_PARA_Vdf + MATLAB_PARA_Vsat;

int16_t CH2_ext_fcn = 0;
float db2_Ithd_1 = 2.0; // 实验值
// 注意，由于是强行兼容的，故死区补偿的参数需*2
float db2_cmp_tick = MATLAB_PARA_tick_db_comp * 2;
float db2_cmp_vds = MATLAB_PARA_Vdf + MATLAB_PARA_Vsat;

int16_t CH1_cur_sta = 0;
int16_t CH2_cur_sta = 0;

int16_t speed_mode = 0;
int16_t torque_mode = 0;
int16_t channel_mode = 0;

int16_t speed_mode2 = 0;
int16_t torque_mode2 = 0;
int16_t channel_mode2 = 0;

float targetTe = 0.0f;
float targetIs = 0.0f;
float targetThetaMTPA = 0.25f;

float targetTe2 = 0.0f;
float targetIs2 = 0.0f;
float targetThetaMTPA2 = 0.25f;

float targetN = 0.0f;
float targetOmegaM = 0.0f;

float targetN2 = 0.0f;
float targetOmegaM2 = 0.0f;

float targetUdc = 0.0f;
float targetPdc = 0.0f;

float targetId_CH1 = 0.0f;
float targetIq_CH1 = 0.0f;
float targetThetaE_CH1 = 0.0f;

float targetId_CH2 = 0.0f;
float targetIq_CH2 = 0.0f;
float targetThetaE_CH2 = 0.0f;

float CH1_Ud_comp = 0.0f;
float CH1_Uq_comp = 0.0f;

float CH2_Ud_comp = 0.0f;
float CH2_Uq_comp = 0.0f;

float thetaEInc = 1e-5;

float thetaEInc2 = 1e-5;

// 保护配置
int16_t IProtectFlg_CH1 = 0;
int16_t UdcProtectFlg_CH1 = 0;

int16_t IProtectFlg_CH2 = 0;
int16_t UdcProtectFlg_CH2 = 0;

// 这些保护共用
int16_t SPDProtectFlg = 0;
int16_t tempProtectFlg = 0;

#define M_1Ord_LPF_Kahan(coeff, oldVal, newVal) \
do{\
 static volatile float errSum = 0 ;\
 volatile float deltaNew = ( newVal - oldVal ) * coeff - errSum;\
 volatile float ans = oldVal + deltaNew;\
 volatile float mustCalcFirst = ans - oldVal;\
 errSum = mustCalcFirst - deltaNew;\
 oldVal = ans;\
}while(0)

void adcOffset_init(int32_t avgNum)
{

    int32_t sumsTab[8] = { 0,0,0,0,0,0,0,0 };
    float avgNum_1 = 1.0f / avgNum;

    DELAY_US(100);

    const int delay_tick_tmp = (float)(1e6 / 25.0) / vCTRL_FREQ;

    // 提示编译器不要展开循环，因为这不是时间关键代码
#pragma UNROLL ( 1 )
    for (int32_t i = 0;i < avgNum;i++)
    {
        F28x_usDelay(delay_tick_tmp);
        sumsTab[1] += CH1_Iu_raw;
        sumsTab[2] += CH1_Iv_raw;
        sumsTab[3] += CH1_Iw_raw;

        sumsTab[5] += CH2_Iu_raw;
        sumsTab[6] += CH2_Iv_raw;
        sumsTab[7] += CH2_Iw_raw;

    }

    CH1_Iu_raw_offset = sumsTab[1] * avgNum_1;
    CH1_Iv_raw_offset = sumsTab[2] * avgNum_1;
    CH1_Iw_raw_offset = sumsTab[3] * avgNum_1;

    CH2_Iu_raw_offset = sumsTab[5] * avgNum_1;
    CH2_Iv_raw_offset = sumsTab[6] * avgNum_1;
    CH2_Iw_raw_offset = sumsTab[7] * avgNum_1;

}

static void cfg_clk_util(uint16_t pwm_freq)
{
    algo_clk_cfg(pwm_freq);

    LPF_Ord1_2_cfg(&CH1_IdFilt, LPF_ORD_2_t, vCTRL_TS, vCTRL_FREQ * MATLAB_PARA_Idq_filter_factor, MATLAB_PARA_Idq_filter_yita);
    LPF_Ord1_2_cfg(&CH1_IqFilt, LPF_ORD_2_t, vCTRL_TS, vCTRL_FREQ * MATLAB_PARA_Idq_filter_factor, MATLAB_PARA_Idq_filter_yita);
    LPF_Ord1_2_cfg(&CH1_I0Filt, LPF_ORD_2_t, vCTRL_TS, vCTRL_FREQ * MATLAB_PARA_Idq_filter_factor, MATLAB_PARA_Idq_filter_yita);
    LPF_Ord1_2_cfg(&CH1_UdcFilt, LPF_ORD_2_t, vCTRL_TS, MATLAB_PARA_outer_filter_bw, MATLAB_PARA_outer_filter_yita);
    LPF_Ord1_2_cfg(&CH1_IdcFilt, LPF_ORD_2_t, vCTRL_TS, MATLAB_PARA_outer_filter_bw, MATLAB_PARA_outer_filter_yita);

    LPF_Ord1_2_cfg(&PdcFilt, LPF_ORD_2_t, vCTRL_TS, MATLAB_PARA_outer_filter_bw, MATLAB_PARA_outer_filter_yita);
    LPF_Ord1_2_cfg(&omegaFilt, LPF_KAHAN_2_t, vCTRL_TS, MATLAB_PARA_omega_filter_bw, NULL);

    PIctrl_Iloop_cfg(&CH1_IdPI, MATLAB_PARA_Iloop_bw_factor, MATLAB_PARA_Ld, MATLAB_PARA_Rall, MATLAB_PARA_Upi_max, -MATLAB_PARA_Upi_max);
    PIctrl_Iloop_cfg(&CH1_IqPI, MATLAB_PARA_Iloop_bw_factor, MATLAB_PARA_Lq, MATLAB_PARA_Rall, MATLAB_PARA_Upi_max, -MATLAB_PARA_Upi_max);

    // 对拖机器，懒得修改matlab脚本了，参数放在 bsp_cfg.h 里
    // ch2 由于控制频率翻倍但单采单更，因此该部分计算仍可套用原先的
    LPF_Ord1_2_cfg(&CH2_IdFilt, LPF_ORD_2_t, vCTRL_TS, vCTRL_FREQ * MATLAB_PARA_Idq_filter_factor, MATLAB_PARA_Idq_filter_yita);
    LPF_Ord1_2_cfg(&CH2_IqFilt, LPF_ORD_2_t, vCTRL_TS, vCTRL_FREQ * MATLAB_PARA_Idq_filter_factor, MATLAB_PARA_Idq_filter_yita);
    LPF_Ord1_2_cfg(&CH2_I0Filt, LPF_ORD_2_t, vCTRL_TS, vCTRL_FREQ * MATLAB_PARA_Idq_filter_factor, MATLAB_PARA_Idq_filter_yita);
    LPF_Ord1_2_cfg(&CH2_UdcFilt, LPF_ORD_2_t, vCTRL_TS, MATLAB_PARA_outer_filter_bw, MATLAB_PARA_outer_filter_yita);
    LPF_Ord1_2_cfg(&CH2_IdcFilt, LPF_ORD_2_t, vCTRL_TS, MATLAB_PARA_outer_filter_bw, MATLAB_PARA_outer_filter_yita);

    LPF_Ord1_2_cfg(&PdcFilt2, LPF_ORD_2_t, vCTRL_TS, MATLAB_PARA_outer_filter_bw, MATLAB_PARA_outer_filter_yita);
    LPF_Ord1_2_cfg(&omegaFilt2, LPF_KAHAN_2_t, vCTRL_TS, MATLAB_PARA_omega_filter_bw, NULL);

    PIctrl_Iloop_cfg(&CH2_IdPI, MATLAB_PARA_Iloop_bw_factor, DYNO_PARA_Ld, DYNO_PARA_Rall, MATLAB_PARA_Upi_max, -MATLAB_PARA_Upi_max);
    PIctrl_Iloop_cfg(&CH2_IqPI, MATLAB_PARA_Iloop_bw_factor, DYNO_PARA_Lq, DYNO_PARA_Rall, MATLAB_PARA_Upi_max, -MATLAB_PARA_Upi_max);

    // 磁链标定
    LPF_Ord1_2_cfg(&CH1_UTarFiltd, LPF_KAHAN_1_t, vCTRL_TS, 20.0, 0);
    LPF_Ord1_2_cfg(&CH1_UTarFiltq, LPF_KAHAN_1_t, vCTRL_TS, 20.0, 0);

    // 位置误差提取
    LMSanfInit(&LMSanfThetaM);
    LPF_Ord1_2_cfg(&CH1_IdFilt_2, LPF_KAHAN_2_t, vCTRL_TS, 5.0, 0);
    LPF_Ord1_2_cfg(&CH1_IqFilt_2, LPF_KAHAN_2_t, vCTRL_TS, 5.0, 0);

    // 位置误差在线补偿
    PIctrl_init(&wPI, 100, 10000, 0, 0, 0);
    PIctrl_init(&RsPI, 0, 0.0001, 0, MATLAB_PARA_Rall * 3.0, MATLAB_PARA_Rall * 0.3);
    MRAS_wr_init(&MRASwr, &wPI, &RsPI, 6.28 * 10.0, MATLAB_PARA_Rall, 4.0);

    KFw_init(&KFw, 1, (2.0 * M_PI * 2.0 * M_PI / 12.0), 2e-4, 2e-3, 1e-4);
}

void ctrl_init()
{
    cfg_clk_util(PWM_FREQ);

    SVPWM_init(&CH1_svpwm, UDC_INIT_VAL);

    SVPWM_init(&CH2_svpwm, UDC_INIT_VAL);

    thetaCal_init(&CH1_thetaI);
    thetaCal_init(&CH1_thetaU);

    thetaCal_init(&CH2_thetaI);
    thetaCal_init(&CH2_thetaU);

    thetaCal_init(&thetaMTPA);

    thetaCal_init(&thetaMTPA2);

    PIctrl_init(&omegaPI, MATLAB_PARA_kp_wloop, MATLAB_PARA_ki_wloop, MATLAB_PARA_ki_wloop, MATLAB_PARA_Wpi_max, MATLAB_PARA_Wpi_min);

    // 对拖电机参数修正
    PIctrl_init(&omegaPI2, MATLAB_PARA_kp_wloop, MATLAB_PARA_ki_wloop, MATLAB_PARA_ki_wloop, MATLAB_PARA_Wpi_max * 0.5f, MATLAB_PARA_Wpi_min * 0.5f);

    PIctrl_init(&UdcPI, MATLAB_PARA_kp_vloop, MATLAB_PARA_ki_vloop, MATLAB_PARA_ki_vloop, MATLAB_PARA_Pdcpi_max, MATLAB_PARA_Pdcpi_min);
    PIctrl_init(&PdcPI, MATLAB_PARA_kp_ploop, MATLAB_PARA_ki_ploop, MATLAB_PARA_ki_ploop, MATLAB_PARA_Pdcpi_max, MATLAB_PARA_Pdcpi_min);

    adrc_init(&UdcAdrc, MATLAB_PARA_ADRC_gamma, MATLAB_PARA_ADRC_c_u1, MATLAB_PARA_ADRC_c_u2, MATLAB_PARA_ADRC_beta1, MATLAB_PARA_ADRC_beta2,
        MATLAB_PARA_Wdcpi_max, MATLAB_PARA_Wdcpi_min, MATLAB_PARA_ADRC_SMCk);

    speedCal_init(&omegaEcal, &omegaFilt, 0);

    speedCal_init(&omegaEcal2, &omegaFilt2, 0);

    protect_init(&curProtect_CH1, CUR_PRCT_THD_L, CUR_PRCT_THD_H, CUR_PRCT_INTG_BASE, CUR_PRCT_INTG_MAX, CUR_PRCT_FOLW_DMAX, CUR_PRCT_FOLW_INTG_MAX);
    protect_init(&UdcProtect_CH1, UDC_PRCT_THD_L, UDC_PRCT_THD_H, UDC_PRCT_INTG_BASE, UDC_PRCT_INTG_MAX, UDC_PRCT_FOLW_DMAX, UDC_PRCT_FOLW_INTG_MAX);

    protect_init(&curProtect_CH2, CUR_PRCT_THD_L2, CUR_PRCT_THD_H2, CUR_PRCT_INTG_BASE2, CUR_PRCT_INTG_MAX2, CUR_PRCT_FOLW_DMAX2, CUR_PRCT_FOLW_INTG_MAX2);
    protect_init(&UdcProtect_CH2, UDC_PRCT_THD_L, UDC_PRCT_THD_H, UDC_PRCT_INTG_BASE, UDC_PRCT_INTG_MAX, UDC_PRCT_FOLW_DMAX, UDC_PRCT_FOLW_INTG_MAX);

    protect_init(&speedProtect, SPD_PRCT_THD_L, SPD_PRCT_THD_H, SPD_PRCT_INTG_BASE, SPD_PRCT_INTG_MAX, SPD_PRCT_FOLW_DMAX, SPD_PRCT_FOLW_INTG_MAX);

    protect_init(&tempProtect, TEMP_PRCT_THD_L, TEMP_PRCT_THD_H, TEMP_PRCT_INTG_BASE, TEMP_PRCT_INTG_MAX, TEMP_PRCT_FOLW_DMAX, TEMP_PRCT_FOLW_INTG_MAX);

    // proposed method
    OECA_init(&OECA1, &OECA_f1);
    Goertz_init(&Goertz1, 1, OECA1.sweepMax);
    Goertz_init(&Goertz2, 2, OECA1.sweepMax);

    encoder_init(&encoder1, ENCO_PPR1);
    encoder_init(&encoder2, ENCO_PPR2);
}

#pragma FUNC_ALWAYS_INLINE(sigSampTask)
static inline void sigSampTask()
{
    // 摆烂没有使用DMA或CLA来读取 eqep, 故需尽早访问寄存器, 以保持延迟一致性
    thetaEnco_raw = encoder_u16Read(&encoder1, EQEP_TYJDEV_REGS1.QPOSCNT);
    spdEnco = encoder_lowSpdCalc(&encoder1, EQEP_TYJDEV_REGS1.QCPRDLAT);

    // 角度处理
    if (CH1_angle_mode2 & 0x1u)
    {
        // 是否人为注入谐波及杂波
        static uint16_t wNoiseCnt = 0;
        wNoiseCnt++;
        rdc_inj_all = harmCalcUtil(thetaEnco_raw, 1, rdc_Acos_1, rdc_Asin_1) +
            harmCalcUtil(thetaEnco_raw, 2, rdc_Acos_2, rdc_Asin_2) +
            harmCalcUtil(thetaEnco_raw, 4, rdc_Acos_4, rdc_Asin_4) +
            rdc_wNoise_gain * wNoiseTabNeg1_1_256[wNoiseCnt & 0xffu];
        rdc_inj_raw = rdc_inj_all * (float)(65536.0 / M_PI / 2.0);
    }
    else
    {
        rdc_inj_raw = 0;
    }

    if (CH1_angle_mode2 & 0x2u)
    {
        // 是否补偿谐波
        rdc_comp_all = harmCalcUtil(thetaEnco_raw, 1, LMSanfThetaM.W[0], LMSanfThetaM.W[1]) +
            harmCalcUtil(thetaEnco_raw, 2, LMSanfThetaM.W[2], LMSanfThetaM.W[3]) +
            harmCalcUtil(thetaEnco_raw, 4, LMSanfThetaM.W[4], LMSanfThetaM.W[5]);
        rdc_comp_raw = rdc_comp_all * (float)(65536.0 / M_PI / 2.0);
    }
    else
    {
        rdc_comp_raw = 0;
    }

    thetaRaw_inter = thetaEnco_raw - thetaEnco_raw_offset + rdc_inj_raw - rdc_comp_raw;

    // ADC signals
    CH1_Iu_raw = bsp_get_CH1_Iu_adcRaw();
    CH1_Iv_raw = bsp_get_CH1_Iv_adcRaw();
    CH1_Iw_raw = bsp_get_CH1_Iw_adcRaw();
    CH1_Ifbk.U = getCurSI(CH1_Iu_raw, CH1_Iu_raw_offset);
    CH1_Ifbk.V = getCurSI(CH1_Iv_raw, CH1_Iv_raw_offset);
    CH1_Ifbk.W = getCurSI(CH1_Iw_raw, CH1_Iw_raw_offset);

    CH1_Vu_raw = bsp_get_CH1_Va_adcRaw();
    CH1_Vv_raw = bsp_get_CH1_Vb_adcRaw();
    CH1_Vw_raw = bsp_get_CH1_Vc_adcRaw();
    CH1_Vfbk.U = getBusVoltSI(CH1_Vu_raw);
    CH1_Vfbk.V = getBusVoltSI(CH1_Vv_raw);
    CH1_Vfbk.W = getBusVoltSI(CH1_Vw_raw);

    // CH1_Idc_raw = bsp_get_CH1_Idc_adcRaw();
    // CH1_Idc_SI = getCurSI(CH1_Idc_raw, CH1_Idc_raw_offset);
    // CH1_Idc = LPF_Ord2_update(&CH1_IdcFilt, CH1_Idc_SI);

    CH1_Udc_raw = bsp_get_CH1_Udc_adcRaw();
    CH1_Udc_SI = getBusVoltSI(CH1_Udc_raw);
    CH1_Udc = LPF_Ord2_update(&CH1_UdcFilt, CH1_Udc_SI);

    // Pdc = LPF_Ord2_update(&PdcFilt, CH1_Udc * CH1_Idc);

    // 电流变换，使用3相降低零漂影响
    // 由于是下桥臂电阻采样，时间太短的000矢量将使采样失真，故结合上周期的发波情况进行补偿
    // 由于涉及到过调制等麻烦事，就不改调制算法了，先凑合用
    if (CH1_svpwm.epwmU < CH1_svpwm.epwmV)
    {
        if (CH1_svpwm.epwmU < CH1_svpwm.epwmW)
        {
            // u最小
            if (CH1_svpwm.epwmU < CH1_samp_lThd)
            {
                // 插补通道，但沿用零序偏移信息
                transX_vw02u(&CH1_Ifbk);
            }
        }
        else
        {
            // w最小
            if (CH1_svpwm.epwmW < CH1_samp_lThd)
            {
                // 插补通道，但沿用零序偏移信息
                transX_uv02w(&CH1_Ifbk);
            }
        }
    }
    else
    {
        if (CH1_svpwm.epwmV < CH1_svpwm.epwmW)
        {
            // v最小
            if (CH1_svpwm.epwmV < CH1_samp_lThd)
            {
                // 插补通道，但沿用零序偏移信息
                transX_uw02v(&CH1_Ifbk);
            }
        }
        else
        {
            // w最小
            if (CH1_svpwm.epwmW < CH1_samp_lThd)
            {
                // 插补通道，但沿用零序偏移信息
                transX_uv02w(&CH1_Ifbk);
            }
        }
    }

    trans3_uvw2albe0(&CH1_Ifbk);
    transX_albe2abs2(&CH1_Ifbk);

    // calculate omega 可以用上次的角度值，区别不大
    omegaMfbk = speedCal_update(&omegaEcal, getThetaEUint(thetaRaw_inter)) * (float)(1.0 / MATLAB_PARA_RDC2ELE_RATIO);

    CH1_thetaE_inter = getThetaEpu(thetaRaw_inter);
    CH1_thetaE_sensor = getThetaEpu(thetaEnco_raw - thetaEnco_raw_offset);
    dbgErr_Sensor = thetaCal_util_angle_norm2(CH1_thetaE_sensor - CH1_thetaE_inter);

    if (CH1_angle_mode2 & 0x8u)
    {
        // 位置误差在线补偿
        MRAS_wr_update(&MRASwr, &CH1_Utar, &CH1_Ifbk);
        KFw_update(&KFw, MRASwr.thetaOut * (float)(2.0 * M_PI), getThetaESI(thetaRaw_inter));

        dbgErr_MARS = thetaCal_util_angle_norm2(CH1_thetaE_sensor - MRASwr.thetaOut);
        dbgErr_KFw = thetaCal_util_angle_norm2(CH1_thetaE_sensor - KFw.thetaOutPu);

        if (CH1_angle_mode2 & 0x10u)
        {
            // 使用来自卡尔曼滤波的角度信号
            CH1_thetaE_inter = KFw.thetaOutPu;
            speedCal_assign(&omegaEcal, KFw.x_k[1]);
        }
    }

    // 角度计算
    switch (CH1_angle_mode)
    {
    case AM_sync_comp:
        thetaCal_setTheta_noWrap(&CH1_thetaI, CH1_thetaE_inter);
        thetaCal_setTheta(&CH1_thetaU, thetaComp(thetaCal_getTheta(&CH1_thetaI), omegaEcal.omegaE));
        targetThetaE_CH1 = thetaCal_getTheta(&CH1_thetaI);
        break;

    case AM_sync:
        thetaCal_setTheta_noWrap(&CH1_thetaI, CH1_thetaE_inter);
        thetaCal_setTheta_noWrap(&CH1_thetaU, thetaCal_getTheta(&CH1_thetaI));
        targetThetaE_CH1 = thetaCal_getTheta(&CH1_thetaI);
        break;

    case AM_self_inc:
        thetaCal_setTheta(&CH1_thetaU, thetaCal_getTheta(&CH1_thetaU) + thetaEInc);
        thetaCal_setTheta_noWrap(&CH1_thetaI, thetaCal_getTheta(&CH1_thetaU));
        targetThetaE_CH1 = thetaCal_getTheta(&CH1_thetaI);
        break;

    case AM_manual:
    default:
        thetaCal_setTheta(&CH1_thetaU, targetThetaE_CH1);
        thetaCal_setTheta(&CH1_thetaI, targetThetaE_CH1);
        break;
    }

    trans3_albe02dq0(&CH1_Ifbk, &CH1_thetaI);
    CH1_Ifilt.d = LPF_Ord2_update(&CH1_IdFilt, CH1_Ifbk.d);
    CH1_Ifilt.q = LPF_Ord2_update(&CH1_IqFilt, CH1_Ifbk.q);
    CH1_Ifilt.abdq0 = LPF_Ord2_update(&CH1_I0Filt, CH1_Ifbk.abdq0);

    // 电压变换
    trans3_uvw2dq0(&CH1_Vfbk, &CH1_thetaI);

    // 反变换得到滤波后的三相基波电流，此处不考虑零序
    // 使用电压的角度，以补偿控制造成的延后（有待证明？）
    trans2_dq2uvw(&CH1_Ifilt, &CH1_thetaU);

    // 磁链标定用电压滤波
    LPF_Ord1_update_kahan(&CH1_UTarFiltd, CH1_Utar.d);
    LPF_Ord1_update_kahan(&CH1_UTarFiltq, CH1_Utar.q);

    if (CH1_angle_mode2 & 0x4u)
    {
        // 误差辨识
        CH1_Ifilt2.d = LPF_Ord2_update_kahan(&CH1_IdFilt_2, CH1_Ifbk.d);
        CH1_Ifilt2.q = LPF_Ord2_update_kahan(&CH1_IqFilt_2, CH1_Ifbk.q);
        // 防止除零
        CH1_Ifilt2.abs2 = fmaxf(transX_dq2abs2(&CH1_Ifilt2), 1.0f);

        thetaEst = (CH1_Ifbk.d * CH1_Ifilt2.q - CH1_Ifbk.q * CH1_Ifilt2.d) * (float)(1.0f / MATLAB_PARA_RDC2ELE_RATIO) / CH1_Ifilt2.abs2;
        thetaEstFilt = LMSanfUpdate(&LMSanfThetaM, thetaEnco_raw, thetaEst);
    }
}

#pragma FUNC_ALWAYS_INLINE(sigSampTask2)
static inline void sigSampTask2()
{
    // 摆烂没有使用DMA或CLA来读取 eqep, 故需尽早访问寄存器, 以保持延迟一致性
    thetaEnco_raw2 = encoder_u16Read(&encoder2, EQEP_TYJDEV_REGS2.QPOSCNT);
    spdEnco2 = encoder_lowSpdCalc(&encoder2, EQEP_TYJDEV_REGS2.QCPRDLAT);

    // ADC signals
    CH2_Iu_raw = bsp_get_CH2_Iu_adcRaw();
    CH2_Iv_raw = bsp_get_CH2_Iv_adcRaw();
    CH2_Iw_raw = bsp_get_CH2_Iw_adcRaw();
    CH2_Ifbk.U = getCurSI(CH2_Iu_raw, CH2_Iu_raw_offset);
    CH2_Ifbk.V = getCurSI(CH2_Iv_raw, CH2_Iv_raw_offset);
    CH2_Ifbk.W = getCurSI(CH2_Iw_raw, CH2_Iw_raw_offset);

    CH2_Vu_raw = bsp_get_CH2_Va_adcRaw();
    CH2_Vv_raw = bsp_get_CH2_Vb_adcRaw();
    CH2_Vw_raw = bsp_get_CH2_Vc_adcRaw();
    CH2_Vfbk.U = getBusVoltSI(CH2_Vu_raw);
    CH2_Vfbk.V = getBusVoltSI(CH2_Vv_raw);
    CH2_Vfbk.W = getBusVoltSI(CH2_Vw_raw);

    // CH2_Idc_raw = bsp_get_CH2_Idc_adcRaw();
    // CH2_Idc_SI = getCurSI(CH2_Idc_raw, CH2_Idc_raw_offset);
    // CH2_Idc = LPF_Ord2_update(&CH2_IdcFilt, CH2_Idc_SI);

    CH2_Udc_raw = bsp_get_CH2_Udc_adcRaw();
    CH2_Udc_SI = getBusVoltSI(CH2_Udc_raw);
    CH2_Udc = LPF_Ord2_update(&CH2_UdcFilt, CH2_Udc_SI);

    // Pdc2 = LPF_Ord2_update(&PdcFilt2, CH2_Udc * CH2_Idc);

    // 电流变换，使用3相降低零漂影响
    // 由于是下桥臂电阻采样，时间太短的000矢量将使采样失真，故结合上周期的发波情况进行补偿
    // 由于涉及到过调制等麻烦事，就不改调制算法了，先凑合用
    if (CH2_svpwm.epwmU < CH2_svpwm.epwmV)
    {
        if (CH2_svpwm.epwmU < CH2_svpwm.epwmW)
        {
            // u最小
            if (CH2_svpwm.epwmU < CH2_samp_lThd)
            {
                // 插补通道，但沿用零序偏移信息
                transX_vw02u(&CH2_Ifbk);
            }
        }
        else
        {
            // w最小
            if (CH2_svpwm.epwmW < CH2_samp_lThd)
            {
                // 插补通道，但沿用零序偏移信息
                transX_uv02w(&CH2_Ifbk);
            }
        }
    }
    else
    {
        if (CH2_svpwm.epwmV < CH2_svpwm.epwmW)
        {
            // v最小
            if (CH2_svpwm.epwmV < CH2_samp_lThd)
            {
                // 插补通道，但沿用零序偏移信息
                transX_uw02v(&CH2_Ifbk);
            }
        }
        else
        {
            // w最小
            if (CH2_svpwm.epwmW < CH2_samp_lThd)
            {
                // 插补通道，但沿用零序偏移信息
                transX_uv02w(&CH2_Ifbk);
            }
        }
    }

    trans3_uvw2albe0(&CH2_Ifbk);
    transX_albe2abs2(&CH2_Ifbk);

    // calculate omega 可以用上次的角度值，区别不大
    omegaMfbk2 = speedCal_update(&omegaEcal2, getThetaEUint(thetaEnco_raw2)) * (float)(1.0 / MATLAB_PARA_RDC2ELE_RATIO);

    // 角度计算
    switch (CH2_angle_mode)
    {
    case AM_sync_comp:
        thetaCal_setTheta_Uint(&CH2_thetaI, getThetaEUint(thetaEnco_raw2 - thetaEnco_raw_offset2));
        thetaCal_setTheta(&CH2_thetaU, thetaComp(thetaCal_getTheta(&CH2_thetaI), omegaEcal2.omegaE));
        targetThetaE_CH2 = thetaCal_getTheta(&CH2_thetaI);
        break;

    case AM_sync:
        thetaCal_setTheta_Uint(&CH2_thetaI, getThetaEUint(thetaEnco_raw2 - thetaEnco_raw_offset2));
        thetaCal_setTheta_noWrap(&CH2_thetaU, thetaCal_getTheta(&CH2_thetaI));
        targetThetaE_CH2 = thetaCal_getTheta(&CH2_thetaI);
        break;

    case AM_self_inc:
        thetaCal_setTheta(&CH2_thetaU, thetaCal_getTheta(&CH2_thetaU) + thetaEInc2);
        thetaCal_setTheta_noWrap(&CH2_thetaI, thetaCal_getTheta(&CH2_thetaU));
        targetThetaE_CH2 = thetaCal_getTheta(&CH2_thetaI);
        break;

    case AM_manual:
    default:
        thetaCal_setTheta(&CH2_thetaU, targetThetaE_CH2);
        thetaCal_setTheta(&CH2_thetaI, targetThetaE_CH2);
        break;
    }

    trans3_albe02dq0(&CH2_Ifbk, &CH2_thetaI);
    CH2_Ifilt.d = LPF_Ord2_update(&CH2_IdFilt, CH2_Ifbk.d);
    CH2_Ifilt.q = LPF_Ord2_update(&CH2_IqFilt, CH2_Ifbk.q);
    CH2_Ifilt.abdq0 = LPF_Ord2_update(&CH2_I0Filt, CH2_Ifbk.abdq0);

    // 电压变换
    trans3_uvw2dq0(&CH2_Vfbk, &CH2_thetaI);

    // 反变换得到滤波后的三相基波电流，此处不考虑零序
    // 使用电压的角度，以补偿控制造成的延后（有待证明？）
    trans2_dq2uvw(&CH2_Ifilt, &CH2_thetaU);
}

#pragma FUNC_ALWAYS_INLINE(protectIsrTask)
static inline void protectIsrTask()
{
    // 任何时候进行电流绝对值保护和有效值保护
    IProtectFlg_CH1 |=
        protectCalc_THD_HL(&curProtect_CH1, CH1_Ifbk.U) |
        protectCalc_THD_HL(&curProtect_CH1, CH1_Ifbk.V) |
        protectCalc_THD_HL(&curProtect_CH1, CH1_Ifbk.W) |
        protectCalc_INTG2(&curProtect_CH1, CH1_Ifbk.abs2) |
        0;

    // 部分模式进行电流跟踪保护
    if (CH1_cur_mode == CM_I_loop)
    {
        IProtectFlg_CH1 |=
            protectCalc_FOLW2(&curProtect_CH1, CH1_Ifilt.q, targetIq_CH1) |
            protectCalc_FOLW2(&curProtect_CH1, CH1_Ifilt.d, targetId_CH1) |
            0;
    }
    else
    {
        protect_FOLW_Clear(&curProtect_CH1);
    }

    // 功率模块硬件故障保护
    if (bsp_GPIO_ifHWFaultSigSet())
    {
        filtHWFaultCnt++;
        if (filtHWFaultCnt > 3)
        {
            IProtectFlg_CH1 |= 0x10u;
            // 触发读取错误位
            drv8305_1.ii = 0;
        }
    }
    else
    {
        filtHWFaultCnt = 0;
    }

    // 任何时候进行转速绝对值保护
    SPDProtectFlg |=
        protectCalc_THD_HL(&speedProtect, omegaMfbk) |
        0;

    // 部分模式进行转速跟踪保护
    if (speed_mode >= SM_speed_loop_N && speed_mode <= SM_speed_loop_ramp_N)
    {
        SPDProtectFlg |=
            protectCalc_FOLW2(&speedProtect, omegaMfbk, targetOmegaM) |
            0;
    }
    else
    {
        protect_FOLW_Clear(&speedProtect);
    }

    // 任何时候进行母线电压绝对值保护
    UdcProtectFlg_CH1 |=
        protectCalc_THD_HL(&UdcProtect_CH1, CH1_Udc) |
        0;

    // 部分模式下进行电压跟踪保护
    if (speed_mode >= SM_Udc_loop)
    {
        UdcProtectFlg_CH1 |=
            protectCalc_FOLW2(&UdcProtect_CH1, CH1_Udc, targetUdc) |
            0;
    }
    else
    {
        protect_FOLW_Clear(&UdcProtect_CH1);
    }

    // 保护结果判定
    if (IProtectFlg_CH1 | UdcProtectFlg_CH1 | tempProtectFlg)
    {
        CH1_cur_mode = CM_error;
        channel_mode &= 0b11111110;
        speed_mode = SM_err;
    }

    if (SPDProtectFlg | tempProtectFlg | UdcProtectFlg_CH1)
    {
        speed_mode = SM_err;
        channel_mode = OM_stop;
    }

    // 手动停机
    if (bsp_GPIO_ifStopSigSet())
    {
        speed_mode = SM_stop;
        channel_mode = OM_stop;
        CH1_cur_mode = CM_stop;
        IProtectFlg_CH1 |= 0x20u;
    }
}

#pragma FUNC_ALWAYS_INLINE(protectIsrTask2)
static inline void protectIsrTask2()
{
    // 任何时候进行电流绝对值保护和有效值保护
    IProtectFlg_CH2 |=
        protectCalc_THD_HL(&curProtect_CH2, CH2_Ifbk.U) |
        protectCalc_THD_HL(&curProtect_CH2, CH2_Ifbk.V) |
        protectCalc_THD_HL(&curProtect_CH2, CH2_Ifbk.W) |
        protectCalc_INTG2(&curProtect_CH2, CH2_Ifbk.abs2) |
        0;

    // 部分模式进行电流跟踪保护
    if (CH2_cur_mode == CM_I_loop)
    {
        IProtectFlg_CH2 |=
            protectCalc_FOLW2(&curProtect_CH2, CH2_Ifilt.q, targetIq_CH2) |
            protectCalc_FOLW2(&curProtect_CH2, CH2_Ifilt.d, targetId_CH2) |
            0;
    }
    else
    {
        protect_FOLW_Clear(&curProtect_CH2);
    }

    // 功率模块硬件故障保护
    if (bsp_GPIO_ifHWFaultSigSet2())
    {
        filtHWFaultCnt2++;
        if (filtHWFaultCnt2 > 3)
        {
            IProtectFlg_CH2 |= 0x10u;
            // 触发读取错误位
            drv8305_2.ii = 0;
        }
    }
    else
    {
        filtHWFaultCnt2 = 0;
    }

    // 任何时候进行转速绝对值保护
    SPDProtectFlg |=
        protectCalc_THD_HL(&speedProtect, omegaMfbk2) |
        0;

    // 部分模式进行转速跟踪保护
    if (speed_mode >= SM_speed_loop_N && speed_mode <= SM_speed_loop_ramp_N)
    {
        SPDProtectFlg |=
            protectCalc_FOLW2(&speedProtect, omegaMfbk2, targetOmegaM2) |
            0;
    }
    else
    {
        protect_FOLW_Clear(&speedProtect);
    }

    // 任何时候进行母线电压绝对值保护
    UdcProtectFlg_CH2 |=
        protectCalc_THD_HL(&UdcProtect_CH2, CH2_Udc) |
        0;

    // 保护结果判定
    if (IProtectFlg_CH2 | UdcProtectFlg_CH2 | tempProtectFlg)
    {
        CH2_cur_mode = CM_error;
        channel_mode2 &= 0b11111110;
        speed_mode2 = SM_err;
    }


    if (SPDProtectFlg | tempProtectFlg | UdcProtectFlg_CH1)
    {
        speed_mode2 = SM_err;
        channel_mode2 = OM_stop;
    }

    // 手动停机
    if (bsp_GPIO_ifStopSigSet())
    {
        speed_mode2 = SM_stop;
        channel_mode2 = OM_stop;
        CH2_cur_mode = CM_stop;
        IProtectFlg_CH2 |= 0x20u;
    }
}

#pragma FUNC_ALWAYS_INLINE(curLoopTask)
static inline void curLoopTask()
{
    switch (CH1_cur_mode)
    {
    case CM_LSSC:
        bsp_epwm_ch1_LSSC2(CH1_cur_mode2);
        break;

    case CM_I_loop:
        // 虽增加平均计算量，但方便debug，且不影响峰值计算量
        PsiLUT(&CH1_Ifilt, &CH1_Psi);
        CH1_Ld4PI = Ld1dLUT(CH1_Ifilt.d);
        CH1_Lq4PI = Lq1dLUT(CH1_Ifilt.q);
        if (CH1_ext_fcn & 0x8u)
        {
            CH1_Ud_comp = -CH1_Psi.q * omegaEcal.omegaE;
            CH1_Uq_comp = CH1_Psi.d * omegaEcal.omegaE;

        }
        else if (CH1_ext_fcn & 0x2u)
        {
            CH1_Ud_comp = -CH1_Ifilt.q * Lq4Icomp * omegaEcal.omegaE;
            CH1_Uq_comp = (CH1_Ifilt.d * Ld4Icomp + Faif4Icomp) * omegaEcal.omegaE;
        }
        else
        {
            CH1_Ud_comp = 0;
            CH1_Uq_comp = 0;
        }

        if (CH1_ext_fcn & 0x10u)
        {
            PIctrl_Iloop_cfg2(&CH1_IdPI, MATLAB_PARA_Iloop_bw_factor, CH1_Ld4PI, MRASwr.Rs);
            PIctrl_Iloop_cfg2(&CH1_IqPI, MATLAB_PARA_Iloop_bw_factor, CH1_Lq4PI, MRASwr.Rs);
        }

        CH1_Utar.d = PIctrl_update_clamp_comp2(&CH1_IdPI, targetId_CH1 - CH1_Ifilt.d, CH1_Ud_comp);
        CH1_Utar.q = PIctrl_update_clamp_comp2(&CH1_IqPI, targetIq_CH1 - CH1_Ifilt.q, CH1_Uq_comp);

    case CM_svpwm:
        if (CH1_ext_fcn & 0x1u)
        {
            float tempUdc1 = CH1_Udc;
            if (CH1_ext_fcn & 0x4u)
            {
                // 死区补偿 part1
                tempUdc1 = dbComp2_UdcOffset(CH1_Udc, db_cmp_vds);
            }
            SVPWM_setUdc(&CH1_svpwm, tempUdc1);
            // 考虑母线电压缓变性及抗饱和保护的不敏感性，在这里进行抗饱和参数的变化
            const float SQRT_3_DIV_3 = 0.5773503f;
            float Usv_max = CH1_Udc * SQRT_3_DIV_3;
            PIctrl_svpwmBoundSet(&CH1_IdPI, Usv_max);
            PIctrl_svpwmBoundSet(&CH1_IqPI, Usv_max);
        }
        trans2_dq2albe(&CH1_Utar, &CH1_thetaU);
        CH1_cur_sta |= SVPWM_dutyCal(&CH1_svpwm, CH1_Utar.al, CH1_Utar.be);
        if (CH1_ext_fcn & 0x4u)
        {
            // 死区补偿 part2
            // 注意这里是单采单更的，与MATLAB指令并不一致
            dbComp2_all(&CH1_svpwm, &CH1_Ifilt, db_Ithd_1, db_cmp_tick);
        }
        bsp_epwm_ch1_reg_set(&CH1_svpwm);
        bsp_epwm_ch1_on();
        break;

    default:
    case CM_stop:
        // 归零指令值
        CH1_Utar.d = 0;
        CH1_Utar.q = 0;
        targetId_CH1 = 0;
        targetIq_CH1 = 0;
        CH1_Ud_comp = 0;
        CH1_Uq_comp = 0;
        // 清空控制器
        PIctrl_setIntg(&CH1_IdPI, 0);
        PIctrl_setIntg(&CH1_IqPI, 0);
        // 归零电压
        trans2_dq2albe(&CH1_Utar, &CH1_thetaU);
        SVPWM_init(&CH1_svpwm, CH1_Udc);
        bsp_epwm_ch1_reg_init();

    case CM_error:
        // 封锁驱动
        bsp_epwm_ch1_off();
        break;
    }
}

#pragma FUNC_ALWAYS_INLINE(curLoopTask2)
static inline void curLoopTask2()
{
    switch (CH2_cur_mode)
    {
    case CM_LSSC:
        bsp_epwm_ch2_LSSC2(CH2_cur_mode2);
        break;

    case CM_I_loop:
        if (CH2_ext_fcn & 0x2u)
        {
            CH2_Ud_comp = -CH2_Ifilt.q * Lq4Icomp2 * omegaEcal2.omegaE;
            CH2_Uq_comp = (CH2_Ifilt.d * Ld4Icomp2 + Faif4Icomp2) * omegaEcal2.omegaE;
        }
        else
        {
            CH2_Ud_comp = 0;
            CH2_Uq_comp = 0;
        }

        CH2_Utar.d = PIctrl_update_clamp_comp2(&CH2_IdPI, targetId_CH2 - CH2_Ifilt.d, CH2_Ud_comp);
        CH2_Utar.q = PIctrl_update_clamp_comp2(&CH2_IqPI, targetIq_CH2 - CH2_Ifilt.q, CH2_Uq_comp);

    case CM_svpwm:
        if (CH2_ext_fcn & 0x1u)
        {
            float tempUdc2 = CH2_Udc;
            if (CH2_ext_fcn & 0x4u)
            {
                // 死区补偿 part1
                tempUdc2 = dbComp2_UdcOffset(CH2_Udc, db2_cmp_vds);
            }
            SVPWM_setUdc(&CH2_svpwm, tempUdc2);
            // 考虑母线电压缓变性及抗饱和保护的不敏感性，在这里进行抗饱和参数的变化
            const float SQRT_3_DIV_3 = 0.5773503f;
            float Usv_max2 = CH2_Udc * SQRT_3_DIV_3;
            PIctrl_svpwmBoundSet(&CH2_IdPI, Usv_max2);
            PIctrl_svpwmBoundSet(&CH2_IqPI, Usv_max2);
        }
        trans2_dq2albe(&CH2_Utar, &CH2_thetaU);
        CH2_cur_sta |= SVPWM_dutyCal(&CH2_svpwm, CH2_Utar.al, CH2_Utar.be);
        if (CH2_ext_fcn & 0x4u)
        {
            // 死区补偿 part2
            // 此处的 MATLAB_PARA_ctrl_freq_mul = 1 !!!
            // 注意这里是单采单更的，与MATLAB指令并不一致
            dbComp2_all(&CH2_svpwm, &CH2_Ifilt, db2_Ithd_1, db2_cmp_tick);
        }
        bsp_epwm_ch2_reg_set(&CH2_svpwm);
        bsp_epwm_ch2_on();
        break;

    default:
    case CM_stop:
        // 归零指令值
        CH2_Utar.d = 0;
        CH2_Utar.q = 0;
        targetId_CH2 = 0;
        targetIq_CH2 = 0;
        CH2_Ud_comp = 0;
        CH2_Uq_comp = 0;
        // 清空控制器
        PIctrl_setIntg(&CH2_IdPI, 0);
        PIctrl_setIntg(&CH2_IqPI, 0);
        // 归零电压
        trans2_dq2albe(&CH2_Utar, &CH2_thetaU);
        SVPWM_init(&CH2_svpwm, CH2_Udc);
        bsp_epwm_ch2_reg_init();

    case CM_error:
        // 封锁驱动
        bsp_epwm_ch2_off();
        break;
    }
}

#pragma FUNC_ALWAYS_INLINE(outerLoopTask)
static inline void outerLoopTask()
{
    // 目标给定
    switch (speed_mode)
    {

    case SM_speed_loop_N:
        targetOmegaM = targetN * (float)(2.0 * M_PI / 60.0);
        break;

    case SM_speed_loop_ramp_N:
        targetN = targetRamp(targetRampVal, targetN, targetRampGrad * vCTRL_TS);
        targetOmegaM = targetN * (float)(2.0 * M_PI / 60.0);
        break;

    case SM_Pdc_loop_ramp:
        targetPdc = targetRamp(targetRampVal, targetPdc, targetRampGrad * vCTRL_TS);
        break;

    case SM_Udc_loop_ramp:
    case SM_Udc_loop_ramp2:
        targetUdc = targetRamp(targetRampVal, targetUdc, targetRampGrad * vCTRL_TS);
        break;
    }

    // 闭环计算
    switch (speed_mode)
    {
    case SM_Udc_loop2:
    case SM_Udc_loop_ramp2:
        if (ADRC_mode)
        {
            targetTe = adrc_update2(&UdcAdrc, targetUdc, CH1_Udc, omegaMfbk);
        }
        else
        {
            targetTe = adrc_update(&UdcAdrc, targetUdc, CH1_Udc, omegaMfbk);
        }
        break;

    case SM_Udc_loop:
    case SM_Udc_loop_ramp:
        targetTe = PIctrl_update_clamp_comp2(&UdcPI, getDeltaCapEnSI(CH1_Udc, targetUdc), Pdc) * omegaEcal.omegaE_inv * MATLAB_PARA_p0;
        break;

    case SM_Pdc_loop:
    case SM_Pdc_loop_ramp:
        targetTe = PIctrl_update_clamp2(&PdcPI, targetPdc - Pdc) * omegaEcal.omegaE_inv * MATLAB_PARA_p0;
        break;

    case SM_speed_loop_N:
    case SM_speed_loop_ramp_N:
        targetTe = PIctrl_update_bCalc2(&omegaPI, targetOmegaM - omegaMfbk);
        break;

    case SM_OECA_exp:
        switch (OECA1.status)
        {
        default:
        case OECA_ready:
            targetIs = 0;
            targetThetaMTPA = 0;
            OECA1.sweepCnt = 0;
            OECA1.sampCnt = 0;
            OECA1.timeCnt = 0;
            OECA1.dbgCnt = 0;
            OECA1.dbgPtr = 0;
            OECA1.sampDiv = 30e3f * OECA1.sweepTime / OECA1.sweepMax + 0.5f;
            OECA1.timeCntMax = 30e3f * OECA1.sweepTime + 0.5f;
            Goertz_init(&Goertz1, 1, OECA1.sweepMax);
            Goertz_init(&Goertz2, 2, OECA1.sweepMax);
            OECA_algoStaSet(&OECA1, 0.0f);
            OECA_omegaOB(&OECA1, thetaEnco_raw * (1.0f / 65536.0f));
            break;

        case OECA_Isweep0:
        {
            targetIs = OECA1.sweepCurrent;
            targetThetaMTPA = OECA_getSweepTheta(&OECA1);
            OECA1.sampCnt++;
            OECA1.timeCnt++;
            OECA1.dbgCnt++;
            float thetaRdcRawNorm = thetaEnco_raw * (1.0f / 65536.0f);
            OECA_omegaOB(&OECA1, thetaRdcRawNorm);
            if (OECA1.dbgCnt >= OECA1.dbgDiv)
            {
                OECA1.dbgCnt = 0;
                // dbg 记录
                OECA1.hDbgBuff[OECA1.dbgPtr * 2 + 0] = thetaRdcRawNorm;
                OECA1.hDbgBuff[OECA1.dbgPtr * 2 + 1] = OECA1.omegaOB;
                OECA1.dbgPtr++;
            }
            if (OECA1.sampCnt >= OECA1.sampDiv)
            {
                OECA1.sampCnt = 0;
                Goertz_iter(&Goertz1, OECA1.omegaOB);
                Goertz_iter(&Goertz2, OECA1.omegaOB);

                OECA1.sweepCnt++;
            }
            if (OECA1.sweepCnt >= OECA1.sweepMax)
            {
                // 跳转状态，关闭输出
                OECA1.status = OECA_Isweep1;
                targetIs = 0;
                // 计算结果，只算一次
                Goertz_getAns(&Goertz1);
                Goertz_getAns(&Goertz2);
                OECA1.estAns1 = OECA_util_angle_norm(Goertz1.AnsArg - 0.5f);
            }
        }

        break;

        case OECA_Isweep1:
            targetIs = 0.0f;
            targetThetaMTPA = 0.0f;

            OECA_omegaOB(&OECA1, thetaEnco_raw * (1.0f / 65536.0f));
            // 参数设计
            if (OECA1.modeCfg & OECA_M_AUTO_PI_PARA)
            {
                {
                    float omegaSlop = (float)(2.0 * M_PI * 30e3f) / (1.0f * OECA1.sweepMax * OECA1.sampDiv);
                    float factor = __divf32(OECA1.PICurrent, OECA1.sweepCurrent);
                    float aaa = (Goertz1.AnsAbs * factor * omegaSlop + 4.0f * Goertz2.AnsAbs * factor * factor * omegaSlop) * OECA1.J_coeff;
                    OECA1.kp_eComp = __divf32(OECA1.h_eComp + 1.0f, (float)(2.0 * 2.0 * M_PI) * aaa * OECA1.h_eComp * OECA1.T_sigma_eComp);
                    OECA1.ki_eComp = __divf32(OECA1.kp_eComp, CTRL_FREQ * OECA1.h_eComp * OECA1.T_sigma_eComp);
                }

                OECA1.modeCfg ^= OECA_M_AUTO_PI_PARA;
            }

            // 初值设定
            if (OECA1.modeCfg & OECA_M_AUTO_INIT_VAL)
            {
                OECA_algoStaSet(&OECA1, OECA1.estAns1);
                OECA1.modeCfg ^= OECA_M_AUTO_INIT_VAL;
            }

            // 自动跳转
            if (OECA1.modeCfg & OECA_M_AUTOALGO)
            {
                // 确保速度较小时再跳转
                if (fabsf(OECA1.omegaOB) < OECA1.maxOmegaMThd)
                {
                    OECA1.status = OECA_algo0;
                }
            }
            if (OECA1.modeCfg & OECA_M_AUTO_GOBACK)
            {
                OECA1.status = OECA_ready;
            }
            break;

        case OECA_algo0:
            targetIs = OECA1.PICurrent;
            OECA_omegaOB(&OECA1, thetaEnco_raw * (1.0f / 65536.0f));
            targetThetaMTPA = OECA_PICalc(&OECA1);
            OECA_pllCalc(&OECA1);
            break;

        case OECA_stop:
            targetIs = 0;
            break;
        }
        break;

    case SM_Te_ramp:
        targetTe = targetRamp(targetRampVal, targetTe, targetRampGrad * vCTRL_TS);
        break;

    case SM_Is_ramp:
        targetIs = targetRamp(targetRampVal, targetIs, targetRampGrad * vCTRL_TS);
        break;

    default:
    case SM_stop:
        // 归零指令值
        targetOmegaM = omegaMfbk;
        targetN = targetOmegaM * (float)(60.0 / 2.0 / M_PI);
        targetTe = 0;
        targetIs = 0;
        targetUdc = CH1_Udc;
        targetPdc = Pdc;
        // 清空控制器
        PIctrl_setIntg(&omegaPI, 0);
        PIctrl_setIntg(&UdcPI, 0);
        PIctrl_setIntg(&PdcPI, 0);

        adrc_clear(&UdcAdrc);
        break;

    case SM_err:
        // 保留现场
        break;
    }

    // 输出环
    switch (channel_mode)
    {
    default:
    case OM_stop:
        targetId_CH1 = 0;
        targetIq_CH1 = 0;
        break;

    case OM_CH1:
        switch (torque_mode)
        {
        case TM_id0:
            targetIq_CH1 = targetTe * (1.0f / MATLAB_PARA_faif / MATLAB_PARA_p0 / 1.5f);
            break;

        default:
        case TM_MTPA:
        {
            float te_tab = targetTe * 1.0f + TE_IDQ_MTPA_2WAY_TAB_OFFSET;
            targetIq_CH1 = lookUp_1d_lin_puX(te_tab, TE_IQ_MTPA_2WAY_TAB, TE_IDQ_MTPA_2WAY_TAB_MAX);
            targetId_CH1 = lookUp_1d_lin_puX(te_tab, TE_ID_MTPA_2WAY_TAB, TE_IDQ_MTPA_2WAY_TAB_MAX);
            break;
        }
        }
        break;

    case OM_free:
        switch (torque_mode)
        {
        case TM_id0:
            targetIq_CH1 = targetIs;
            break;

        default:
        case TM_MTPA:
            thetaCal_setTheta(&thetaMTPA, targetThetaMTPA);
            targetIq_CH1 = targetIs * thetaCal_getSinVal(&thetaMTPA);
            targetId_CH1 = targetIs * thetaCal_getCosVal(&thetaMTPA);
            break;
        }
        break;
    }
}

#pragma FUNC_ALWAYS_INLINE(outerLoopTask2)
static inline void outerLoopTask2()
{
    // 目标给定
    switch (speed_mode2)
    {
    case SM_speed_loop_N:
        targetOmegaM2 = targetN2 * (float)(2.0 * M_PI / 60.0);
        break;

    case SM_speed_loop_ramp_N:
        targetN2 = targetRamp(targetRampVal2, targetN2, targetRampGrad2 * vCTRL_TS);
        targetOmegaM2 = targetN2 * (float)(2.0 * M_PI / 60.0);
        break;
    }

    // 闭环计算
    switch (speed_mode2)
    {
    case SM_Udc_loop2:
    case SM_Udc_loop_ramp2:
        // not supported
        break;

    case SM_Udc_loop:
    case SM_Udc_loop_ramp:
        // not supported
        break;

    case SM_Pdc_loop:
    case SM_Pdc_loop_ramp:
        // not supported
        break;

    case SM_speed_loop_N:
    case SM_speed_loop_ramp_N:
        targetTe2 = PIctrl_update_bCalc2(&omegaPI2, targetOmegaM2 - omegaMfbk2);
        break;

    case SM_OECA_exp:
        // not supported
        break;

    case SM_Te_ramp:
        targetTe2 = targetRamp(targetRampVal2, targetTe2, targetRampGrad2 * vCTRL_TS);
        break;

    case SM_Is_ramp:
        targetIs2 = targetRamp(targetRampVal2, targetIs2, targetRampGrad2 * vCTRL_TS);
        break;

    default:
    case SM_stop:
        // 归零指令值
        targetOmegaM2 = omegaMfbk2;
        targetN2 = targetOmegaM2 * (float)(60.0 / 2.0 * M_PI);
        targetTe2 = 0;
        targetIs2 = 0;
        // 清空控制器
        PIctrl_setIntg(&omegaPI2, 0);

        break;

    case SM_err:
        // 保留现场
        break;
    }

    // 输出环
    switch (channel_mode2)
    {
    default:
    case OM_stop:
        targetId_CH2 = 0;
        targetIq_CH2 = 0;
        break;

    case OM_CH1:
        switch (torque_mode2)
        {
        case TM_id0:
            targetIq_CH2 = targetTe2 * (1.0f / DYNO_PARA_faif / DYNO_PARA_p0 / 1.5f);
            break;

        default:
        case TM_MTPA:
        {
            targetIq_CH2 = targetTe2 * (1.0f / DYNO_PARA_faif / DYNO_PARA_p0 / 1.5f);
            // 固定功角的MTPA
            thetaCal_setTheta(&thetaMTPA2, targetThetaMTPA2);
            float tan_thetaMTPA_1 = thetaCal_getSinVal(&thetaMTPA2) != 0 ? thetaCal_getCosVal(&thetaMTPA2) / thetaCal_getSinVal(&thetaMTPA2) : 0.0f;
            targetId_CH2 = targetIq_CH2 * tan_thetaMTPA_1;
            break;
        }
        }
        break;

    case OM_free:
        switch (torque_mode2)
        {
        case TM_id0:
            targetIq_CH2 = targetIs2;
            break;

        default:
        case TM_MTPA:
            thetaCal_setTheta(&thetaMTPA2, targetThetaMTPA2);
            targetIq_CH2 = targetIs2 * thetaCal_getSinVal(&thetaMTPA2);
            targetId_CH2 = targetIs2 * thetaCal_getCosVal(&thetaMTPA2);
            break;

        }
        break;

    }
}

#pragma CODE_SECTION(mainIsrProcess, MEM_MACRO);
void mainIsrProcess()
{
    sigSampTask();
    protectIsrTask();
    outerLoopTask();
    curLoopTask();

    return;
}

#pragma CODE_SECTION(mainIsrProcess2, MEM_MACRO);
void mainIsrProcess2()
{
    sigSampTask2();
    protectIsrTask2();
    outerLoopTask2();
    curLoopTask2();

    return;
}

void protectMainLoopTask()
{
    const float tempLpfCoeff = 0.1;

    // 获得温度, 这个1ms才执行一次，无需判断是否完成转换，直接读即可

    NTC_raw = (int32_t)bsp_get_ntc1_adcRaw() - 32768;
    NTC_ref_raw = (int32_t)bsp_get_ntcRef1_adcRaw() - 32768;

    // NTC 温度计算
    {
        // 只关心绝对值
        float temp = 1.0f / (logf(fabsf(NTC_Rref_R25 * NTC_raw / NTC_ref_raw)) / NTC_B + 1 / NTC_T0) - 273.15f;
        // 简单LPF
        NTC_temp = tempLpfCoeff * (temp - NTC_temp) + NTC_temp;
    }

    // 温度保护
    tempProtectFlg |=
        protectCalc_THD_HL(&tempProtect, NTC_temp) |
        0;

    // 触发下一次adc
    bsp_adc_swTrig_ntc();

    return;
}

void mainLoopProcess()
{
    if (bsp_tim0_polling_OF())
    {
        // 1ms间隔已到
        // 清空标志位
        bsp_tim0_clearFlg_OF();

        // 无NTC
        // protectMainLoopTask();

        drv8305_idleErrCheck(&drv8305_1);
        drv8305_idleErrCheck2(&drv8305_2);
    }

    return;
}
