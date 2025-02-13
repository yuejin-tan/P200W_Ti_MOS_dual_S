/*
 * interp_tab.c
 *
 *  Created on: 2023年5月9日
 *      Author: t
 */

#include "interp_tab.h"
#include "algo_code_config.h"
#include "psi_lut.h"
#include "bsp_cfg.h"

 // MTPA情况下转矩和用D轴电流的对应关系，对应转矩为0，1，2……23
const float TE_ID_MTPA_2WAY_TAB[] = {
-95.1993F, //-25
-90.9582F, //-24
-86.7171F, //-23
-82.4760F, //-22
-78.2349F, //-21
-73.9938F,
-69.8192F,
-65.9019F,
-62.0809F,
-58.1454F,
-54.5526F,
-50.7952F,
-47.0607F,
-43.5219F,
-40.1762F,
-36.4646F,
-32.7767F,
-29.1138F,
-25.4885F,
-21.8242F,
-18.0281F,
-14.3423F,
-10.4731F,
-6.2729F,
-2.4611F,
0.0000F,  //0
-2.4611F,
-6.2729F,
-10.4731F,
-14.3423F,
-18.0281F,
-21.8242F,
-25.4885F,
-29.1138F,
-32.7767F,
-36.4646F,
-40.1762F,
-43.5219F,
-47.0607F,
-50.7952F,
-54.5526F,
-58.1454F,
-62.0809F,
-65.9019F,
-69.8192F,
-73.9938F, //20
-78.2349F, //21
-82.4760F, //22
-86.7171F, //23
-90.9582F, //24
-95.1993F, //25
};

// MTPA情况下转矩和用Q轴电流的对应关系，对应转矩为0，1，2……23
const float TE_IQ_MTPA_2WAY_TAB[] = {
-67.8156F, //-25
-66.4327F, //-24
-65.0498F, //-23
-63.6669F, //-22
-62.2840F, //-21
-60.9011F,
-59.3975F,
-57.8522F,
-56.2140F,
-54.3850F,
-52.5832F,
-50.5565F,
-48.3895F,
-46.1867F,
-44.0005F,
-41.7293F,
-39.3986F,
-37.0031F,
-34.5279F,
-31.8822F,
-28.9340F,
-25.7788F,
-22.0103F,
-17.0049F,
-10.5501F,
0.0000F,
10.5501F,
17.0049F,
22.0103F,
25.7788F,
28.9340F,
31.8822F,
34.5279F,
37.0031F,
39.3986F,
41.7293F,
44.0005F,
46.1867F,
48.3895F,
50.5565F,
52.5832F,
54.3850F,
56.2140F,
57.8522F,
59.3975F,
60.9011F,
62.2840F, //21
63.6669F, //22
65.0498F, //23
66.4327F, //24
67.8156F, //25
};

// 磁链查找表
const float psiLUTis_1 = (16.0 - 1.0) / 10.0;

const float psidLUTpu16negIdxIqy[16 * 16] = { 0.0075, 0.0075, 0.0075, 0.0075, 0.0075, 0.0075, 0.0075, 0.0075, 0.0075, 0.0075, 0.0075, 0.0075, 0.0075, 0.0075, 0.0075, 0.0075, 0.007413333333333333, 0.007413333333333333, 0.007413333333333333, 0.007413333333333333, 0.007413333333333333, 0.007413333333333333, 0.007413333333333333, 0.007413333333333333, 0.007413333333333333, 0.007413333333333333, 0.007413333333333333, 0.007413333333333333, 0.007413333333333333, 0.007413333333333333, 0.007413333333333333, 0.007413333333333333, 0.007326666666666666, 0.007326666666666666, 0.007326666666666666, 0.007326666666666666, 0.007326666666666666, 0.007326666666666666, 0.007326666666666666, 0.007326666666666666, 0.007326666666666666, 0.007326666666666666, 0.007326666666666666, 0.007326666666666666, 0.007326666666666666, 0.007326666666666666, 0.007326666666666666, 0.007326666666666666, 0.00724, 0.00724, 0.00724, 0.00724, 0.00724, 0.00724, 0.00724, 0.00724, 0.00724, 0.00724, 0.00724, 0.00724, 0.00724, 0.00724, 0.00724, 0.00724, 0.007153333333333333, 0.007153333333333333, 0.007153333333333333, 0.007153333333333333, 0.007153333333333333, 0.007153333333333333, 0.007153333333333333, 0.007153333333333333, 0.007153333333333333, 0.007153333333333333, 0.007153333333333333, 0.007153333333333333, 0.007153333333333333, 0.007153333333333333, 0.007153333333333333, 0.007153333333333333, 0.007066666666666666, 0.007066666666666666, 0.007066666666666666, 0.007066666666666666, 0.007066666666666666, 0.007066666666666666, 0.007066666666666666, 0.007066666666666666, 0.007066666666666666, 0.007066666666666666, 0.007066666666666666, 0.007066666666666666, 0.007066666666666666, 0.007066666666666666, 0.007066666666666666, 0.007066666666666666, 0.00698, 0.00698, 0.00698, 0.00698, 0.00698, 0.00698, 0.00698, 0.00698, 0.00698, 0.00698, 0.00698, 0.00698, 0.00698, 0.00698, 0.00698, 0.00698, 0.006893333333333333, 0.006893333333333333, 0.006893333333333333, 0.006893333333333333, 0.006893333333333333, 0.006893333333333333, 0.006893333333333333, 0.006893333333333333, 0.006893333333333333, 0.006893333333333333, 0.006893333333333333, 0.006893333333333333, 0.006893333333333333, 0.006893333333333333, 0.006893333333333333, 0.006893333333333333, 0.006806666666666667, 0.006806666666666667, 0.006806666666666667, 0.006806666666666667, 0.006806666666666667, 0.006806666666666667, 0.006806666666666667, 0.006806666666666667, 0.006806666666666667, 0.006806666666666667, 0.006806666666666667, 0.006806666666666667, 0.006806666666666667, 0.006806666666666667, 0.006806666666666667, 0.006806666666666667, 0.00672, 0.00672, 0.00672, 0.00672, 0.00672, 0.00672, 0.00672, 0.00672, 0.00672, 0.00672, 0.00672, 0.00672, 0.00672, 0.00672, 0.00672, 0.00672, 0.006633333333333333, 0.006633333333333333, 0.006633333333333333, 0.006633333333333333, 0.006633333333333333, 0.006633333333333333, 0.006633333333333333, 0.006633333333333333, 0.006633333333333333, 0.006633333333333333, 0.006633333333333333, 0.006633333333333333, 0.006633333333333333, 0.006633333333333333, 0.006633333333333333, 0.006633333333333333, 0.006546666666666667, 0.006546666666666667, 0.006546666666666667, 0.006546666666666667, 0.006546666666666667, 0.006546666666666667, 0.006546666666666667, 0.006546666666666667, 0.006546666666666667, 0.006546666666666667, 0.006546666666666667, 0.006546666666666667, 0.006546666666666667, 0.006546666666666667, 0.006546666666666667, 0.006546666666666667, 0.00646, 0.00646, 0.00646, 0.00646, 0.00646, 0.00646, 0.00646, 0.00646, 0.00646, 0.00646, 0.00646, 0.00646, 0.00646, 0.00646, 0.00646, 0.00646, 0.006373333333333333, 0.006373333333333333, 0.006373333333333333, 0.006373333333333333, 0.006373333333333333, 0.006373333333333333, 0.006373333333333333, 0.006373333333333333, 0.006373333333333333, 0.006373333333333333, 0.006373333333333333, 0.006373333333333333, 0.006373333333333333, 0.006373333333333333, 0.006373333333333333, 0.006373333333333333, 0.006286666666666666, 0.006286666666666666, 0.006286666666666666, 0.006286666666666666, 0.006286666666666666, 0.006286666666666666, 0.006286666666666666, 0.006286666666666666, 0.006286666666666666, 0.006286666666666666, 0.006286666666666666, 0.006286666666666666, 0.006286666666666666, 0.006286666666666666, 0.006286666666666666, 0.006286666666666666, 0.0062, 0.0062, 0.0062, 0.0062, 0.0062, 0.0062, 0.0062, 0.0062, 0.0062, 0.0062, 0.0062, 0.0062, 0.0062, 0.0062, 0.0062, 0.0062 };
const float psiqLUTpu16negIdxIqy[16 * 16] = { 0.0, 8.666666666666665e-05, 0.0001733333333333333, 0.00026, 0.0003466666666666666, 0.00043333333333333326, 0.00052, 0.0006066666666666667, 0.0006933333333333332, 0.0007799999999999999, 0.0008666666666666665, 0.0009533333333333332, 0.00104, 0.0011266666666666667, 0.0012133333333333334, 0.0013, 0.0, 8.666666666666665e-05, 0.0001733333333333333, 0.00026, 0.0003466666666666666, 0.00043333333333333326, 0.00052, 0.0006066666666666667, 0.0006933333333333332, 0.0007799999999999999, 0.0008666666666666665, 0.0009533333333333332, 0.00104, 0.0011266666666666667, 0.0012133333333333334, 0.0013, 0.0, 8.666666666666665e-05, 0.0001733333333333333, 0.00026, 0.0003466666666666666, 0.00043333333333333326, 0.00052, 0.0006066666666666667, 0.0006933333333333332, 0.0007799999999999999, 0.0008666666666666665, 0.0009533333333333332, 0.00104, 0.0011266666666666667, 0.0012133333333333334, 0.0013, 0.0, 8.666666666666665e-05, 0.0001733333333333333, 0.00026, 0.0003466666666666666, 0.00043333333333333326, 0.00052, 0.0006066666666666667, 0.0006933333333333332, 0.0007799999999999999, 0.0008666666666666665, 0.0009533333333333332, 0.00104, 0.0011266666666666667, 0.0012133333333333334, 0.0013, 0.0, 8.666666666666665e-05, 0.0001733333333333333, 0.00026, 0.0003466666666666666, 0.00043333333333333326, 0.00052, 0.0006066666666666667, 0.0006933333333333332, 0.0007799999999999999, 0.0008666666666666665, 0.0009533333333333332, 0.00104, 0.0011266666666666667, 0.0012133333333333334, 0.0013, 0.0, 8.666666666666665e-05, 0.0001733333333333333, 0.00026, 0.0003466666666666666, 0.00043333333333333326, 0.00052, 0.0006066666666666667, 0.0006933333333333332, 0.0007799999999999999, 0.0008666666666666665, 0.0009533333333333332, 0.00104, 0.0011266666666666667, 0.0012133333333333334, 0.0013, 0.0, 8.666666666666665e-05, 0.0001733333333333333, 0.00026, 0.0003466666666666666, 0.00043333333333333326, 0.00052, 0.0006066666666666667, 0.0006933333333333332, 0.0007799999999999999, 0.0008666666666666665, 0.0009533333333333332, 0.00104, 0.0011266666666666667, 0.0012133333333333334, 0.0013, 0.0, 8.666666666666665e-05, 0.0001733333333333333, 0.00026, 0.0003466666666666666, 0.00043333333333333326, 0.00052, 0.0006066666666666667, 0.0006933333333333332, 0.0007799999999999999, 0.0008666666666666665, 0.0009533333333333332, 0.00104, 0.0011266666666666667, 0.0012133333333333334, 0.0013, 0.0, 8.666666666666665e-05, 0.0001733333333333333, 0.00026, 0.0003466666666666666, 0.00043333333333333326, 0.00052, 0.0006066666666666667, 0.0006933333333333332, 0.0007799999999999999, 0.0008666666666666665, 0.0009533333333333332, 0.00104, 0.0011266666666666667, 0.0012133333333333334, 0.0013, 0.0, 8.666666666666665e-05, 0.0001733333333333333, 0.00026, 0.0003466666666666666, 0.00043333333333333326, 0.00052, 0.0006066666666666667, 0.0006933333333333332, 0.0007799999999999999, 0.0008666666666666665, 0.0009533333333333332, 0.00104, 0.0011266666666666667, 0.0012133333333333334, 0.0013, 0.0, 8.666666666666665e-05, 0.0001733333333333333, 0.00026, 0.0003466666666666666, 0.00043333333333333326, 0.00052, 0.0006066666666666667, 0.0006933333333333332, 0.0007799999999999999, 0.0008666666666666665, 0.0009533333333333332, 0.00104, 0.0011266666666666667, 0.0012133333333333334, 0.0013, 0.0, 8.666666666666665e-05, 0.0001733333333333333, 0.00026, 0.0003466666666666666, 0.00043333333333333326, 0.00052, 0.0006066666666666667, 0.0006933333333333332, 0.0007799999999999999, 0.0008666666666666665, 0.0009533333333333332, 0.00104, 0.0011266666666666667, 0.0012133333333333334, 0.0013, 0.0, 8.666666666666665e-05, 0.0001733333333333333, 0.00026, 0.0003466666666666666, 0.00043333333333333326, 0.00052, 0.0006066666666666667, 0.0006933333333333332, 0.0007799999999999999, 0.0008666666666666665, 0.0009533333333333332, 0.00104, 0.0011266666666666667, 0.0012133333333333334, 0.0013, 0.0, 8.666666666666665e-05, 0.0001733333333333333, 0.00026, 0.0003466666666666666, 0.00043333333333333326, 0.00052, 0.0006066666666666667, 0.0006933333333333332, 0.0007799999999999999, 0.0008666666666666665, 0.0009533333333333332, 0.00104, 0.0011266666666666667, 0.0012133333333333334, 0.0013, 0.0, 8.666666666666665e-05, 0.0001733333333333333, 0.00026, 0.0003466666666666666, 0.00043333333333333326, 0.00052, 0.0006066666666666667, 0.0006933333333333332, 0.0007799999999999999, 0.0008666666666666665, 0.0009533333333333332, 0.00104, 0.0011266666666666667, 0.0012133333333333334, 0.0013, 0.0, 8.666666666666665e-05, 0.0001733333333333333, 0.00026, 0.0003466666666666666, 0.00043333333333333326, 0.00052, 0.0006066666666666667, 0.0006933333333333332, 0.0007799999999999999, 0.0008666666666666665, 0.0009533333333333332, 0.00104, 0.0011266666666666667, 0.0012133333333333334, 0.0013 };

const float LdLUTpu16negId[16] = { MATLAB_PARA_Ld,MATLAB_PARA_Ld,MATLAB_PARA_Ld,MATLAB_PARA_Ld,MATLAB_PARA_Ld,MATLAB_PARA_Ld,MATLAB_PARA_Ld,MATLAB_PARA_Ld,MATLAB_PARA_Ld,MATLAB_PARA_Ld,MATLAB_PARA_Ld,MATLAB_PARA_Ld,MATLAB_PARA_Ld,MATLAB_PARA_Ld,MATLAB_PARA_Ld,MATLAB_PARA_Ld, };
const float LqLUTpu16Iq[16] = { MATLAB_PARA_Lq,MATLAB_PARA_Lq,MATLAB_PARA_Lq,MATLAB_PARA_Lq,MATLAB_PARA_Lq,MATLAB_PARA_Lq,MATLAB_PARA_Lq,MATLAB_PARA_Lq,MATLAB_PARA_Lq,MATLAB_PARA_Lq,MATLAB_PARA_Lq,MATLAB_PARA_Lq,MATLAB_PARA_Lq,MATLAB_PARA_Lq,MATLAB_PARA_Lq,MATLAB_PARA_Lq, };
