/*
 * topCfg_local.h
 *
 *  Created on: 2023年10月13日
 *      Author: t
 */

#ifndef TOPCFG_LOCAL_H_
#define TOPCFG_LOCAL_H_

 // 数组大小 LS2-5: 2k*4; D0-1: 2k*2; GS0-11: 4k*12;
#define RAMBUFFSIZE (15ul * 4ul * 1024ul / 2ul)

// 实验DSP内部内存
#define CDB_BUFF_ADDR ((void*)0x9000ul)

// CDB 端口
#define CAN_CDB_BASE CANB_BASE

#endif /* TOPCFG_LOCAL_H_ */
