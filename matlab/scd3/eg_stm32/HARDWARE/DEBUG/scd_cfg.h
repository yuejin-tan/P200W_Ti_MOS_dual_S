/**
 * @file scd_cfg.h
 * @brief SCD的配置文件，无需手动包含，修改宏值可修改具体配置
 * @author Tangent (498339337@qq.com)
 * @date 2022-04-07
 * 
 * @copyright Copyright (c) 2022 @Tangent
 */

#ifndef INC_SCD_CFG_H_
#define INC_SCD_CFG_H_

#include "stdint.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define SCD_VERSION "V0.2.2"

/// 接受字符串命令的缓冲区大小，现在最大39字符
#define SCD_REVBUFF_SIZE 40
/// 发送变量的序号列表，现在1帧最多发送16个变量(取决于上位机和MCU软件实现)
#define SCD_SEND_TAB_SIZE 20
/// 支持的协议数量
#define SCD_PROTOCOL_NUM 5
/// 支持的字符串命令数量
#define SCD_CMD_NUM 5
/// 每帧最大数据长度
#define SCD_MAX_NUMS_PER_FRAME 1001
/// 最大DUMP长度
#define SCD_MAX_DUMP_COUNT 32000
/// print函数的缓冲区大小，现在最大39字符
#define SCD_PRINTBUFF_SIZE 40

/// 是否加入针对TI DSP的ramfuncs声明
#define SCD_IF_USE_RAM_FUNCS 0
#define SCD_IF_USE_HIGH_VER_TI_COMPLIER 1

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif /* INC_SCD_CFG_H_ */
