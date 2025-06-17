/**
 * @file scd_inc.h
 * @brief SCD的头文件，主要数据结构和接口的声明
 * @author Tangent (498339337@qq.com)
 * @date 2022-04-07
 * 
 * @copyright Copyright (c) 2022 @Tangent
 */

#ifndef INC_SCD_INC_H_
#define INC_SCD_INC_H_

#include "stdint.h"

#include "scd_cfg.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define TYJ_uint16_t 0x02u
#define TYJ_int16_t 0x12u
#define TYJ_uint32_t 0x04u
#define TYJ_int32_t 0x14u
#define TYJ_float 0x24u
#define TYJ_function 0x32u
// MCU处理同对应的uint类型，上位机显示不同
#define TYJ_bin16 0x02u
#define TYJ_hex16 0x02u
#define TYJ_bin32 0x04u
#define TYJ_hex32 0x04u

   /**
 * @brief scd变量记录单元
 */
   struct TYJ_UNIT_STRUCT
   {
      /// 地址
      void *_addr;
      /// 数据类型
      int16_t _type;
   };

   /**
 * @brief scd控制结构，记录各个控制状态量
 */
   struct SCD_CTRL_STRUCT
   {
      /// 要发的数据的表
      volatile int16_t structToSendTab[SCD_SEND_TAB_SIZE];
      /// 接受缓冲区指针
      volatile unsigned char (*_recBuff)[SCD_REVBUFF_SIZE];
      /// print缓冲区
      volatile unsigned char *_printBuff;
      /// 1号协议结尾值
      volatile unsigned char *_endbyte1;
      /// DUMP协议结尾值
      volatile unsigned char *_endbyte2;
      /// PRINT协议结尾值
      volatile unsigned char *_endbyte3;
      /// 使用哪个数据记录
      const struct TYJ_UNIT_STRUCT *_unit_struct;
      /// SCO 变量buffer
      volatile float tempBuff;
      /// 总共的结构数
      volatile int16_t structNum;
      /// 双缓冲，使用哪个缓冲
      volatile int16_t bufNum;
      /// 接收到的字节数
      volatile int16_t bytesRec;
      /// 发送的字节号
      volatile int16_t byteToSend;
      /// 正在发送的帧，0表示起始标志位，总数加一表示结束标志位
      volatile int16_t structToSend;
      /// 使用的发送协议序号
      volatile int16_t sco_protocol_num;
      /// 将要使用的发送协议序号
      volatile int16_t sco_protocol_num_next;
      /// 发送的字节号,用于var协议
      volatile int16_t byteToSend2;
      /// 正在发送的帧，0表示起始标志位，总数加一表示结束标志位，用于var协议
      volatile int16_t structToSend2;
      /// 发送的字节号，用于dump协议
      volatile int16_t byteToSend3;
      /// 正在发送的帧，0表示起始标志位，用于dump协议
      volatile int16_t structToSend3;
      /// 需要dump的对象号
      volatile int16_t dumpTarget;
      /// 需要dump的数量
      volatile int16_t dumpNumCnt;
      /// 单包计数
      volatile int16_t dumpPkgCnt;
      /// 是否续包
      volatile int16_t ifContPkg;
      /// 是否正在打印
      volatile int16_t isPrintBusyFlg;
      /// 打印的字节号
      volatile int16_t byteToPrint;
      /// 帧内字节标号
      volatile int16_t byteToSend4;
   };

   /**
 * @brief scd收到1个字节，将pc主机发的、由sci外设收到的一个字节放入data即可
 */
   void SCD_Rev1Byte(struct SCD_CTRL_STRUCT *scd, uint16_t data);

   /**
 * @brief scd发送一个字节，将返回值放入sci外设发给主机
 * @return uint16_t 要发送给pc的字节
 */
   uint16_t scd_send1Byte(struct SCD_CTRL_STRUCT *scd);

   /**
 * @brief 通过scd传输dbg字符串，以\0表示结尾
 * @return uint16_t 0代表成功，1代表失败
 */
   uint16_t scd_print(struct SCD_CTRL_STRUCT *scd, const char *printDataPtr);

/**
 * @brief 声明宏，根据尾缀生成对应的初始化函数和结构体的声明
 */
#define SCD_REG_DECLEAR(TYJ_SUFFIX) \
   void scd_init##TYJ_SUFFIX(void); \
   extern struct SCD_CTRL_STRUCT scd##TYJ_SUFFIX

   /// 声明_1号scd实例
   SCD_REG_DECLEAR(_1);

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif /* INC_SCD_INC_H_ */
