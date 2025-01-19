/*
 * bsp_runtime2.c
 *
 *  Created on: 2023年10月12日
 *      Author: t
 */

#include "bsp_runtime2.h"
#include "cdb_cpu2.h"
#include "bsp_inline2.h"

#include "proj2.h"

#include "topCfg.h"

#pragma CODE_SECTION(infLoopMain2, "ramfuncs2");
void infLoopMain2(void)
{
    while (1)
    {
        // 主处理函数
        mainLoopProcess_cpu2();

        if (bsp_tim1_polling_OF())
        {
            // 1.5ms间隔已到
            // 清空标志位
            bsp_tim1_clearFlg_OF();

#ifdef _SCD_ENABLE
            cdb_prd_call(&cdb1, CDB_BUFF_ADDR);
#endif
        }

    }

}


