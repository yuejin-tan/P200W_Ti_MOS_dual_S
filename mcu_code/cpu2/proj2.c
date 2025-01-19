/*
 * proj2.c
 *
 *  Created on: 2023年10月13日
 *      Author: t
 */

#include "proj2.h"
#include "F28x_Project.h"

#include "cpu2funcs.h"
#include "bsp_inline2.h"
#include "algo_code_config.h"

#pragma CODE_SECTION(ctrl_init2, "ramfuncs2");
void ctrl_init2()
{

}

#pragma CODE_SECTION(mainLoopProcess_cpu2, "ramfuncs2");
void  mainLoopProcess_cpu2()
{
    if (bsp_tim0_polling_OF())
    {
        // tim 0 100ms间隔已到
        // 清空标志位
        bsp_tim0_clearFlg_OF();

        bsp_LED_D10_BLUE_CTRL(2);
    }
}
