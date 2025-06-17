/**
 * @file scd.c
 * @brief SCD主逻辑实现
 * @author Tangent (498339337@qq.com)
 * @version V0.2
 * @date 2022-01-04
 *
 * @copyright Copyright (c) 2022 @Tangent
 */

#include "scd_cfg.h"
#include "scd_inc.h"

#if SCD_IF_USE_HIGH_VER_TI_COMPLIER
#define MEM_MACRO ".TI.ramfunc"
#else
#define MEM_MACRO "ramfuncs"
#endif

// 为保证表现一致，不使用标准C库
/* _cz_strtoxl macro */
#define IFL_NEG 1
#define IFL_READDIGIT 2
#define IFL_OVERFLOW 4
#define IFL_UNSIGNED 8

#define NULL ((void *)0)

static const int MAX_NUMS_PER_FRAME = SCD_MAX_NUMS_PER_FRAME;

const uint32_t cz_ctype[256] = {
    32,
    65824,
    131616,
    197408,
    263200,
    328992,
    394784,
    460576,
    526368,
    592232,
    657960,
    723752,
    789544,
    855336,
    921120,
    986912,
    1052704,
    1118496,
    1184288,
    1250080,
    1315872,
    1381664,
    1447456,
    1513248,
    1579040,
    1644832,
    1710624,
    1776416,
    1842208,
    1908000,
    1973792,
    2039584,
    2105416,
    2171152,
    2236944,
    2302736,
    2368528,
    2434320,
    2500112,
    2565904,
    2631696,
    2697488,
    2763280,
    2829072,
    2894864,
    2960656,
    3026448,
    3092240,
    3158020,
    3223812,
    3289604,
    3355396,
    3421188,
    3486980,
    3552772,
    3618564,
    3684356,
    3750148,
    3815952,
    3881744,
    3947536,
    4013328,
    4079120,
    4144912,
    4210704,
    6373761,
    6439553,
    6505345,
    6571137,
    6636929,
    6702721,
    6768385,
    6834177,
    6899969,
    6965761,
    7031553,
    7097345,
    7163137,
    7228929,
    7294721,
    7360513,
    7426305,
    7492097,
    7557889,
    7623681,
    7689473,
    7755265,
    7821057,
    7886849,
    7952641,
    8018433,
    5987088,
    6052880,
    6118672,
    6184464,
    6250256,
    6316048,
    6373762,
    6439554,
    6505346,
    6571138,
    6636930,
    6702722,
    6768386,
    6834178,
    6899970,
    6965762,
    7031554,
    7097346,
    7163138,
    7228930,
    7294722,
    7360514,
    7426306,
    7492098,
    7557890,
    7623682,
    7689474,
    7755266,
    7821058,
    7886850,
    7952642,
    8018434,
    8092432,
    8158224,
    8224016,
    8289808,
    8355616,
    8421376,
    8487168,
    8552960,
    8618752,
    8684544,
    8750336,
    8816128,
    8881920,
    8947712,
    9013504,
    9079296,
    9145088,
    9210880,
    9276672,
    9342464,
    9408256,
    9474048,
    9539840,
    9605632,
    9671424,
    9737216,
    9803008,
    9868800,
    9934592,
    10000384,
    10066176,
    10131968,
    10197760,
    10263552,
    10329344,
    10395136,
    10460928,
    10526720,
    10592512,
    10658304,
    10724096,
    10789888,
    10855680,
    10921472,
    10987264,
    11053056,
    11118848,
    11184640,
    11250432,
    11316224,
    11382016,
    11447808,
    11513600,
    11579392,
    11645184,
    11710976,
    11776768,
    11842560,
    11908352,
    11974144,
    12039936,
    12105728,
    12171520,
    12237312,
    12303104,
    12368896,
    12434688,
    12500480,
    12566272,
    12632064,
    12697856,
    12763648,
    12829440,
    12895232,
    12961024,
    13026816,
    13092608,
    13158400,
    13224192,
    13289984,
    13355776,
    13421568,
    13487360,
    13553152,
    13618944,
    13684736,
    13750528,
    13816320,
    13882112,
    13947904,
    14013696,
    14079488,
    14145280,
    14211072,
    14276864,
    14342656,
    14408448,
    14474240,
    14540032,
    14605824,
    14671616,
    14737408,
    14803200,
    14868992,
    14934784,
    15000576,
    15066368,
    15132160,
    15197952,
    15263744,
    15329536,
    15395328,
    15461120,
    15526912,
    15592704,
    15658496,
    15724288,
    15790080,
    15855872,
    15921664,
    15987456,
    16053248,
    16119040,
    16184832,
    16250624,
    16316416,
    16382208,
    16448000,
    16513792,
    16579584,
    16645376,
    16711168,
    16776960,
};

#define CZ_INT32_MAX 0x7fffffffl
#define CZ_INT32_MIN ((-CZ_INT32_MAX) - 1)

#define CZ_UPPER 0x0001
#define CZ_LOWER 0x0002
#define CZ_DIGIT 0x0004
#define CZ_SPACE 0x0008 // HT LF VT FF CR SP
#define CZ_PUNCT 0x0010
#define CZ_CONTROL 0x0020
#define CZ_BLANK 0x0040
#define CZ_HEX 0x0080

#define cz_isalpha(c) (cz_ctype[(c)] & (CZ_UPPER | CZ_LOWER))
#define cz_isupper(c) (cz_ctype[(c)] & CZ_UPPER)
#define cz_islower(c) (cz_ctype[(c)] & CZ_LOWER)
#define cz_isdigit(c) (cz_ctype[(c)] & CZ_DIGIT)
#define cz_isalnum(c) (cz_ctype[(c)] & (CZ_DIGIT | CZ_UPPER | CZ_LOWER))
#define cz_ispunct(c) (cz_ctype[(c)] & CZ_PUNCT)
#define cz_isspace(c) (cz_ctype[(c)] & CZ_SPACE)
#define cz_isblank(c) (cz_ctype[(c)] & CZ_BLANK)
#define cz_iscntrl(c) (cz_ctype[(c)] & CZ_CONTROL)
#define cz_isascii(c) (((unsigned)(c)) < 128)
#define cz_isgraph(c) ((c) >= 0x21 && (c) <= 0x7e)
#define cz_isprint(c) ((c) >= 0x20 && (c) <= 0x7e)
#define cz_isxdigit(c) (cz_ctype[(c)] & (CZ_HEX | CZ_DIGIT))

#define cz_toupper(c) ((int)((cz_ctype[(c)] >> 8) & 0xffu))
#define cz_tolower(c) ((int)((cz_ctype[(c)] >> 16) & 0xffu))

/* istrtoxl */
#if SCD_IF_USE_RAM_FUNCS
#pragma CODE_SECTION(_cz_strtoxl, MEM_MACRO);
#endif
static uint32_t _cz_strtoxl(const char *nptr, const char **endptr, int ibase, int flags)
{
    const char *p;
    char c;
    uint32_t number;
    uint32_t digval;
    uint32_t maxval;
    uint32_t limit;
    uint32_t maxu32;

    if (endptr != NULL)
        *endptr = nptr;

    p = nptr;
    number = 0;

    c = *p++;
    while (cz_isspace((int)(unsigned char)c))
        c = *p++;

    if (c == '+')
        c = *p++;
    if (c == '-')
    {
        flags |= IFL_NEG;
        c = *p++;
    }

    if (c == '+')
        c = *p++;

    if (ibase < 0 || ibase == 1 || ibase > 36)
    {
        if (endptr)
            *endptr = nptr;
        return 0;
    }

    if (ibase == 0)
    {
        if (c != '0')
            ibase = 10;
        else if (*p == 'x' || *p == 'X')
            ibase = 16;
        else if (*p == 'b' || *p == 'B')
            ibase = 2;
        else
            ibase = 8;
    }

    if (ibase == 16)
    {
        if (c == '0' && (*p == 'x' || *p == 'X'))
        {
            p++;
            c = *p++;
        }
    }
    else if (ibase == 2)
    {
        if (c == '0' && (*p == 'b' || *p == 'B'))
        {
            p++;
            c = *p++;
        }
    }

    maxu32 = ~((uint32_t)0);
    maxval = maxu32 / ((uint32_t)ibase);

    for (;;)
    {
        if (cz_isdigit((int)(unsigned char)c))
            digval = c - '0';
        else if (cz_isalpha((int)(unsigned char)c))
            digval = (char)cz_toupper((unsigned char)c) - 'A' + 10;
        else
            break;

        if (digval >= (uint32_t)ibase)
            break;

        flags |= IFL_READDIGIT;

        if (number < maxval || (number == maxval &&
                                (uint32_t)digval <= maxval))
        {
            number = number * ((uint32_t)ibase) + digval;
        }
        else
        {
            flags |= IFL_OVERFLOW;
            if (endptr == NULL)
            {
                break;
            }
        }

        c = *p++;
    }

    --p;

    limit = ((uint32_t)CZ_INT32_MAX) + 1;

    if (!(flags & IFL_READDIGIT))
    {
        if (endptr)
            *endptr = nptr;
        number = 0;
    }
    else if ((flags & IFL_UNSIGNED) && (flags & IFL_NEG))
    {
        number = 0;
    }
    else if ((flags & IFL_OVERFLOW) ||
             (!(flags & IFL_UNSIGNED) &&
              (((flags & IFL_NEG) && (number > limit)) ||
               (!(flags & IFL_NEG) && (number > limit - 1)))))
    {
        if (flags & IFL_UNSIGNED)
            number = maxu32;
        else if (flags & IFL_NEG)
            number = (uint32_t)CZ_INT32_MIN;
        else
            number = (uint32_t)CZ_INT32_MAX;
    }

    if (endptr)
        *endptr = p;

    if (flags & IFL_NEG)
        number = (uint32_t)(-(int32_t)number);

    return number;
}

#if SCD_IF_USE_RAM_FUNCS
#pragma CODE_SECTION(cz_atol, MEM_MACRO);
#endif
static int32_t cz_atol(const char *nptr)
{
    return (int32_t)_cz_strtoxl(nptr, 0, 10, 0);
}

#if SCD_IF_USE_RAM_FUNCS
#pragma CODE_SECTION(_atof_pow10, MEM_MACRO);
#endif
static float _atof_pow10(float data, int exp)
{
    while (exp > 0)
    {
        data *= 10.0f;
        --exp;
    }
    while (exp < 0)
    {
        data *= 0.1f;
        ++exp;
    }
    return data;
}

#if SCD_IF_USE_RAM_FUNCS
#pragma CODE_SECTION(atof_tyj, MEM_MACRO);
#endif
static float atof_tyj(const char *chrIn)
{
    const char *nextCharPtr = NULL;
    float decPart = 0;
    float ret = 0;
    int ePart = 0;
    int sign = 1;
    while (!cz_isalnum((int)(unsigned char)(*chrIn)))
    {
        if (cz_ispunct((int)(unsigned char)(*chrIn)))
        {
            if (*chrIn == '-')
            {
                sign = -sign;
            }
            else if (*chrIn == '+')
            {
                // pass
            }
            else
            {
                //不允许的
                return 0;
            }
        }
        chrIn++;
    }
    // 找出前面的数字并分离
    int32_t intPart = _cz_strtoxl(chrIn, &nextCharPtr, 10, 0);
    // 有小数点
    if (*nextCharPtr == '.')
    {
        chrIn = nextCharPtr + 1;
        int32_t decInt = _cz_strtoxl(chrIn, &nextCharPtr, 10, 0);
        int lenth = nextCharPtr - chrIn;
        decPart = _atof_pow10((float)decInt, -lenth);
    }
    // 有e或E
    if (*nextCharPtr == 'e' || *nextCharPtr == 'E')
    {
        chrIn = nextCharPtr + 1;
        ePart = _cz_strtoxl(chrIn, &nextCharPtr, 10, 0);
    }
    if (sign == 1)
    {
        ret = intPart + decPart;
    }
    else
    {
        ret = -decPart - intPart;
    }
    return _atof_pow10(ret, ePart);
}

#if SCD_IF_USE_RAM_FUNCS
#pragma CODE_SECTION(SCD_CMD_SET, MEM_MACRO);
#endif
static void SCD_CMD_SET(struct SCD_CTRL_STRUCT *scd, volatile unsigned char *paraPtr, int16_t targetNum)
{
    const struct TYJ_UNIT_STRUCT *SCD_DATA_STRUCT_tab = scd->_unit_struct;
    if (SCD_DATA_STRUCT_tab[targetNum]._type == TYJ_float)
    {
        volatile float *xtemp = (float *)(void *)SCD_DATA_STRUCT_tab[targetNum]._addr;
        *xtemp = atof_tyj((const char *)paraPtr);
    }
    if (SCD_DATA_STRUCT_tab[targetNum]._type == TYJ_uint16_t)
    {
        volatile uint16_t *xtemp = (uint16_t *)(void *)SCD_DATA_STRUCT_tab[targetNum]._addr;
        *xtemp = cz_atol((const char *)paraPtr);
    }
    if (SCD_DATA_STRUCT_tab[targetNum]._type == TYJ_int16_t)
    {
        volatile int16_t *xtemp = (int16_t *)(void *)SCD_DATA_STRUCT_tab[targetNum]._addr;
        *xtemp = cz_atol((const char *)paraPtr);
    }
    if (SCD_DATA_STRUCT_tab[targetNum]._type == TYJ_uint32_t)
    {
        volatile uint32_t *xtemp = (uint32_t *)(void *)SCD_DATA_STRUCT_tab[targetNum]._addr;
        *xtemp = cz_atol((const char *)paraPtr);
    }
    if (SCD_DATA_STRUCT_tab[targetNum]._type == TYJ_int32_t)
    {
        volatile int32_t *xtemp = (int32_t *)(void *)SCD_DATA_STRUCT_tab[targetNum]._addr;
        *xtemp = cz_atol((const char *)paraPtr);
    }
    // 其余类型是不修改的
}

#if SCD_IF_USE_RAM_FUNCS
#pragma CODE_SECTION(SCD_CMD_CONF, MEM_MACRO);
#endif
static void SCD_CMD_CONF(struct SCD_CTRL_STRUCT *scd, volatile unsigned char *paraPtr, int16_t targetNum)
{
    // targetNum即是序号
    if (targetNum >= 0 && targetNum < SCD_SEND_TAB_SIZE)
    {
        // 变量号现在转换即可
        int varNum = cz_atol((const char *)paraPtr);
        //        if (varNum >= 0 && varNum < scd->structNum)
        {
            scd->structToSendTab[targetNum] = varNum;
        }
    }
}

#if SCD_IF_USE_RAM_FUNCS
#pragma CODE_SECTION(SCD_CMD_CHG, MEM_MACRO);
#endif
static void SCD_CMD_CHG(struct SCD_CTRL_STRUCT *scd, volatile unsigned char *paraPtr, int16_t targetNum)
{
    // 配置协议类型
    (void)targetNum;
    int p_num = cz_atol((const char *)paraPtr);
    if (p_num >= 0 && p_num < SCD_PROTOCOL_NUM)
    {
        scd->sco_protocol_num_next = p_num;
    }
}

#if SCD_IF_USE_RAM_FUNCS
#pragma CODE_SECTION(SCD_CMD_DUMP, MEM_MACRO);
#endif
static void SCD_CMD_DUMP(struct SCD_CTRL_STRUCT *scd, volatile unsigned char *paraPtr, int16_t targetNum)
{
    scd->dumpTarget = targetNum;
    int p_num = cz_atol((const char *)paraPtr);
    if (p_num <= 0)
    {
        p_num = 1;
    }
    if (p_num > SCD_MAX_DUMP_COUNT)
    {
        p_num = SCD_MAX_DUMP_COUNT;
    }
    scd->dumpNumCnt = p_num;
}

#if SCD_IF_USE_RAM_FUNCS
#pragma CODE_SECTION(SCD_CMD_CALL, MEM_MACRO);
#endif
static void SCD_CMD_CALL(struct SCD_CTRL_STRUCT *scd, volatile unsigned char *paraPtr, int16_t targetNum)
{
    uint16_t judgeType = (scd->_unit_struct[targetNum]._type) & 0xffu;
    if (judgeType == TYJ_function)
    {
        // 避免打错直接down掉，还是加入一次校验
        int para = cz_atol((const char *)paraPtr);
        typedef void (*funcPtr_t)(int32_t);
        funcPtr_t funcPtr = (funcPtr_t)(void *)scd->_unit_struct[targetNum]._addr;
        (*funcPtr)(para);
    }
}

// 支持的命令
static const char *SCD_CMD_TAB[SCD_CMD_NUM] = {"set", "conf", "chg", "dump", "call"};
static void (*SCD_CMD_FCNS[SCD_CMD_NUM])(struct SCD_CTRL_STRUCT *scd, volatile unsigned char *paraPtr,
                                         int16_t targetNum) = {SCD_CMD_SET, SCD_CMD_CONF, SCD_CMD_CHG, SCD_CMD_DUMP, SCD_CMD_CALL};

#if SCD_IF_USE_RAM_FUNCS
#pragma CODE_SECTION(SCD_cmd_handle, MEM_MACRO);
#endif
static void SCD_cmd_handle(struct SCD_CTRL_STRUCT *scd)
{
    volatile unsigned char(*recBuff)[SCD_REVBUFF_SIZE] = scd->_recBuff;
    // 开头可有特殊字符
    int16_t xstart = 0;
    while (!cz_isalnum((int)(unsigned char)(recBuff[0][xstart])))
    {
        xstart++;
        if (recBuff[0][xstart] == 0)
        {
            return;
        }
    }
    // 而后必须为字母，否则指令非法
    if (!cz_isalpha((int)(unsigned char)(recBuff[0][xstart])))
    {
        return;
    }
    // 得到指令长度
    int16_t cmdLen = 0;
    while (cz_isalpha((int)(unsigned char)(recBuff[0][cmdLen + xstart])))
    {
        cmdLen++;
    }
    int16_t cmd2exec = -1;
    for (int16_t ii = 0; ii < SCD_CMD_NUM; ii++)
    {
        // 逐一比较是否相符
        int16_t isSame = 1;
        for (int16_t jj = 0; jj < cmdLen; jj++)
        {
            if (recBuff[0][jj + xstart] != SCD_CMD_TAB[ii][jj])
            {
                isSame = 0;
            }
        }
        if (isSame)
        {
            cmd2exec = ii;
            break;
        }
    }

    int16_t ptr = cmdLen + xstart;
    // 对象号至少有一位数，且必须为正整数
    if (!cz_isdigit((int)(unsigned char)(recBuff[0][ptr])))
    {
        return;
    }
    // 对象号
    int16_t targetNum = 0;
    while (cz_isdigit((int)(unsigned char)(recBuff[0][ptr])))
    {
        targetNum = targetNum * 10 + recBuff[0][ptr] - '0';
        ptr++;
    }

    if (recBuff[0][ptr] != '#')
    {
        // 此处必须为#号
        return;
    }
    ptr++;

    if (cmd2exec >= 0 && cmd2exec < SCD_CMD_NUM && targetNum < scd->structNum && targetNum >= 0)
    {
        // 满足要求则调用相应处理函数
        (*SCD_CMD_FCNS[cmd2exec])(scd, &recBuff[0][ptr], targetNum);
    }
}

#if SCD_IF_USE_RAM_FUNCS
#pragma CODE_SECTION(SCD_Rev1Byte, MEM_MACRO);
#endif
void SCD_Rev1Byte(struct SCD_CTRL_STRUCT *scd, uint16_t data)
{
    volatile unsigned char(*recBuff)[SCD_REVBUFF_SIZE] = scd->_recBuff;
    if (data == '!')
    {
        // 结尾符，刷新缓存
        if (scd->bytesRec > 0)
        {
            // 校验收到是否相同
            uint16_t isSame = 1;
            for (int16_t ii = 0; ii < scd->bytesRec; ii++)
            {
                if (recBuff[0][ii] != recBuff[1][ii])
                {
                    isSame = 0;
                }
            }
            if (isSame)
            {
                // 相同则认为传输无误，继续处理，默认采用0号缓存处理
                // 先添加结尾符 0
                recBuff[0][scd->bytesRec] = 0;
                // 进一步处理
                SCD_cmd_handle(scd);
                // 采用0号缓存继续
                scd->bufNum = 0;
                // 破坏1号缓存内容，避免多次运行
                recBuff[1][0] = 0;
                // recBuff[1][1] = 0;
            }
            else
            {
                // 使用下个Buff继续
                // scd->bufNum = (scd->bufNum + 1) % 2;
                // 提高计算效率，使用枚举判断
                if (scd->bufNum)
                {
                    scd->bufNum = 0;
                }
                else
                {
                    scd->bufNum = 1;
                }
            }
            // 清零计数值
            scd->bytesRec = 0;
        }
    }
    else
    {
        recBuff[scd->bufNum][scd->bytesRec++] = data;
    }
}

#if SCD_IF_USE_RAM_FUNCS
#pragma CODE_SECTION(SCO_get, MEM_MACRO);
#endif
static uint16_t SCO_get(struct SCD_CTRL_STRUCT *scd)
{
    // 兼容VOFA的just float
#define SCO_STARTBYTE_NUM 0
#define SCO_ENDBYTE_NUM 4

    // 起始位和结束位定义
    const unsigned char startByte[SCO_STARTBYTE_NUM + 1] = {0x00};
    const unsigned char endByte[SCO_ENDBYTE_NUM + 1] = {0x00, 0x00, 0x80, 0x7f};

    if (scd->structToSend == 0)
    {
        // 起始位
        if (scd->byteToSend < SCO_STARTBYTE_NUM)
        {
            return startByte[scd->byteToSend++];
        }
        else
        {
            scd->byteToSend = 0;
            scd->structToSend++;
            // 起始位发送完成，递归找下一数据
            return SCO_get(scd);
        }
    }
    else if (scd->structToSendTab[scd->structToSend - 1] > scd->structNum)
    {
        // 结束位
        if (scd->byteToSend < SCO_ENDBYTE_NUM)
        {
            return endByte[scd->byteToSend++];
        }
        else
        {
            scd->byteToSend = 0;
            scd->structToSend = 0;
            // 结束位发送完成，发送0x100;
            return 0x100u;
        }
    }
    else
    {
        //数据位
        if (scd->byteToSend < (4))
        {
            if (scd->byteToSend == 0)
            {
                // 一律转为float发送
                uint16_t judgeType = (scd->_unit_struct[scd->structToSendTab[scd->structToSend - 1]]._type) & 0xffu;
                void *tempCommonPtr = (void *)scd->_unit_struct[scd->structToSendTab[scd->structToSend - 1]]._addr;
                if (judgeType == TYJ_uint16_t || judgeType == TYJ_bin16 || judgeType == TYJ_hex16)
                {
                    uint16_t *tempPtr = (uint16_t *)tempCommonPtr;
                    scd->tempBuff = *tempPtr;
                }
                else if (judgeType == TYJ_uint32_t || judgeType == TYJ_bin32 || judgeType == TYJ_hex32)
                {
                    uint32_t *tempPtr = (uint32_t *)tempCommonPtr;
                    scd->tempBuff = *tempPtr;
                }
                else if (judgeType == TYJ_float)
                {
                    float *tempPtr = (float *)tempCommonPtr;
                    scd->tempBuff = *tempPtr;
                }
                else if (judgeType == TYJ_int32_t)
                {
                    int32_t *tempPtr = (int32_t *)tempCommonPtr;
                    scd->tempBuff = *tempPtr;
                }
                else if (judgeType == TYJ_function)
                {
                    scd->tempBuff = (uint32_t)tempCommonPtr;
                }
                else
                {
                    // 匹配不上就用int16类型
                    int16_t *tempPtr = (int16_t *)tempCommonPtr;
                    scd->tempBuff = *tempPtr;
                }
            }
            uint16_t *tempPtr = (uint16_t *)(void *)&(scd->tempBuff);
            tempPtr = &tempPtr[scd->byteToSend >> 1];
            if (scd->byteToSend++ & 0x1)
            {
                return ((*tempPtr) >> 8) & 0xffu;
            }
            else
            {
                return (*tempPtr) & 0xffu;
            }
        }
        else
        {
            scd->byteToSend = 0;
            scd->structToSend++;
            // 数据位发送完成，递归找下一数据
            return SCO_get(scd);
        }
    }
}

#if SCD_IF_USE_RAM_FUNCS
#pragma CODE_SECTION(SCO_VAR, MEM_MACRO);
#endif
static uint16_t SCO_VAR(struct SCD_CTRL_STRUCT *scd)
{
#define SCO_STARTBYTE_NUM1 0
#define SCO_ENDBYTE_NUM1 4

    // 起始位和结束位定义
    const unsigned char startByte[SCO_STARTBYTE_NUM1 + 1] = {0x00};
    static uint16_t checkUint16Sum = 0;

    if (scd->structToSend2 == 0)
    {
        // 起始位
        if (scd->byteToSend2 < SCO_STARTBYTE_NUM1)
        {
            return startByte[scd->byteToSend2++];
        }
        else
        {
            scd->byteToSend2 = 0;
            scd->structToSend2++;
            // 起始位发送完成，递归找下一数据
            // 将checkSum设为初值
            checkUint16Sum = 0;
            return SCO_VAR(scd);
        }
    }
    else if (scd->structToSend2 > scd->structNum)
    {
        // 结束位
        if (scd->byteToSend2 < SCO_ENDBYTE_NUM1)
        {
            return scd->_endbyte1[scd->byteToSend2++];
        }
        else
        {
            scd->byteToSend2 = 0;
            scd->structToSend2 = 0;
            // 只发1个包
            if (scd->sco_protocol_num_next == 1)
            {
                scd->sco_protocol_num_next = 0;
            }
            // 结束位发送完成，发送0x100;
            return 0x100u;
        }
    }
    else
    {
        //数据位
        if (scd->byteToSend2 < (4))
        {
            if (scd->byteToSend2 == 0)
            {
                // 一律转为float发送
                uint16_t judgeType = (scd->_unit_struct[scd->structToSend2 - 1]._type) & 0xffu;
                void *tempCommonPtr = (void *)scd->_unit_struct[scd->structToSend2 - 1]._addr;
                if (judgeType == TYJ_uint16_t || judgeType == TYJ_bin16 || judgeType == TYJ_hex16)
                {
                    uint16_t *tempPtr = (uint16_t *)tempCommonPtr;
                    scd->tempBuff = *tempPtr;
                }
                else if (judgeType == TYJ_uint32_t || judgeType == TYJ_bin32 || judgeType == TYJ_hex32)
                {
                    uint32_t *tempPtr = (uint32_t *)tempCommonPtr;
                    scd->tempBuff = *tempPtr;
                }
                else if (judgeType == TYJ_float)
                {
                    float *tempPtr = (float *)tempCommonPtr;
                    scd->tempBuff = *tempPtr;
                }
                else if (judgeType == TYJ_int32_t)
                {
                    int32_t *tempPtr = (int32_t *)tempCommonPtr;
                    scd->tempBuff = *tempPtr;
                }
                else if (judgeType == TYJ_function)
                {
                    scd->tempBuff = (uint32_t)tempCommonPtr;
                }
                else
                {
                    // 匹配不上就用int16类型
                    int16_t *tempPtr = (int16_t *)tempCommonPtr;
                    scd->tempBuff = *tempPtr;
                }
            }
            uint16_t *tempPtr = (uint16_t *)(void *)&(scd->tempBuff);
            tempPtr = &tempPtr[scd->byteToSend2 >> 1];
            if (scd->byteToSend2++ & 0x1)
            {
                return ((*tempPtr) >> 8) & 0xffu;
            }
            else
            {
                checkUint16Sum += *tempPtr;
                return (*tempPtr) & 0xffu;
            }
        }
        else
        {
            scd->byteToSend2 = 0;
            scd->structToSend2++;
            // 数据位发送完成，递归找下一数据
            // 将checkSum放到结尾包里
            scd->_endbyte1[0] = checkUint16Sum & 0xffu;
            scd->_endbyte1[1] = (checkUint16Sum >> 8) & 0xffu;
            return SCO_VAR(scd);
        }
    }
}

#if SCD_IF_USE_RAM_FUNCS
#pragma CODE_SECTION(SCO_DEBUG, MEM_MACRO);
#endif
static uint16_t SCO_DEBUG(struct SCD_CTRL_STRUCT *scd)
{
    // debug用，只会发0
    (void)scd;

    // 若模式没改就发0
    if (scd->sco_protocol_num_next == 2)
    {
        return 0;
    }
    // 否则发送0x100，切换协议
    return 0x100u;
}

#if SCD_IF_USE_RAM_FUNCS
#pragma CODE_SECTION(SCO_DUMP, MEM_MACRO);
#endif
static uint16_t SCO_DUMP(struct SCD_CTRL_STRUCT *scd)
{
#define SCO_ENDBYTE_NUM2 8

    static uint16_t checkUint16Sum = 0;

    if (scd->structToSend3 >= scd->dumpNumCnt)
    {
        // 正常条件下的结束位
        if (scd->byteToSend3 == 0)
        {
            // 首次进入
            scd->_endbyte2[0] = 0xcdu;
            scd->_endbyte2[1] = 0xccu;
            scd->_endbyte2[2] = 0xcu;
            scd->_endbyte2[3] = 0x40u;
        }
        if (scd->byteToSend3 < SCO_ENDBYTE_NUM2)
        {
            return scd->_endbyte2[scd->byteToSend3++];
        }
        else
        {
            scd->byteToSend3 = 0;
            scd->structToSend3 = 0;
            checkUint16Sum = 0;
            // 设定包计数值为初值，这里考虑上结尾包
            scd->dumpPkgCnt = 2;
            scd->ifContPkg = 0;
            // 只发1个包
            if (scd->sco_protocol_num_next == 3)
            {
                scd->sco_protocol_num_next = 0;
            }
            // 正常结束，发送0x100;
            return 0x100u;
        }
    }
    else if (scd->dumpPkgCnt > MAX_NUMS_PER_FRAME)
    {
        // 由于超长而分帧
        if (scd->byteToSend3 == 0)
        {
            // 首次进入
            if (scd->ifContPkg)
            {
                scd->_endbyte2[0] = 0xcdu;
                scd->_endbyte2[1] = 0xccu;
                scd->_endbyte2[2] = 0x8cu;
                scd->_endbyte2[3] = 0x3fu;
            }
            else
            {
                scd->_endbyte2[0] = 0;
                scd->_endbyte2[1] = 0;
                scd->_endbyte2[2] = 0;
                scd->_endbyte2[3] = 0;
            }
        }
        if (scd->byteToSend3 < SCO_ENDBYTE_NUM2)
        {
            return scd->_endbyte2[scd->byteToSend3++];
        }
        else
        {
            scd->byteToSend3 = 0;
            checkUint16Sum = 0;
            // 设定包计数值为初值，这里考虑上结尾包
            scd->dumpPkgCnt = 2;
            scd->ifContPkg = 1;
            // 不转交控制权，继续递归
            return SCO_DUMP(scd);
        }
    }
    else
    {
        //数据位
        if (scd->byteToSend3 < (4))
        {
            if (scd->byteToSend3 == 0)
            {
                // 一律转为float发送
                uint16_t judgeType = (scd->_unit_struct[scd->dumpTarget]._type) & 0xffu;
                void *tempCommonPtr = (void *)scd->_unit_struct[scd->dumpTarget]._addr;
                if (judgeType == TYJ_uint16_t || judgeType == TYJ_bin16 || judgeType == TYJ_hex16)
                {
                    uint16_t *tempPtr = (uint16_t *)tempCommonPtr;
                    scd->tempBuff = tempPtr[scd->structToSend3];
                }
                else if (judgeType == TYJ_uint32_t || judgeType == TYJ_bin32 || judgeType == TYJ_hex32)
                {
                    uint32_t *tempPtr = (uint32_t *)tempCommonPtr;
                    scd->tempBuff = tempPtr[scd->structToSend3];
                }
                else if (judgeType == TYJ_float)
                {
                    float *tempPtr = (float *)tempCommonPtr;
                    scd->tempBuff = tempPtr[scd->structToSend3];
                }
                else if (judgeType == TYJ_int32_t)
                {
                    int32_t *tempPtr = (int32_t *)tempCommonPtr;
                    scd->tempBuff = tempPtr[scd->structToSend3];
                }
                else if (judgeType == TYJ_function)
                {
                    scd->tempBuff = (uint32_t)tempCommonPtr;
                }
                else
                {
                    // 匹配不上就用int16类型
                    int16_t *tempPtr = (int16_t *)tempCommonPtr;
                    scd->tempBuff = tempPtr[scd->structToSend3];
                }
            }
            uint16_t *tempPtr = (uint16_t *)(void *)&(scd->tempBuff);
            tempPtr = &tempPtr[scd->byteToSend3 >> 1];
            if (scd->byteToSend3++ & 0x1)
            {
                return ((*tempPtr) >> 8) & 0xffu;
            }
            else
            {
                checkUint16Sum += *tempPtr;
                return (*tempPtr) & 0xffu;
            }
        }
        else
        {
            scd->byteToSend3 = 0;
            scd->structToSend3++;
            scd->dumpPkgCnt++;
            // 数据位发送完成，递归找下一数据
            // 将checkSum放到结尾包里
            scd->_endbyte2[4] = checkUint16Sum & 0xffu;
            scd->_endbyte2[5] = (checkUint16Sum >> 8) & 0xffu;
            return SCO_DUMP(scd);
        }
    }
}

#if SCD_IF_USE_RAM_FUNCS
#pragma CODE_SECTION(SCO_PRINT, MEM_MACRO);
#endif
static uint16_t SCO_PRINT(struct SCD_CTRL_STRUCT *scd)
{
#define SCO_ENDBYTE_NUM3 2

    // 用判断代替求余
    if (scd->byteToSend4 == 4)
    {
        scd->byteToSend4 = 0;
    }
    if (scd->byteToSend4 == 0)
    {
        // 规划帧
        if (scd->byteToPrint < scd->isPrintBusyFlg)
        {
            scd->_endbyte3[0] = scd->_printBuff[scd->byteToPrint++];
        }
        else
        {
            // 发完了
            scd->byteToPrint = 0;
            scd->isPrintBusyFlg = 0;
            return 0x100u;
        }
        if (scd->byteToPrint < scd->isPrintBusyFlg)
        {
            scd->_endbyte3[1] = scd->_printBuff[scd->byteToPrint++];
        }
        else
        {
            // 第二字节，没东西但还是要发一帧
            scd->_endbyte3[1] = 0;
        }
        return scd->_endbyte3[scd->byteToSend4++];
    }
    return scd->_endbyte3[scd->byteToSend4++];
}

static uint16_t (*SCD_PROTOCOL_FCNS[SCD_PROTOCOL_NUM])(struct SCD_CTRL_STRUCT *scd) = {SCO_get,
                                                                                       SCO_VAR,
                                                                                       SCO_DEBUG,
                                                                                       SCO_DUMP,
                                                                                       SCO_PRINT};

#if SCD_IF_USE_RAM_FUNCS
#pragma CODE_SECTION(scd_send1Byte, MEM_MACRO);
#endif
uint16_t scd_send1Byte(struct SCD_CTRL_STRUCT *scd)
{
    uint16_t ret;
    ret = SCD_PROTOCOL_FCNS[scd->sco_protocol_num](scd);
    if (ret & (((uint16_t)0xffu) << 8))
    {
        // 到结尾了,换协议
        if (scd->isPrintBusyFlg)
        {
            // 确保print优先级最高，不会被chg命令覆盖
            scd->sco_protocol_num = 4;
        }
        else
        {
            scd->sco_protocol_num = scd->sco_protocol_num_next;
        }
        ret = SCD_PROTOCOL_FCNS[scd->sco_protocol_num](scd);
    }
    return ret & 0xffu;
}

#if SCD_IF_USE_RAM_FUNCS
#pragma CODE_SECTION(scd_print, MEM_MACRO);
#endif
uint16_t scd_print(struct SCD_CTRL_STRUCT *scd, const char *printDataPtr)
{
    if (scd->isPrintBusyFlg)
    {
        return 1;
    }
    int chrCnt = 0;
    while (printDataPtr[chrCnt] != 0)
    {
        scd->_printBuff[chrCnt] = printDataPtr[chrCnt];
        chrCnt++;
        if (chrCnt == SCD_PRINTBUFF_SIZE)
        {
            break;
        }
    }
    scd->isPrintBusyFlg = chrCnt;

    return 0;
}
