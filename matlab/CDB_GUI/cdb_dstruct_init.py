# 初始化库
import struct
import time
import os

from canlib import canlib, Frame

# 枚举CAN端口
num_channels = canlib.getNumberOfChannels()
print(f"Found {num_channels} channels")
for ch in range(num_channels):
    chd = canlib.ChannelData(ch)
    print(f"{ch}. {chd.channel_name} ({chd.card_upc_no} / {chd.card_serial_no})")
# 比特率
cdbBitrate = canlib.Bitrate.BITRATE_1M

# 建立变量字典
varDict = {}
baseLoc = os.path.dirname(os.path.abspath(__file__))
mapfile_dir = baseLoc+"\\..\\..\\build_release"
mapfile_name = [file for file in os.listdir(
    mapfile_dir) if file.endswith(".map")][0]
tiMapFileLoc = mapfile_dir + "\\" + mapfile_name
with open(tiMapFileLoc, 'r') as f:
    mapFileTxt = f.read()
startMapChaStr = """GLOBAL SYMBOLS: SORTED BY Symbol Address"""
mapFileTxt = mapFileTxt[mapFileTxt.find(startMapChaStr)+len(startMapChaStr):-1]
mapFileTxtLines = mapFileTxt.split('\n')
# 扔掉前面4行和后面3行
mapFileTxtLines = mapFileTxtLines[4:-2]
# 创建变量表字典
for metaTxt in mapFileTxtLines:
    metaTxtList = metaTxt.split()
    if (not metaTxtList[2].startswith("_")):
        continue
    if (metaTxtList[2][1:].startswith("_")):
        continue
    metaTxtList[2] = metaTxtList[2][1:]
    varAddr = int(metaTxtList[1], 16)
    if (0xc000 <= varAddr <= 0x1C000 or 0x3F800 <= varAddr <= 0x40000):
        # 筛选掉显然不行的地址范围
        varDict[metaTxtList[2]] = varAddr
    # print(metaTxtList[2], hex(varAddr))

# 人工标注特殊类型的变量
varTypeDict = {}
varTypeDict["RDV_err_check"] = "function"
varTypeDict["adcOffset_init"] = "function"
varTypeDict["scd_Udc_set"] = "function"
varTypeDict["sram_init"] = "function"
varTypeDict["ad2s1210_ClaAccModeEn"] = "function"

# 结构体信息
struct_info_dict = {
    "PIctrl_struct": (("kp", 0, "float"),
                      ("ki", 2, "float"),
                      ("kb", 4, "float"),
                      ("max", 6, "float"),
                      ("min", 8, "float"),
                      ("integral", 10, "float"),
                      ("ans", 12, "float"),
                      ),
    "Trans_struct": (("U", 0, "float"),
                     ("V", 2, "float"),
                     ("W", 4, "float"),
                     ("al", 6, "float"),
                     ("be", 8, "float"),
                     ("d", 10, "float"),
                     ("q", 12, "float"),
                     ("abdq0", 14, "float"),
                     ),
    "LPF_Ord1_2_struct": (("A", 0, "float"),
                          ("B1", 2, "float"),
                          ("B2", 4, "float"),
                          ("C1", 6, "float"),
                          ("C2", 8, "float"),
                          ),
    "protect_struct": (("thd_l", 0, "float"),
                       ("thd_h", 2, "float"),
                       ("intg_base", 4, "float"),
                       ("intg_max", 6, "float"),
                       ("intg_val", 8, "float"),
                       ("follow_Dmax", 10, "float"),
                       ("follow_intg_max", 12, "float"),
                       ("follow_intg_val", 14, "float"),
                       ),
    "cdb_struct": (("trigSta", 0, "int16_t"),
                   ),
    "ADRC_struct": (("para_gamma", 0, "float"),
                    ("para_cu1", 2, "float"),
                    ("para_cu2", 4, "float"),
                    ("para_beta1", 6, "float"),
                    ("para_beta2", 8, "float"),
                    ("para_TeMax", 10, "float"),
                    ("para_TeMin", 12, "float"),
                    ("para_SMCk", 14, "float"),
                    ("x1", 16, "float"),
                    ("e_u", 18, "float"),
                    ("e1", 20, "float"),
                    ("z1", 22, "float"),
                    ("z2", 24, "float"),
                    ("out_pi", 26, "float"),
                    ("out_smc", 28, "float"),
                    ("out_eso", 30, "float"),
                    ("Te", 32, "float"),
                    ("u", 34, "float"),
                    ("pi_intg", 36, "float"),
                    ("b0", 38, "float"),
                    ("b0_inv", 40, "float"),
                    ("b0_k", 42, "float"),
                    ),
    "LMSanf_struct": (("W[0]", 0, "float"),
                      ("W[1]", 2, "float"),
                      ("W[2]", 4, "float"),
                      ("W[3]", 6, "float"),
                      ("W[4]", 8, "float"),
                      ("W[5]", 10, "float"),
                      ("mu", 12, "float"),
                      ),
}


# 通过头文件获取特殊类型的变量
headerFileLocTab_fromBase = [
    "\\..\\..\\mcu_code\\cpu1\\proj.h",
]
for headerFileLocX in headerFileLocTab_fromBase:
    with open(baseLoc + headerFileLocX, 'r', encoding='utf-8') as headFile:
        headFileTxtLines = headFile.readlines()
    for lineTxtX in headFileTxtLines:
        if (lineTxtX.startswith("extern")):
            words = lineTxtX.split(" ")
            # 第一类，结构体
            if (words[1] == "struct" and len(words) >= 4):
                # 加入特殊变量表
                structInstName = words[3].split(";")[0]
                varTypeDict[structInstName] = words[2]
                if (words[2] in struct_info_dict and structInstName in varDict):
                    # 有人工标记的，补充加入其成员信息
                    for infox in struct_info_dict[words[2]]:
                        fullName = structInstName+"."+infox[0]
                        varDict[fullName] = varDict[structInstName]+infox[1]
                        varTypeDict[fullName] = infox[2]
                continue
            # 第二类，其他变量
            if (len(words) >= 3):
                varTypeDict[words[2].split(";")[0]] = words[1]
                continue

# 人工补充位于CPU2的特殊变量
# cdb1
for infox in struct_info_dict["cdb_struct"]:
    fullName = "cdb1."+infox[0]
    varDict[fullName] = varDict["cdb1"]+infox[1]
    varTypeDict[fullName] = infox[2]

# print(varTypeDict)
# print(varDict)
