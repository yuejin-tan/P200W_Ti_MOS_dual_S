-------------------
--ET4100: COM4
--开始
--scd_mdbNewPort(7, 9600, 80, 1)
--转速
--rev=scd_mdbReadRegF32(1, 320)
--扭矩
--rev=scd_mdbReadRegF32(1, 322)
--结束
--scd_mdbClosePort()


--------------------
--电阻箱 ：com3
--scd_rBox_init(6)

--------------------
--scd : com10

--查看变量选定
scd_tabItemSelect("CH1_Id")
scd_tabItemSelect("CH1_Iq")

scd_tabItemSelect("targetIs")
scd_tabItemSelect("targetThetaMTPA")

scd_tabItemSelect("OECA1.theta_eComp")
scd_tabItemSelect("OECA1.theta_pll")
scd_tabItemSelect("OECA1.omegaOB")

--设定包间隔
scd_deltaPkgSet(0.7813)

--自动配置
scd_plotCfg()

--自动加入绘图
scd_plotItemSelect("CH1_Id")
scd_plotItemSelect("CH1_Iq")

scd_plotItemSelect("targetIs")
scd_plotItemSelect("targetThetaMTPA")

scd_plotItemSelect("OECA1.theta_eComp")
scd_plotItemSelect("OECA1.theta_pll")
scd_plotItemSelect("OECA1.omegaOB")

--画图位置调节
scd_YaxisSet("thetaRDC_raw", -300000, 300000)
scd_YaxisSet("targetTe", -30, 30)
scd_YaxisSet("CH1_Udc", -380, 300)
scd_YaxisSet("CH1_Idc", -250, 50)
scd_YaxisSet("CH1_Id", -355, 75)
scd_YaxisSet("targetId_CH1", -290, 140)
scd_YaxisSet("CH1_Iq", -370, 60)
scd_YaxisSet("targetIq_CH1", -140, 290)
scd_YaxisSet("CH1_Utar.d", -600, 600)
scd_YaxisSet("CH1_Utar.q", -600, 600)
scd_YaxisSet("targetUdc", -800, 500)
scd_YaxisSet("targetN", -20000, 50000)
scd_YaxisSet("targetOmegaM", -2000, 5000)
scd_YaxisSet("targetIs", -210, 90)
scd_YaxisSet("targetThetaMTPA", -4, 2)
scd_YaxisSet("OECA1.theta_pll", -3.3, 2.7)
scd_YaxisSet("OECA1.theta_eComp", -3.4, 2.6)
scd_YaxisSet("OECA1.omegaOB", -5, 15)
scd_YaxisSet("omegaMfbk", -100, 5000)
scd_YaxisSet("CH1_Iv_raw", -20000, 50000)
scd_YaxisSet("CH1_Iw_raw", -20000, 50000)
scd_YaxisSet("CH1_Idc_raw", -20000, 50000)
scd_YaxisSet("CH1_powerTempMax", -100, 1000)
scd_YaxisSet("PT100_1_2_temp[0]", -100, 1000)
scd_YaxisSet("PT100_1_2_temp[1]", -100, 1000)

--变量范围设定
scd_valRangeSet("CH1_cur_mode", 3, 0)
scd_valRangeSet("speed_mode", 15, 0)
scd_valRangeSet("torque_mode", 1, 0)
scd_valRangeSet("channel_mode", 4, 0)
scd_valRangeSet("CH1_angle_mode", 3, 0)
scd_valRangeSet("CH1_ext_fcn", 15, 0)
scd_valRangeSet("CH1_deadBand_compVal", 500, 0)
scd_valRangeSet("thetaRDC_raw_offset_CH1", 65535, 0)
scd_valRangeSet("IProtectFlg_CH1", 0, 0)
scd_valRangeSet("IProtectFlg_CH2", 0, 0)
scd_valRangeSet("SPDProtectFlg", 0, 0)
scd_valRangeSet("UdcProtectFlg_CH1", 0, 0)
scd_valRangeSet("tempProtectFlg", 0, 0)
scd_valRangeSet("targetTe", 20, -20)
scd_valRangeSet("targetIs", 100, -100)
scd_valRangeSet("targetThetaMTPA", 1, 0)
scd_valRangeSet("targetId_CH1", 200, -200)
scd_valRangeSet("targetIq_CH1", 200, -200)
scd_valRangeSet("targetThetaE_CH1", 1, 0)
scd_valRangeSet("targetUdc", 300, 30)
scd_valRangeSet("targetN", 13000, 0)
scd_valRangeSet("targetOmegaM", 1200, 0)
scd_valRangeSet("targetWaveMode", 1, 0)
scd_valRangeSet("targetWaveFreq", 5000, 0)
scd_valRangeSet("targetWaveAmp", 100, 0)
scd_valRangeSet("targetWaveOffset", 100, -100)
scd_valRangeSet("targetRampVal", 12000, -240)
scd_valRangeSet("targetRampGrad", 1000, 0)
scd_valRangeSet("CH1_Iv_raw_offset", 4095, 0)
scd_valRangeSet("CH1_Iw_raw_offset", 4095, 0)
scd_valRangeSet("CH1_Idc_raw_offset", 4095, 0)
scd_valRangeSet("ad2s1210ErrGPIO", 0, 0)
