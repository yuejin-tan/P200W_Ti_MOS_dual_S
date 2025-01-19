function tyjEval(tGEL_Str) {
    print("eval:\t" + tGEL_Str);
    eval(tGEL_Str);
}

function OECA_2OrdFiltSet(name, f_filter, ksai) {

    var f_sample = 30e3;

    var T_sample = 1 / f_sample;
    var omega_N = 2 * Math.PI * f_filter;

    var B1 = -2 * Math.exp(-ksai * omega_N * T_sample) * Math.cos(omega_N * T_sample * Math.sqrt(1 - ksai * ksai));
    var B2 = Math.exp(-2 * ksai * omega_N * T_sample);
    var A = 1 + B1 + B2;

    tyjEval(name + ".A=" + String(A) + ";");
    tyjEval(name + ".B1=" + String(B1) + ";");
    tyjEval(name + ".B2=" + String(B2) + ";");
}

// eg OECA_2OrdFiltSet("OECA_f1",200,0.7)
// eg OECA_2OrdFiltSet("OECA_f2",200,0.7)

var gRDC_Offset = 24250;

var gRDC_add = 3277;
var gRDC_testCnt = 0;

var gRDC_d_angle = 10.0
var gRDC_now_angle = -gRDC_d_angle

function OECT_UtilTry() {
    gRDC_testCnt = gRDC_testCnt + 1;
    print("cnt:\t" + String(gRDC_testCnt));
    tyjEval("OECA1.status=0;");
    tyjEval("OECA1.modeCfg=7;");
    tyjEval("thetaRDC_raw_offset_CH1=" + String(gRDC_Offset + gRDC_testCnt * gRDC_add) + ";");
    tyjEval("OECA1.status=1;");
}

function OECT_UtilTry2() {
    gRDC_now_angle = gRDC_now_angle + gRDC_d_angle;

    var aaa = Math.round(65536 * 2 + gRDC_Offset - gRDC_now_angle / 360.0 / 2.0 * 65536.0) % 65536

    print("gRDC_now_angle:\t" + String(gRDC_now_angle));
    tyjEval("OECA1.status=0;");
    tyjEval("OECA1.modeCfg=7;");
    tyjEval("thetaRDC_raw_offset_CH1=" + String(aaa) + ";");
    tyjEval("OECA1.status=1;");
}

function OECT_UtilTry3() {
    // gRDC_Offset = (gRDC_Offset + gRDC_add) % 65536;
    gRDC_testCnt = gRDC_testCnt + 1;
    print("cnt:\t" + String(gRDC_testCnt));
    tyjEval("OECA1.status=0;");
    tyjEval("OECA1.modeCfg=7;");
    tyjEval("OECA1.status=1;");
}

function OECT_UtilTry4() {
    // gRDC_Offset = (gRDC_Offset + gRDC_add) % 65536;
    gRDC_testCnt = gRDC_testCnt + 1;
    print("cnt:\t" + String(gRDC_testCnt));
    tyjEval("OECA1.status=0;");
    tyjEval("OECA1.modeCfg=14;");
    tyjEval("OECA1.status=1;");
}
