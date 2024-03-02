#include "DSP28x_Project.h"
//#include "our_encoder.h"
#include "our_ecap.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

#define NSleep_On()     GpioDataRegs.GPBSET.bit.GPIO42=1

/////////////////////////////////////////////////////////
extern unsigned int RamfuncsLoadStart;   //extern unsigned int 抹除暫存器的值
extern unsigned int RamfuncsLoadEnd;
extern unsigned int RamfuncsRunStart;
void MemCopy(unsigned int *SourceAddr, unsigned int *SourceEndAddr, unsigned int *DestAddr){
	while(SourceAddr < SourceEndAddr){
		*DestAddr++ = *SourceAddr++;
	}
	return;
}
/////////////////////////////////////////////////////////

void SetupEPwm(void);
void InitECapture(void);

long count2 = 0;
int fast2 = 0;
long count3 = 0;
int fast3 = 0;
int prd2 = 0;
int prd3 = 0;

//eqep參數初始化
/*
int DirectionQep = 0;
int LineEncoder = 1000;
int Encoder_N = 4000;
float Speed_Mr_RPM = 0;
float Speed_Mr_RPM1 = 0;
float Position_Pre = 0;
float Position_Cur= 0;
*/
//ecap參數初始化

int ecap1_count = 0;
float Ecap1_TS1 = 0;
float Ecap1_TS3 = 0;
float Ecap2_TS1 = 0;
float Ecap2_TS2 = 0;
float Ecap2_gap_A_B = 0;
float Ecap2_gap_pulse = 0;
float calc_pulse = 0;
float calc_time;
float speed = 0;
int FORWARD = 1;
int BACKWARD = -1;
int Direction_flag = 0;
int Ecap_Z_flag = 0;


void main(void){
    InitSysCtrl();

    ///////////////////////////////////////////////////////////////
    MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);
    InitFlash();
    ///////////////////////////////////////////////////////////////
    //InitGpio();

    EALLOW;                                 //關閉寫保護
    GpioCtrlRegs.GPAMUX1.all = 0xC550;
    GpioCtrlRegs.GPAMUX2.all = 0x00CC;
    GpioCtrlRegs.GPBMUX1.all = 0x0000;
	GpioCtrlRegs.GPBMUX2.all = 0x0000;
	GpioCtrlRegs.GPBDIR.bit.GPIO42 = 1;
    EDIS;                                   //開啟寫保護

    GpioDataRegs.GPADAT.all = 0x0000;
    GpioDataRegs.GPBDAT.all = 0x0000;
    /////////////////////////////////////////////////////////////////////
    InitPieCtrl();
    InitPieVectTable();  //初始化中斷向量表
    InitEPwm();
    //InitEPwm1Gpio();  //GPIO0、GPIO1使能EPWM功能
    InitEPwm2Gpio();  //GPIO2、GPIO3使能EPWM功能
    InitEPwm3Gpio();  //GPIO4、GPIO5使能EPWM功能
    InitECap1Gpio();  //GPIO11
    InitECap2Gpio();  //GPIO7
    InitECap3Gpio();  //GPIO9
    //InitEQep1Gpio();
    /////////////////////////////////////////////////////////////////////
    IER = 0x0000;
    IFR = 0x0000;

    DINT;  //禁止中斷
    EALLOW;
    PieVectTable.EPWM2_INT = &EPWM2_INT_ISR;
    PieVectTable.EPWM3_INT = &EPWM3_INT_ISR;
    PieVectTable.ECAP1_INT = &ECAP1_INT_ISR;
    PieVectTable.ECAP2_INT = &ECAP2_INT_ISR;
    PieVectTable.ECAP3_INT = &ECAP3_INT_ISR;
    EDIS;

    IER |= M_INT3;  //EPWM1~8_INT屬於CPU中斷3所以須使能    //M_INT3 = 0x0004
    IER |= M_INT4;  //ECAP1~3_INT屬於CPU中斷4所以須使能    //M_INT4 = 0x0008
    PieCtrlRegs.PIEIER3.bit.INTx2 = 1;  //INT3.2
    PieCtrlRegs.PIEIER3.bit.INTx3 = 1;  //INT3.3
    PieCtrlRegs.PIEIER4.bit.INTx1 = 1;  //INT4.1
    PieCtrlRegs.PIEIER4.bit.INTx2 = 1;  //INT4.2
    PieCtrlRegs.PIEIER4.bit.INTx3 = 1;  //INT4.3
    EINT;  //重新開啟中斷

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;
    SetupEPwm();
    InitECapture();
    //QEP_pos_speed_get_init();
    NSleep_On();

    while(1){
    }
}

void SetupEPwm(void){
	EALLOW;
	SysCtrlRegs.PCLKCR1.bit.EPWM2ENCLK = 1;
	SysCtrlRegs.PCLKCR1.bit.EPWM3ENCLK = 1;
	SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
/*
//////1
	EPwm1Regs.TBCTL.bit.CLKDIV = 3;  //除以8
	EPwm1Regs.TBCTL.bit.HSPCLKDIV = 5;  //除以10(80Mhz/80)
	EPwm1Regs.TBCTL.bit.CTRMODE = 2;  //增減模式

	EPwm1Regs.TBSTS.all = 0;
	EPwm1Regs.TBPHS.half.TBPHS = 0;
	EPwm1Regs.TBCTR = 0;
	EPwm1Regs.TBPRD = 10000;  //週期(四個波)為2PRD*(1/1*10^6) EX.PRD=50000為40HZ(12RPM)
	                          //週期(四個波)為2PRD*(1/1*10^6) EX.PRD=10000為200HZ(60RPM)
	EPwm1Regs.CMPCTL.bit.SHDWAMODE = 1;
	EPwm1Regs.CMPCTL.bit.SHDWBMODE = 1;
	EPwm1Regs.CMPA.half.CMPA = 5000;  //只使用CMPA計時器，占空比50%
	//EPwm1Regs.CMPB = 50000;

	EPwm1Regs.AQCTLA.bit.ZRO = 2;
	EPwm1Regs.AQCTLA.bit.CAU = 1;
	EPwm1Regs.AQCTLA.bit.PRD = 1;
	EPwm1Regs.AQCTLA.bit.CAD = 2;

	EPwm1Regs.AQCTLB.bit.ZRO = 2;
	EPwm1Regs.AQCTLB.bit.CAU = 1;
	EPwm1Regs.AQCTLB.bit.PRD = 1;
	EPwm1Regs.AQCTLB.bit.CAD = 2;

	EPwm1Regs.DBCTL.bit.IN_MODE = 2;   //IN_MODE=2 A為上升延時B為下降延時
	EPwm1Regs.DBCTL.bit.POLSEL = 2;    //POLSEL=2 B反轉
	EPwm1Regs.DBCTL.bit.OUT_MODE = 3;  //OUT_MODE=3 使能上升、下降延時
	EPwm1Regs.DBRED = 1;               //死區1微秒
	EPwm1Regs.DBFED = 1;

	EPwm1Regs.ETSEL.bit.INTEN = 1;    //中斷事件使能
	EPwm1Regs.ETSEL.bit.INTSEL = 2;   //ctr=prd
	EPwm1Regs.ETPS.bit.INTPRD = 1;
*/
//////2
	EPwm2Regs.TBCTL.bit.CLKDIV = 3;  //除以8
	EPwm2Regs.TBCTL.bit.HSPCLKDIV = 5;  //除以10(80Mhz/80)
	EPwm2Regs.TBCTL.bit.CTRMODE = 2;  //增減模式

	EPwm2Regs.TBSTS.all = 0;
	EPwm2Regs.TBPHS.half.TBPHS = 0;
	EPwm2Regs.TBCTR = 0;
	EPwm2Regs.TBPRD = 10000;  //週期(四個波)為2PRD*(1/1*10^6) EX.PRD=50000為40HZ(12RPM)
							  //週期(四個波)為2PRD*(1/1*10^6) EX.PRD=10000為200HZ(60RPM)
	EPwm2Regs.CMPCTL.bit.SHDWAMODE = 1;
	EPwm2Regs.CMPCTL.bit.SHDWBMODE = 1;
	EPwm2Regs.CMPA.half.CMPA = 5000;  //只使用CMPA計時器，占空比50%
	//EPwm2Regs.CMPB = 50000;

	EPwm2Regs.AQCTLA.bit.ZRO = 2;
	EPwm2Regs.AQCTLA.bit.CAU = 1;
	EPwm2Regs.AQCTLA.bit.PRD = 1;
	EPwm2Regs.AQCTLA.bit.CAD = 2;

	EPwm2Regs.AQCTLB.bit.ZRO = 2;
	EPwm2Regs.AQCTLB.bit.CAU = 1;
	EPwm2Regs.AQCTLB.bit.PRD = 1;
	EPwm2Regs.AQCTLB.bit.CAD = 2;

	EPwm2Regs.DBCTL.bit.IN_MODE = 2;   //IN_MODE=2 A為上升延時B為下降延時
	EPwm2Regs.DBCTL.bit.POLSEL = 2;    //POLSEL=2 B反轉
	EPwm2Regs.DBCTL.bit.OUT_MODE = 3;  //OUT_MODE=3 使能上升、下降延時
	EPwm2Regs.DBRED = 1;               //死區1微秒
	EPwm2Regs.DBFED = 1;

	EPwm2Regs.ETSEL.bit.INTEN = 1;    //中斷事件使能
	EPwm2Regs.ETSEL.bit.INTSEL = 5;   //ctr=prd
	EPwm2Regs.ETPS.bit.INTPRD = 1;

//////3
	EPwm3Regs.TBCTL.bit.CLKDIV = 3;  //除以8
	EPwm3Regs.TBCTL.bit.HSPCLKDIV = 5;  //除以10(80Mhz/80)
	EPwm3Regs.TBCTL.bit.CTRMODE = 2;  //增減模式

	EPwm3Regs.TBSTS.all = 0;
	EPwm3Regs.TBPHS.half.TBPHS = 0;
	EPwm3Regs.TBCTR = 0;
	EPwm3Regs.TBPRD = 10000;  //週期(四個波)為2PRD*(1/1*10^6) EX.PRD=50000為40HZ(12RPM)
							  //週期(四個波)為2PRD*(1/1*10^6) EX.PRD=10000為200HZ(60RPM)
	EPwm3Regs.CMPCTL.bit.SHDWAMODE = 1;
	EPwm3Regs.CMPCTL.bit.SHDWBMODE = 1;
	EPwm3Regs.CMPA.half.CMPA = 5000;  //只使用CMPA計時器，占空比50%
	//EPwm2Regs.CMPB = 50000;

	EPwm3Regs.AQCTLA.bit.ZRO = 2;
	EPwm3Regs.AQCTLA.bit.CAU = 2;
	EPwm3Regs.AQCTLA.bit.PRD = 1;
	EPwm3Regs.AQCTLA.bit.CAD = 1;

	EPwm3Regs.AQCTLB.bit.ZRO = 2;
	EPwm3Regs.AQCTLB.bit.CAU = 2;
	EPwm3Regs.AQCTLB.bit.PRD = 1;
	EPwm3Regs.AQCTLB.bit.CAD = 1;

	EPwm3Regs.DBCTL.bit.IN_MODE = 2;   //IN_MODE=2 A為上升延時B為下降延時
	EPwm3Regs.DBCTL.bit.POLSEL = 2;    //POLSEL=2 B反轉
	EPwm3Regs.DBCTL.bit.OUT_MODE = 3;  //OUT_MODE=3 使能上升、下降延時
	EPwm3Regs.DBRED = 1;               //死區1微秒
	EPwm3Regs.DBFED = 1;

	EPwm3Regs.ETSEL.bit.INTEN = 1;    //中斷事件使能
	EPwm3Regs.ETSEL.bit.INTSEL = 5;   //ctr=prd
	EPwm3Regs.ETPS.bit.INTPRD = 1;

	SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;
	EDIS;
	prd2 = EPwm2Regs.TBPRD;
	prd3 = EPwm3Regs.TBPRD;
	EPwm2Regs.ETCLR.bit.INT = 1;
	EPwm3Regs.ETCLR.bit.INT = 1;
}
/*
interrupt void EPWM_INT2(void){
	//EPwm2Regs.ETFLG.bit.INT = 0;
	EPwm2Regs.ETCLR.bit.INT = 1;
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
	//EPwm2Regs.ETCLR.bit.INT = 1;
	count2++;
	asm ("      ESTOP0");


	if(count2 % 50 == 0 && fast2 == 0){
		EPwm2Regs.TBPRD = prd2 * 2;
		EPwm2Regs.CMPA.half.CMPA = (1 * EPwm2Regs.TBPRD) / 2;
		EPwm2Regs.ETCLR.bit.INT = 1;
		PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
		fast2++;
	}
	else if(count2 % 50 == 0 && fast2 == 1){
		EPwm2Regs.TBPRD = prd2;
		EPwm2Regs.CMPA.half.CMPA = (1 * EPwm2Regs.TBPRD) / 2;
		EPwm2Regs.ETCLR.bit.INT = 1;
		PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
		fast2--;
	}
	else{
		EPwm2Regs.ETCLR.bit.INT = 1;
		PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
	}

}
*/
/*
interrupt void EPWM_INT3(void){
	EPwm3Regs.ETFLG.bit.INT = 0;
	EPwm3Regs.ETCLR.bit.INT = 1;
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
	count3++;
	asm ("      ESTOP0");


	if(count3 % 50 == 0 && fast3 == 0){
		EPwm3Regs.TBPRD = prd3 * 2;
		EPwm3Regs.CMPA.half.CMPA = (1 * EPwm3Regs.TBPRD) / 2;
		EPwm3Regs.ETCLR.bit.INT = 1;
		PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
		fast3++;
	}
	else if(count3 % 50 == 0 && fast3 == 1){
		EPwm3Regs.TBPRD = prd3;
		EPwm3Regs.CMPA.half.CMPA = (1 * EPwm3Regs.TBPRD) / 2;
		EPwm3Regs.ETCLR.bit.INT = 1;
		PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
		fast3--;
	}
	else{
		EPwm3Regs.ETCLR.bit.INT = 1;
		PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
	}

}
*/
