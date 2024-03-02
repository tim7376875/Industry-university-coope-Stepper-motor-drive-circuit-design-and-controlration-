#include "DSP28x_Project.h"
#include "our_encoder.h"
//#include "our_ecap.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

/////////////////////////////////////////////////////////
extern unsigned int RamfuncsLoadStart;
extern unsigned int RamfuncsRunStart;
void MemCopy(unsigned int *SourceAddr, unsigned int *SourceEndAddr, unsigned int *DestAddr){
	while(SourceAddr < SourceEndAddr){
		*DestAddr++ = *SourceAddr++;
	}
	return;
}
/////////////////////////////////////////////////////////

#define A1_On()     GpioDataRegs.GPASET.bit.GPIO0=1
#define B1_On()     GpioDataRegs.GPASET.bit.GPIO2=1
#define A2_On()     GpioDataRegs.GPASET.bit.GPIO1=1
#define B2_On()     GpioDataRegs.GPASET.bit.GPIO3=1
#define A1_Off()    GpioDataRegs.GPACLEAR.bit.GPIO0=1
#define B1_Off()    GpioDataRegs.GPACLEAR.bit.GPIO2=1
#define A2_Off()    GpioDataRegs.GPACLEAR.bit.GPIO1=1
#define B2_Off()    GpioDataRegs.GPACLEAR.bit.GPIO3=1

#define A3_On()     GpioDataRegs.GPASET.bit.GPIO4=1
#define B3_On()     GpioDataRegs.GPASET.bit.GPIO6=1
#define A4_On()     GpioDataRegs.GPASET.bit.GPIO5=1
#define B4_On()     GpioDataRegs.GPASET.bit.GPIO7=1
#define A3_Off()    GpioDataRegs.GPACLEAR.bit.GPIO4=1
#define B3_Off()    GpioDataRegs.GPACLEAR.bit.GPIO6=1
#define A4_Off()    GpioDataRegs.GPACLEAR.bit.GPIO5=1
#define B4_Off()    GpioDataRegs.GPACLEAR.bit.GPIO7=1

volatile long dt;  //用在馬達delay time  100000 = 0.1s
				   //                   5000 = 0.05s

static const unsigned long act_stepmotor[] = { 2, 200, 5000, 0};	//馬達動作編號、步數、delay time

int i = 0, j = 0, k = 0, pi = 0,countf = 0, act = 0, step;

void SetupEPwm(void);					//EPWM設置
void InitECapture(void);				//ECAP設置
interrupt void cpu_timer0_isr(void);	//PID中斷、EQEP計算瞬時轉速
void pid(void);							//PID中斷
void first_pid(void);
void first_rotation(void);
//eqep參數設置

int DirectionQep = 0;	//馬達轉向
int LineEncoder = 1000;
int Encoder_N = 4000;	//四倍頻率捕捉
float Speed_Mr_RPM = 0; //馬達轉速
float Position_Pre = 0; //馬達上一個位置
float Position_Cur= 0;  //馬達當前位置
unsigned int rotation_step;
int first_flag = 0;
int pid_signal = 0;
int thetashift = 0;
int thetastep = 0;
int pid_step = 0;

//ecap參數設置

int ecap1_count = 0;
float Ecap1_TS1 = 0;
float Ecap1_TS3 = 0;
float Ecap2_TS1 = 0;
float Ecap2_TS2 = 0;
float Ecap2_gap_A_B = 0;
float Ecap2_gap_pulse = 0;
float calc_pulse = 0;
float calc_time = 0;
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
    EALLOW;
    GpioCtrlRegs.GPAMUX1.all = 0x0000;		//GPIOA、B初始化
    GpioCtrlRegs.GPAMUX2.all = 0x0000;
    GpioCtrlRegs.GPBMUX1.all = 0x0000;
	GpioCtrlRegs.GPBMUX2.all = 0x0000;
	GpioCtrlRegs.GPBDIR.bit.GPIO42 = 1;

	//產學版設置
	GpioCtrlRegs.GPAPUD.bit.GPIO0   = 0;   // Pull-up
	GpioDataRegs.GPACLEAR.bit.GPIO0 = 1;   // Low
	GpioCtrlRegs.GPAMUX1.bit.GPIO0  = 0;   // GPIO
	GpioCtrlRegs.GPADIR.bit.GPIO0   = 1;   // output

	GpioCtrlRegs.GPAPUD.bit.GPIO1   = 0;   // Pull-up
	GpioDataRegs.GPACLEAR.bit.GPIO1 = 1;   // Low
	GpioCtrlRegs.GPAMUX1.bit.GPIO1  = 0;   // GPIO
	GpioCtrlRegs.GPADIR.bit.GPIO1   = 1;   // output

	GpioCtrlRegs.GPAPUD.bit.GPIO2   = 0;   // Pull-up
	GpioDataRegs.GPACLEAR.bit.GPIO2 = 1;   // Low
	GpioCtrlRegs.GPAMUX1.bit.GPIO2  = 0;   // GPIO
	GpioCtrlRegs.GPADIR.bit.GPIO2   = 1;   // output

	GpioCtrlRegs.GPAPUD.bit.GPIO3   = 0;   // Pull-up
	GpioDataRegs.GPACLEAR.bit.GPIO3 = 1;   // Low
	GpioCtrlRegs.GPAMUX1.bit.GPIO3  = 0;   // GPIO
	GpioCtrlRegs.GPADIR.bit.GPIO3   = 1;   // output

	GpioCtrlRegs.GPAPUD.bit.GPIO4   = 0;   // Pull-up
	GpioDataRegs.GPACLEAR.bit.GPIO4 = 1;   // Low
	GpioCtrlRegs.GPAMUX1.bit.GPIO4  = 0;   // GPIO
	GpioCtrlRegs.GPADIR.bit.GPIO4   = 1;   // output

	GpioCtrlRegs.GPAPUD.bit.GPIO5   = 0;   // Pull-up
	GpioDataRegs.GPACLEAR.bit.GPIO5 = 1;   // Low
	GpioCtrlRegs.GPAMUX1.bit.GPIO5  = 0;   // GPIO
	GpioCtrlRegs.GPADIR.bit.GPIO5   = 1;   // output

	GpioCtrlRegs.GPAPUD.bit.GPIO6   = 0;   // Pull-up
	GpioDataRegs.GPACLEAR.bit.GPIO6 = 1;   // Low
	GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 0;   // GPIO
	GpioCtrlRegs.GPADIR.bit.GPIO6   = 1;   // output

	GpioCtrlRegs.GPAPUD.bit.GPIO7   = 0;   // Pull-up
	GpioDataRegs.GPACLEAR.bit.GPIO7 = 1;   // Low
	GpioCtrlRegs.GPAMUX1.bit.GPIO7  = 0;   // GPIO
	GpioCtrlRegs.GPADIR.bit.GPIO7   = 1;   // output
    EDIS;                                   //嚙踝�嚙賣��蕭�蕭

    GpioDataRegs.GPADAT.all = 0x0000;
    GpioDataRegs.GPBDAT.all = 0x0000;
    /////////////////////////////////////////////////////////////////////
    InitPieCtrl();
    InitPieVectTable();
    InitEQep1Gpio();
    /////////////////////////////////////////////////////////////////////
    IER = 0x0000;
    IFR = 0x0000;

    DINT;
    EALLOW;
    PieVectTable.TINT0 = &cpu_timer0_isr;			//EQEP中斷位置、PID位置
    EDIS;

    InitCpuTimers();						//初始化counter
	ConfigCpuTimer(&CpuTimer0, 80, 10000);	//第二個參數設置cpu速度80MHz 第三個參數為中斷頻率10000 = 0.01s
	CpuTimer0Regs.TCR.all = 0x4001;

	IER |= M_INT1;
    IER |= M_INT3;
    //IER |= M_INT4;
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;  //INT1.7

    EINT;
    ERTM;

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
    SetupEPwm();
    //InitECapture();
    QEP_pos_speed_get_init();		//EQEP初始化
    first_rotation();
    pid_signal = 1;
    DELAY_US(3000000);
    EQep1Regs.QEPCTL.bit.PCRM = 01;
    DELAY_US(1000000);
    EQep1Regs.QEPCTL.bit.SWI = 1;
    DELAY_US(1000000);

    while(1){
    	act = act_stepmotor[i];
		switch(act){
		case 0:   //重新開始
			i = 0;
			break;

		case 1:   //反轉
			i++;
			step = act_stepmotor[i];
			pid_step = step + pid_step;
			pid_step = pid_step % 200;
			i++;
			while(step > 0){
				j++;
				if(j == 4) j=0;
				if(j == 0){
					A1_On(); B1_On();  A3_On(); B3_On();DELAY_US(1);
					A2_Off();  B2_Off(); A4_Off(); B4_Off();
					dt = act_stepmotor[i];
				    DELAY_US(dt);
					step--;
				}
				else if(j == 1){
					A1_Off(); B1_On();  A3_Off(); B3_On();DELAY_US(1);
					A2_On();  B2_Off(); A4_On();  B4_Off();
					dt = act_stepmotor[i];
				    DELAY_US(dt);
					step--;
				}
				else if(j == 2){
					A1_Off(); B1_Off(); A3_Off(); B3_Off(); DELAY_US(1);
					A2_On();  B2_On(); A4_On(); B4_On();
					dt = act_stepmotor[i];
				    DELAY_US(dt);
					step--;
				}
				else if(j == 3){
					A1_On(); B1_Off();  A3_On(); B3_Off();DELAY_US(1);
					A2_Off();  B2_On(); A4_Off(); B4_On();
					dt = act_stepmotor[i];
				    DELAY_US(dt);
					step--;
				}
			}
			i++;
			break;

		case 2:   //正轉
			i++;
			step = act_stepmotor[i];
			pid_step = step + pid_step;
			pid_step = pid_step % 200;
			i++;
			while(step > 0){
				j--;
				if(j < 0) j = 3;
				if(j == 0){
					A1_On(); B1_On();A3_On(); B3_On(); DELAY_US(1);
					A2_Off();  B2_Off();A4_Off();  B4_Off();
					dt = act_stepmotor[i];
                    DELAY_US(dt);
					step--;
				}
				else if(j == 1){
					A1_Off(); B1_On();A3_Off(); B3_On();DELAY_US(1);
					A2_On();  B2_Off();A4_On();  B4_Off();
					dt = act_stepmotor[i];
                    DELAY_US(dt);
					step--;
				}
				else if(j == 2){
					A1_Off(); B1_Off();A3_Off(); B3_Off();DELAY_US(1);
					A2_On();  B2_On();A4_On();  B4_On();
					dt = act_stepmotor[i];
                    DELAY_US(dt);
					step--;
				}
				else if(j == 3){
					A1_On(); B1_Off();A3_On(); B3_Off();DELAY_US(1);
					A2_Off();  B2_On();A4_Off();  B4_On();
					dt = act_stepmotor[i];
                    DELAY_US(dt);
					step--;
				}
			}
			i++;
			break;
		case 3:   //停止
			i++;
			dt = act_stepmotor[i];
			A1_Off(); B1_Off(); A2_Off(); B2_Off(); A3_Off(); B3_Off(); A4_Off(); B4_Off();
            DELAY_US(dt);
			i++;
			break;

		}
    }
}

interrupt void cpu_timer0_isr(void)
{
   CpuTimer0.InterruptCount++;
   // Acknowledge this interrupt to receive more interrupts from group 1
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

void SetupEPwm(void){
	EALLOW;
	SysCtrlRegs.PCLKCR1.bit.EPWM2ENCLK = 1;
	SysCtrlRegs.PCLKCR1.bit.EPWM3ENCLK = 1;
	SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
/*
	EPwm1Regs.TBCTL.bit.CLKDIV = 3;  //嚙賣隤�
	EPwm1Regs.TBCTL.bit.HSPCLKDIV = 5;  //嚙賣隤�0(80Mhz/80)
	EPwm1Regs.TBCTL.bit.CTRMODE = 2;  //�等嚙賜��迎蕭

	EPwm1Regs.TBSTS.all = 0;
	EPwm1Regs.TBPHS.half.TBPHS = 0;
	EPwm1Regs.TBCTR = 0;
	EPwm1Regs.TBPRD = 10000;  //嚙賣�嚙�嚙踐�嚙賜�嚙踝蕭嚙瞑RD*(1/1*10^6) EX.PRD=50000嚙踝蕭0HZ(12RPM)
	                          //嚙賣�嚙�嚙踐�嚙賜�嚙踝蕭嚙瞑RD*(1/1*10^6) EX.PRD=10000嚙踝蕭00HZ(60RPM)
	EPwm1Regs.CMPCTL.bit.SHDWAMODE = 1;
	EPwm1Regs.CMPCTL.bit.SHDWBMODE = 1;
	EPwm1Regs.CMPA.half.CMPA = 5000;  //嚙質�蕭瘥PA�殷�嚙踝蕭�蕭嚙踐�敺嚙�%
	//EPwm1Regs.CMPB = 50000;

	EPwm1Regs.AQCTLA.bit.ZRO = 2;
	EPwm1Regs.AQCTLA.bit.CAU = 1;
	EPwm1Regs.AQCTLA.bit.PRD = 1;
	EPwm1Regs.AQCTLA.bit.CAD = 2;

	EPwm1Regs.AQCTLB.bit.ZRO = 2;
	EPwm1Regs.AQCTLB.bit.CAU = 1;
	EPwm1Regs.AQCTLB.bit.PRD = 1;
	EPwm1Regs.AQCTLB.bit.CAD = 2;

	EPwm1Regs.DBCTL.bit.IN_MODE = 2;   //IN_MODE=2 A嚙賜�嚙踝蕭�抵麾嚙踐�嚙賜�嚙踝蕭�麾嚙踝蕭
	EPwm1Regs.DBCTL.bit.POLSEL = 2;    //POLSEL=2 B嚙踝�嚙�
	EPwm1Regs.DBCTL.bit.OUT_MODE = 3;  //OUT_MODE=3 �輯摩����蕭嚙踐撩嚙踝蕭�麾嚙踝蕭
	EPwm1Regs.DBRED = 1;               //��嚙��箸�嚙�
	EPwm1Regs.DBFED = 1;
*/
/*
	EPwm1Regs.ETSEL.bit.INTEN = 1;    //��謘�麾�輯摩��
	EPwm1Regs.ETSEL.bit.INTSEL = 2;   //ctr=prd
	EPwm1Regs.ETPS.bit.INTPRD = 1;
*/
//////2
	EPwm2Regs.TBCTL.bit.CLKDIV = 3;  //嚙賣隤�
	EPwm2Regs.TBCTL.bit.HSPCLKDIV = 5;  //嚙賣隤�0(80Mhz/80)
	EPwm2Regs.TBCTL.bit.CTRMODE = 2;  //�等嚙賜��迎蕭

	EPwm2Regs.TBSTS.all = 0;
	EPwm2Regs.TBPHS.half.TBPHS = 0;
	EPwm2Regs.TBCTR = 0;
	EPwm2Regs.TBPRD = 10000;  //嚙賣�嚙�嚙踐�嚙賜�嚙踝蕭嚙瞑RD*(1/1*10^6) EX.PRD=50000嚙踝蕭0HZ(12RPM)
							  //嚙賣�嚙�嚙踐�嚙賜�嚙踝蕭嚙瞑RD*(1/1*10^6) EX.PRD=10000嚙踝蕭00HZ(60RPM)
	EPwm2Regs.CMPCTL.bit.SHDWAMODE = 1;
	EPwm2Regs.CMPCTL.bit.SHDWBMODE = 1;
	EPwm2Regs.CMPA.half.CMPA = 5000;  //嚙質�蕭瘥PA�殷�嚙踝蕭�蕭嚙踐�敺嚙�%
	//EPwm2Regs.CMPB = 50000;

	EPwm2Regs.AQCTLA.bit.ZRO = 2;
	EPwm2Regs.AQCTLA.bit.CAU = 1;
	EPwm2Regs.AQCTLA.bit.PRD = 1;
	EPwm2Regs.AQCTLA.bit.CAD = 2;

	EPwm2Regs.AQCTLB.bit.ZRO = 2;
	EPwm2Regs.AQCTLB.bit.CAU = 1;
	EPwm2Regs.AQCTLB.bit.PRD = 1;
	EPwm2Regs.AQCTLB.bit.CAD = 2;

	EPwm2Regs.DBCTL.bit.IN_MODE = 2;   //IN_MODE=2 A嚙賜�嚙踝蕭�抵麾嚙踐�嚙賜�嚙踝蕭�麾嚙踝蕭
	EPwm2Regs.DBCTL.bit.POLSEL = 2;    //POLSEL=2 B嚙踝�嚙�
	EPwm2Regs.DBCTL.bit.OUT_MODE = 3;  //OUT_MODE=3 �輯摩����蕭嚙踐撩嚙踝蕭�麾嚙踝蕭
	EPwm2Regs.DBRED = 1;               //��嚙��箸�嚙�
	EPwm2Regs.DBFED = 1;

	EPwm2Regs.ETSEL.bit.INTEN = 1;    //��謘�麾�輯摩��
	EPwm2Regs.ETSEL.bit.INTSEL = 5;   //ctr=prd
	EPwm2Regs.ETPS.bit.INTPRD = 1;

//////3
	EPwm3Regs.TBCTL.bit.CLKDIV = 3;  //嚙賣隤�
	EPwm3Regs.TBCTL.bit.HSPCLKDIV = 5;  //嚙賣隤�0(80Mhz/80)
	EPwm3Regs.TBCTL.bit.CTRMODE = 2;  //�等嚙賜��迎蕭

	EPwm3Regs.TBSTS.all = 0;
	EPwm3Regs.TBPHS.half.TBPHS = 0;
	EPwm3Regs.TBCTR = 0;
	EPwm3Regs.TBPRD = 10000;  //嚙賣�嚙�嚙踐�嚙賜�嚙踝蕭嚙瞑RD*(1/1*10^6) EX.PRD=50000嚙踝蕭0HZ(12RPM)
							  //嚙賣�嚙�嚙踐�嚙賜�嚙踝蕭嚙瞑RD*(1/1*10^6) EX.PRD=10000嚙踝蕭00HZ(60RPM)
	EPwm3Regs.CMPCTL.bit.SHDWAMODE = 1;
	EPwm3Regs.CMPCTL.bit.SHDWBMODE = 1;
	EPwm3Regs.CMPA.half.CMPA = 5000;  //嚙質�蕭瘥PA�殷�嚙踝蕭�蕭嚙踐�敺嚙�%
	//EPwm2Regs.CMPB = 50000;

	EPwm3Regs.AQCTLA.bit.ZRO = 2;
	EPwm3Regs.AQCTLA.bit.CAU = 2;
	EPwm3Regs.AQCTLA.bit.PRD = 1;
	EPwm3Regs.AQCTLA.bit.CAD = 1;

	EPwm3Regs.AQCTLB.bit.ZRO = 2;
	EPwm3Regs.AQCTLB.bit.CAU = 2;
	EPwm3Regs.AQCTLB.bit.PRD = 1;
	EPwm3Regs.AQCTLB.bit.CAD = 1;

	EPwm3Regs.DBCTL.bit.IN_MODE = 2;   //IN_MODE=2 A嚙賜�嚙踝蕭�抵麾嚙踐�嚙賜�嚙踝蕭�麾嚙踝蕭
	EPwm3Regs.DBCTL.bit.POLSEL = 2;    //POLSEL=2 B嚙踝�嚙�
	EPwm3Regs.DBCTL.bit.OUT_MODE = 3;  //OUT_MODE=3 �輯摩����蕭嚙踐撩嚙踝蕭�麾嚙踝蕭
	EPwm3Regs.DBRED = 1;               //��嚙��箸�嚙�
	EPwm3Regs.DBFED = 1;

	EPwm3Regs.ETSEL.bit.INTEN = 1;    //��謘�麾�輯摩��
	EPwm3Regs.ETSEL.bit.INTSEL = 5;   //ctr=prd
	EPwm3Regs.ETPS.bit.INTPRD = 1;

	SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;
	EDIS;
	//prd2 = EPwm2Regs.TBPRD;
	//prd3 = EPwm3Regs.TBPRD;
	EPwm2Regs.ETCLR.bit.INT = 1;
	EPwm3Regs.ETCLR.bit.INT = 1;
}