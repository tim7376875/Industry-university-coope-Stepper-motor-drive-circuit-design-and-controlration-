#include "DSP28x_Project.h"
#include "our_encoder.h"
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

volatile long dt;  //setup DELAY_US() 100000 = 0.1s		5000 = 0.05s
				   //ex: DELAY_US(2000000) is stop 2s
static const unsigned long act_stepmotor[] =
{
	// B. No error 60rpm
	1, 50, 50, 2500, 	//rotation to 150 step stop 3 second
	3, 3000000,

	1, 30, 30, 2500,
	3, 2000000,
	1, 40, 40, 2500,
	3, 2000000,
	1, 50, 50, 2500,
	3, 2000000,
	2, 50, 50, 2500,
	3, 2000000,
	2, 40, 40, 2500,
	3, 2000000,
	2, 30, 30, 2500,
	3, 2000000,
	//0				If looping the action use 0
};

int i = 0, j = 0, k = 0, pi = 0, countf = 0, act = 0, act2 = 0, step, step1, step2;
int i2 = 0, j2 = 0,step_err;

//void SetupEPwm(void);					//EPWM setup(stepmotor not use)
interrupt void cpu_timer0_isr(void);	//EQEP Calculate motor speed
interrupt void cpu_timer1_isr(void);	//PID interrupt(Compensate for motor out of step)
void pid(void);							//first motor perform compensation
void pid2(void);						//second motor perform compensation

void first_pid(void);					//initialization
void first_pid2(void);
void first_rotation(void);

void motor1_CCW(void);					//motor action
void motor2_CCW(void);
void motor1_CW(void);
void motor2_CW(void);
void motor1_pidCCW(void);
void motor2_pidCCW(void);
void motor1_pidCW(void);
void motor2_pidCW(void);

int DirectionQep = 0;		//the direction of the motor
int LineEncoder = 1000;		//Encoder accuracy
int Encoder_N = 4000;
float Speed_Mr_RPM = 0; 	//motor speed(rpm)
float Position_Pre = 0; 	//previous position
float Position_Cur= 0;  	//current position
unsigned int rotation_step;
float rotation_angle;
int DirectionQep2;
int LineEncoder2;
int Encoder_N2;
float Speed_Mr_RPM2;
float Position_Pre2;
float Position_Cur2;
unsigned int rotation_step2;
float rotation_angle2;

int pid_signal = 0;			//initialization signal
int thetashift = 0;			//Motor1 offset
int thetastep = 0;			//Motor1 offset steps
int pid_step1 = 0;
int thetashift2 = 0;		//Motor2 offset
int thetastep2 = 0;			//Motor2 offset steps
int pid_step2 = 0;
long pid_speed = 5000;		//pid speed
long fpid_speed = 5000;		//first pid speed

void main(void){
    InitSysCtrl();
    ///////////////////////////////////////////////////////////////
    MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);
    InitFlash();
    ///////////////////////////////////////////////////////////////
    EALLOW;
    GpioCtrlRegs.GPAMUX1.all = 0x0000;		//GPIOA� initialization
    GpioCtrlRegs.GPAMUX2.all = 0x0000;
    GpioCtrlRegs.GPBMUX1.all = 0x0000;
	GpioCtrlRegs.GPBMUX2.all = 0x0000;
	GpioCtrlRegs.GPBDIR.bit.GPIO42 = 1;
	//GPIO setup
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
	GpioCtrlRegs.GPADIR.bit.GPIO7   = 1;   // output                                  豱�

    GpioDataRegs.GPADAT.all = 0x0000;
    GpioDataRegs.GPBDAT.all = 0x0000;
    EDIS;
    /////////////////////////////////////////////////////////////////////
    InitPieCtrl();
    InitPieVectTable();
    InitEQep1Gpio();
    InitEQep2Gpio();
    /////////////////////////////////////////////////////////////////////
    IER = 0x0000;
    IFR = 0x0000;

    DINT;
    EALLOW;
    PieVectTable.TINT0 = &cpu_timer0_isr;			//EQEP interrupt address
    PieVectTable.TINT1 = &cpu_timer1_isr;			//PID interrupt address
    //PieVectTable.EPWM2_INT = &EPWM2_INT_ISR;
    //PieVectTable.EPWM3_INT = &EPWM3_INT_ISR;
    EDIS;

    InitCpuTimers();						//initialize CpuTimer0����
	ConfigCpuTimer(&CpuTimer0, 80, 100000);	//The second parameter sets the cpu speed to 80MHz
											//and the third parameter is the interrupt frequency 10000 = 0.01s
	ConfigCpuTimer(&CpuTimer1, 80, 300000);
	//ConfigCpuTimer(&CpuTimer2, 80, 1000000);
	CpuTimer0Regs.TCR.all = 0x4001;
	CpuTimer1Regs.TCR.all = 0x4001;
	//CpuTimer2Regs.TCR.all = 0x4001
	IER |= M_INT1;
    IER |= M_INT13;

    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;  //INT1.7
    EINT;
    ERTM;

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
    QEP_pos_speed_get_init();		//EQEP initialization

    first_rotation();
    pid_signal = 1;
    DELAY_US(2000000);				//after first rotation stop for 2 seconds
	if(pid_signal == 1){
	   thetashift = 4000 - EQep1Regs.QPOSCNT;
	   thetastep = thetashift / 20;
	   thetashift2 = 4000 - EQep2Regs.QPOSCNT;
	   thetastep2 = thetashift2 / 20;
	   first_pid2();
	   first_pid();
	   pid_signal = 0;
	}
    DELAY_US(1000000);
    EQep1Regs.QEPCTL.bit.PCRM = 01;
    EQep2Regs.QEPCTL.bit.PCRM = 01;
    DELAY_US(1000000);
    EQep1Regs.QEPCTL.bit.SWI = 1;
    EQep2Regs.QEPCTL.bit.SWI = 1;
    DELAY_US(1000000);

    while(1){
    	act = act_stepmotor[i];
		switch(act){
		case 0:   //return to step 1
			i = 0;
			break;

		case 1:   //motor1&2 CCW
			i++;
			step1 = act_stepmotor[i];
			i++;
			step2 = act_stepmotor[i];
			pid_step1 = -1 * step1 + pid_step1;
			pid_step1 = pid_step1 % 200;
			pid_step2 = -1 * step2 + pid_step2;
			pid_step2 = pid_step2 % 200;

			i++;
			while(step1 > 0 || step2 > 0){
				if(step1 == 0){
					A1_Off(); B1_Off(); A2_Off(); B2_Off();
				}
				if(step2 == 0){
					A3_Off(); B3_Off(); A4_Off(); B4_Off();
				}
				motor1_CCW();
				motor2_CCW();
			}
			i++;
			break;

		case 2:   //motor1&2 CW
			i++;
			step1 = act_stepmotor[i];
			i++;
			step2 = act_stepmotor[i];
			pid_step1 = step1 + pid_step1;
			pid_step1 = pid_step1 % 200;
			pid_step2 = step2 + pid_step2;
			pid_step2 = pid_step2 % 200;

			i++;
			while(step1 > 0 || step2 > 0){
				if(step1 == 0){
					A1_Off(); B1_Off(); A2_Off(); B2_Off();
				}
				if(step2 == 0){
					A3_Off(); B3_Off(); A4_Off(); B4_Off();
				}
				motor1_CW();
				motor2_CW();
			}
			i++;
			break;

		case 3:   //motor stop
			i++;
			dt = act_stepmotor[i];
			A1_Off(); B1_Off(); A2_Off(); B2_Off(); A3_Off(); B3_Off(); A4_Off(); B4_Off();
			DELAY_US(dt);
			i++;
			break;

		case 4:   //motor1 CCW & motor2 CW
			i++;
			step1 = act_stepmotor[i];
			i++;
			step2 = act_stepmotor[i];
			pid_step1 = -1 * step1 + pid_step1;
			pid_step1 = pid_step1 % 200;
			pid_step2 = step2 + pid_step2;
			pid_step2 = pid_step2 % 200;

			i++;
			while(step1 > 0 || step2 > 0){
				if(step1 == 0){
					A1_Off(); B1_Off(); A2_Off(); B2_Off();
				}
				if(step2 == 0){
					A3_Off(); B3_Off(); A4_Off(); B4_Off();
				}
				motor1_CCW();
				motor2_CW();
			}
			i++;
			break;

		case 5:   //motor1 CW & motor2 CCW
			i++;
			step1 = act_stepmotor[i];
			i++;
			step2 = act_stepmotor[i];
			pid_step1 = step1 + pid_step1;
			pid_step1 = pid_step1 % 200;
			pid_step2 = -1 * step2 + pid_step2;
			pid_step2 = pid_step2 % 200;

			i++;
			while(step1 > 0 || step2 > 0){
				if(step1 == 0){
					A1_Off(); B1_Off(); A2_Off(); B2_Off();
				}
				if(step2 == 0){
					A3_Off(); B3_Off(); A4_Off(); B4_Off();
				}
				motor1_CW();
				motor2_CCW();
			}
			i++;
			break;
		}
    }
}

interrupt void cpu_timer0_isr(void){
   CpuTimer0.InterruptCount++;
   if(act == 0 || act == 1 || act == 2 || act == 3 || act == 4 || act == 5){
	   QEP_pos_speed_get_Calc();
	   //QEP_pos_speed_get_Calc2();
   }

   // Acknowledge this interrupt to receive more interrupts from group 1
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

interrupt void cpu_timer1_isr(void){
//	CpuTimer1.InterruptCount++;
//
//	if(pid_step1 < 0){
//		pid_step1 = 200 + pid_step1;
//	}
//	if(pid_step2 < 0){
//		pid_step2 = 200 + pid_step2;
//	}
//
//	if(act == 3){
//		//MOTOR1
//		thetashift = pid_step1 * 20 - EQep1Regs.QPOSCNT;
//		   if(thetashift > 10){//CW
//			   if(thetashift % 20 == 0){
//				   thetastep = thetashift / 20;
//				   pid();
//			   }
//			   else{
//				   thetastep = thetashift / 20 + 1;
//				   pid();
//			   }
//		   }
//		   else if( thetashift < -10 && thetashift > -3800 ){ //CCW
//			   if(thetashift % 20 == 0){
//				   thetastep = thetashift / 20;
//				   pid();
//			   }
//			   else{
//				   thetastep = thetashift / 20 - 1;
//				   pid();
//			   }
//		   }
//		   else if( thetashift < -3800 ){ //CW
//			   thetashift = 4000 + thetashift;
//			   if( thetashift > 10 ){
//				   if(thetashift % 20 == 0){
//					   thetastep = thetashift / 20;
//					   pid();
//				   }
//				   else{
//					   thetastep = thetashift / 20 + 1;
//					   pid();
//				   }
//			   }
//		   }else{
//		   //MOTOR2
//		   thetashift2 = pid_step2 * 20 - EQep2Regs.QPOSCNT;
//			if(thetashift2 > 10){//CW
//			   if(thetashift2 % 20 == 0){
//				   thetastep2 = thetashift2 / 20;
//				   pid2();
//			   }
//			   else{
//				   thetastep2 = thetashift2 / 20 + 1;
//				   pid2();
//			   }
//		   }
//		   else if( thetashift2 < -10 && thetashift2 > -3800 ){ //CCW
//			   if(thetashift2 % 20 == 0){
//				   thetastep2 = thetashift2 / 20;
//				   pid2();
//			   }
//			   else{
//				   thetastep2 = thetashift2 / 20 - 1;
//				   pid2();
//			   }
//		   }
//		   else if( thetashift2 < -3800 ){ //CW
//			   thetashift2 = 4000 + thetashift2;
//			   if( thetashift > 10 ){
//				   if(thetashift2 % 20 == 0){
//					   thetastep2 = thetashift2 / 20;
//					   pid2();
//				   }
//				   else{
//					   thetastep2 = thetashift2 / 20 + 1;
//					   pid2();
//				   }
//			   }
//		   }
//		 }
//	}
}

void pid(void){

	if(thetastep < 0){
		step1 =  -1 * thetastep;
		while(step1 > 0){
			j++;
			if(j == 4) j=0;
			if(j == 0){
				A1_On(); B1_On();  DELAY_US(1);
				A2_Off();  B2_Off();
				DELAY_US(pid_speed);
				step1--;
			}
			else if(j == 1){
				A1_Off(); B1_On();  DELAY_US(1);
				A2_On();  B2_Off();
				DELAY_US(pid_speed);
				step1--;
			}
			else if(j == 2){
				A1_Off(); B1_Off(); DELAY_US(1);
				A2_On();  B2_On();
				DELAY_US(pid_speed);
				step1--;
			}
			else if(j == 3){
				A1_On(); B1_Off();  DELAY_US(1);
				A2_Off();  B2_On();
				DELAY_US(pid_speed);
				step1--;
			}
		}
		A1_Off(); B1_Off(); A2_Off(); B2_Off();
	}

	if(thetastep > 0){
		step1 = thetastep;
		while(step1 > 0){
			j--;
			if(j < 0) j = 3;
			if(j == 0){
				A1_On(); B1_On(); DELAY_US(1);
				A2_Off();  B2_Off();
				DELAY_US(pid_speed);
				step1--;
			}
			else if(j == 1){
				A1_Off(); B1_On();DELAY_US(1);
				A2_On();  B2_Off();
				DELAY_US(pid_speed);
				step1--;
			}
			else if(j == 2){
				A1_Off(); B1_Off();DELAY_US(1);
				A2_On();  B2_On();
				DELAY_US(pid_speed);
				step1--;
			}
			else if(j == 3){
				A1_On(); B1_Off();DELAY_US(1);
				A2_Off();  B2_On();
				DELAY_US(pid_speed);
				step1--;
			}
		}
		A1_Off(); B1_Off(); A2_Off(); B2_Off();
	}
}

void pid2(void){

	if(thetastep2 < 0){
		step2 =  -1 * thetastep2;
		while(step2 > 0){
			j2++;
			if(j2 == 4) j2=0;
			if(j2 == 0){
				A3_On(); B3_On();  DELAY_US(1);
				A4_Off();  B4_Off();
				DELAY_US(pid_speed);
				step2--;
			}
			else if(j2 == 1){
				A3_Off(); B3_On();  DELAY_US(1);
				A4_On();  B4_Off();
				DELAY_US(pid_speed);
				step2--;
			}
			else if(j2 == 2){
				A3_Off(); B3_Off();  DELAY_US(1);
				A4_On();  B4_On();
				DELAY_US(pid_speed);
				step2--;
			}
			else if(j2 == 3){
				A3_On(); B3_Off();  DELAY_US(1);
				A4_Off();  B4_On();
				DELAY_US(pid_speed);
				step2--;
			}
		}
		A3_Off(); B3_Off(); A4_Off(); B4_Off();
	}

	if(thetastep2 > 0){
		step2 = thetastep2;
		while(step2 > 0){
			j2--;
			if(j2 < 0) j2 = 3;
			if(j2 == 0){
				A3_On(); B3_On(); DELAY_US(1);
				A4_Off();  B4_Off();
				DELAY_US(pid_speed);
				step2--;
			}
			else if(j2 == 1){
				A3_Off(); B3_On(); DELAY_US(1);
				A4_On();  B4_Off();
				DELAY_US(pid_speed);
				step2--;
			}
			else if(j2 == 2){
				A3_Off(); B3_Off(); DELAY_US(1);
				A4_On();  B4_On();
				DELAY_US(pid_speed);
				step2--;
			}
			else if(j2 == 3){
				A3_On(); B3_Off(); DELAY_US(1);
				A4_Off();  B4_On();
				DELAY_US(pid_speed);
				step2--;
			}
		}
		A3_Off(); B3_Off(); A4_Off(); B4_Off();
	}
}

void first_pid(void){
	step = thetastep;
	while(step > 0){
		j--;
		if(j < 0) j = 3;
		if(j == 0){
			A1_On(); B1_On(); DELAY_US(1);
			A2_Off();  B2_Off();
			DELAY_US(fpid_speed);
			step--;
		}
		else if(j == 1){
			A1_Off(); B1_On(); DELAY_US(1);
			A2_On();  B2_Off();
			DELAY_US(fpid_speed);
			step--;
		}
		else if(j == 2){
			A1_Off(); B1_Off(); DELAY_US(1);
			A2_On();  B2_On();
			DELAY_US(fpid_speed);
			step--;
		}
		else if(j == 3){
			A1_On(); B1_Off(); DELAY_US(1);
			A2_Off();  B2_On();
			DELAY_US(fpid_speed);
			step--;
		}
	}
	A1_Off(); B1_Off(); A2_Off(); B2_Off();
}
void first_pid2(void){

	step2 = thetastep2;
	while(step2 > 0){
		j2--;
		if(j2 < 0) j2 = 3;
		if(j2 == 0){
			A3_On(); B3_On(); DELAY_US(1);
			A4_Off();  B4_Off();
			DELAY_US(fpid_speed);
			step2--;
		}
		else if(j2 == 1){
			A3_Off(); B3_On(); DELAY_US(1);
			A4_On();  B4_Off();
			DELAY_US(fpid_speed);
			step2--;
		}
		else if(j2 == 2){
			A3_Off(); B3_Off(); DELAY_US(1);
			A4_On();  B4_On();
			DELAY_US(fpid_speed);
			step2--;
		}
		else if(j2 == 3){
			A3_On(); B3_Off(); DELAY_US(1);
			A4_Off();  B4_On();
			DELAY_US(fpid_speed);
			step2--;
		}
	}
	A3_Off(); B3_Off(); A4_Off(); B4_Off();
}

void first_rotation(void){
	step = 210;
	step = step * 2; //two motor
	dt = fpid_speed; //motor speed
	while(step > 0){
		motor1_pidCW();
		motor2_pidCW();
	}
	A1_Off(); B1_Off(); A2_Off(); B2_Off(); A3_Off(); B3_Off(); A4_Off(); B4_Off();
}

// Motor rotation
void motor1_CCW(void){
	j++;
	if(j == 4) j=0;
	if(j == 0){
		A1_On(); B1_On(); DELAY_US(1);
		A2_Off();  B2_Off();
		dt = act_stepmotor[i];
		DELAY_US(dt);
		step1--;
	}
	else if(j == 1){
		A1_Off(); B1_On(); DELAY_US(1);
		A2_On();  B2_Off();
		dt = act_stepmotor[i];
		DELAY_US(dt);
		step1--;
	}
	else if(j == 2){
		A1_Off(); B1_Off(); DELAY_US(1);
		A2_On();  B2_On();
		dt = act_stepmotor[i];
		DELAY_US(dt);
		step1--;
	}
	else if(j == 3){
		A1_On(); B1_Off(); DELAY_US(1);
		A2_Off();  B2_On();
		dt = act_stepmotor[i];
		DELAY_US(dt);
		step1--;
	}
}
void motor2_CCW(void){
	j2++;
	if(j2 == 4) j2=0;
	if(j2 == 0){
		A3_On(); B3_On(); DELAY_US(1);
		A4_Off();  B4_Off();
		dt = act_stepmotor[i];
		DELAY_US(dt);
		step2--;
	}
	else if(j2 == 1){
		A3_Off(); B3_On(); DELAY_US(1);
		A4_On();  B4_Off();
		dt = act_stepmotor[i];
		DELAY_US(dt);
		step2--;
	}
	else if(j2 == 2){
		A3_Off(); B3_Off(); DELAY_US(1);
		A4_On();  B4_On();
		dt = act_stepmotor[i];
		DELAY_US(dt);
		step2--;
	}
	else if(j2 == 3){
		A3_On(); B3_Off(); DELAY_US(1);
		A4_Off();  B4_On();
		dt = act_stepmotor[i];
		DELAY_US(dt);
		step2--;
	}
}
void motor1_CW(void){
	j--;
	if(j < 0) j = 3;
	if(j == 0){
		A1_On(); B1_On(); DELAY_US(1);
		A2_Off();  B2_Off();
		dt = act_stepmotor[i];
		DELAY_US(dt);
		step1--;
	}
	else if(j == 1){
		A1_Off(); B1_On(); DELAY_US(1);
		A2_On();  B2_Off();
		dt = act_stepmotor[i];
		DELAY_US(dt);
		step1--;
	}
	else if(j == 2){
		A1_Off(); B1_Off(); DELAY_US(1);
		A2_On();  B2_On();
		dt = act_stepmotor[i];
		DELAY_US(dt);
		step1--;
	}
	else if(j == 3){
		A1_On(); B1_Off(); DELAY_US(1);
		A2_Off();  B2_On();
		dt = act_stepmotor[i];
		DELAY_US(dt);
		step1--;
	}
}
void motor2_CW(void){
	j2--;
	if(j2 < 0) j2 = 3;
	if(j2 == 0){
		A3_On(); B3_On(); DELAY_US(1);
		A4_Off();  B4_Off();
		dt = act_stepmotor[i];
		DELAY_US(dt);
		step2--;
	}
	else if(j2 == 1){
		A3_Off(); B3_On();DELAY_US(1);
		A4_On();  B4_Off();
		dt = act_stepmotor[i];
		DELAY_US(dt);
		step2--;
	}
	else if(j2 == 2){
		A3_Off(); B3_Off();DELAY_US(1);
		A4_On();  B4_On();
		dt = act_stepmotor[i];
		DELAY_US(dt);
		step2--;
	}
	else if(j2 == 3){
		A3_On(); B3_Off();DELAY_US(1);
		A4_Off();  B4_On();
		dt = act_stepmotor[i];
		DELAY_US(dt);
		step2--;
	}
}

// PID_motor rotation
void motor1_pidCCW(void){
	j++;
	if(j == 4) j=0;
	if(j == 0){
		A1_On(); B1_On(); DELAY_US(1);
		A2_Off();  B2_Off();
		DELAY_US(dt);
		step--;
	}
	else if(j == 1){
		A1_Off(); B1_On(); DELAY_US(1);
		A2_On();  B2_Off();
		DELAY_US(dt);
		step--;
	}
	else if(j == 2){
		A1_Off(); B1_Off(); DELAY_US(1);
		A2_On();  B2_On();
		DELAY_US(dt);
		step--;
	}
	else if(j == 3){
		A1_On(); B1_Off(); DELAY_US(1);
		A2_Off();  B2_On();
		DELAY_US(dt);
		step--;
	}
}
void motor2_pidCCW(void){
	j2++;
	if(j2 == 4) j2=0;
	if(j2 == 0){
		A3_On(); B3_On(); DELAY_US(1);
		A4_Off();  B4_Off();
		DELAY_US(dt);
		step--;
	}
	else if(j2 == 1){
		A3_Off(); B3_On(); DELAY_US(1);
		A4_On();  B4_Off();
		DELAY_US(dt);
		step--;
	}
	else if(j2 == 2){
		A3_Off(); B3_Off(); DELAY_US(1);
		A4_On();  B4_On();
		DELAY_US(dt);
		step--;
	}
	else if(j2 == 3){
		A3_On(); B3_Off(); DELAY_US(1);
		A4_Off();  B4_On();
		DELAY_US(dt);
		step--;
	}
}
void motor1_pidCW(void){
	j--;
	if(j < 0) j = 3;
	if(j == 0){
		A1_On(); B1_On(); DELAY_US(1);
		A2_Off();  B2_Off();
		DELAY_US(dt);
		step--;
	}
	else if(j == 1){
		A1_Off(); B1_On(); DELAY_US(1);
		A2_On();  B2_Off();
		DELAY_US(dt);
		step--;
	}
	else if(j == 2){
		A1_Off(); B1_Off(); DELAY_US(1);
		A2_On();  B2_On();
		DELAY_US(dt);
		step--;
	}
	else if(j == 3){
		A1_On(); B1_Off(); DELAY_US(1);
		A2_Off();  B2_On();
		DELAY_US(dt);
		step--;
	}
}
void motor2_pidCW(void){
	j2--;
	if(j2 < 0) j2 = 3;
	if(j2 == 0){
		A3_On(); B3_On(); DELAY_US(1);
		A4_Off();  B4_Off();
		DELAY_US(dt);
		step--;
	}
	else if(j2 == 1){
		A3_Off(); B3_On();DELAY_US(1);
		A4_On();  B4_Off();
		DELAY_US(dt);
		step--;
	}
	else if(j2 == 2){
		A3_Off(); B3_Off();DELAY_US(1);
		A4_On();  B4_On();
		DELAY_US(dt);
		step--;
	}
	else if(j2 == 3){
		A3_On(); B3_Off();DELAY_US(1);
		A4_Off();  B4_On();
		DELAY_US(dt);
		step--;
	}
}
