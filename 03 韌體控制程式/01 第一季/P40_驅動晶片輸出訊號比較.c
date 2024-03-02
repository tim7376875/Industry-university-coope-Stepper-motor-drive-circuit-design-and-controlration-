#include "DSP28x_Project.h"
#include "math.h"

#define A2_On()     GpioDataRegs.GPASET.bit.GPIO10=1
#define B2_On()     GpioDataRegs.GPASET.bit.GPIO11=1
#define A3_On()     GpioDataRegs.GPBSET.bit.GPIO54=1
#define B3_On()     GpioDataRegs.GPBSET.bit.GPIO34=1

#define A2_Off()    GpioDataRegs.GPACLEAR.bit.GPIO10=1
#define B2_Off()    GpioDataRegs.GPACLEAR.bit.GPIO11=1
#define A3_Off()    GpioDataRegs.GPBCLEAR.bit.GPIO54=1
#define B3_Off()    GpioDataRegs.GPBCLEAR.bit.GPIO34=1

#define NSleep_On()     GpioDataRegs.GPBSET.bit.GPIO39=1

#define CPU_RATE 12.500L // for an 80MHz CPU clock speed


#define NOP()		__asm("		NOP")

void GPIO_SETUP(void);
inline void Delay(unsigned long Delay_Counter);

extern unsigned int RamfuncsLoadStart;   //extern unsigned int 抹除暫存器的值
extern unsigned int RamfuncsLoadEnd;
extern unsigned int RamfuncsRunStart;
void MemCopy(unsigned int *SourceAddr,
			 unsigned int *SourceEndAddr,
			 unsigned int *DestAddr);
int i=0;

int main(void)
{
InitSysCtrl();    //初始化系統控制
	InitPieCtrl();
	InitPieVectTable();  //初始化中斷向量表
	DINT;	// Disable CPU interrupts and clear all CPU interrupt flags  //關閉全部中斷
			//  Call Flash Initialization to setup flash wait-states
			//  This function must reside in RAM
	MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);
	InitFlash();
	IER = 0x0000;	// 除能所有cpu中斷
	IFR = 0x0000;	// 除能所有cpu中斷旗標

     GPIO_SETUP();
     EPWM_SETUP();
	ADC_SETUP();	//Setup ADC
	SPI_SETUP();	//Setup SPI


  //  Sleep_On();
	NSleep_On();
    while(1)
    {

    	        A2_Off(); B2_On();  A3_On();  B3_Off(); DELAY_US(10000*0.2);
                A2_Off(); B2_On();  A3_Off(); B3_On();  DELAY_US(10000*0.2);
                A2_On();  B2_Off(); A3_Off(); B3_On();  DELAY_US(10000*0.2);
                A2_On();  B2_Off(); A3_On();  B3_Off(); DELAY_US(10000*0.2);

    }

}

void MemCopy(unsigned int *SourceAddr,
			 unsigned int *SourceEndAddr,
			 unsigned int *DestAddr)
{
     while(SourceAddr < SourceEndAddr)
     {
        *DestAddr++ = *SourceAddr++;
     }
     return;
}

void GPIO_SETUP(void)
{
	EALLOW;
	//XINT SETUP
	GpioCtrlRegs.GPAPUD.bit.GPIO25	 = 1;	// Disable pull-up
	GpioCtrlRegs.GPAMUX2.bit.GPIO25	 = 1;	// GPIO25 Ecap2
	GpioCtrlRegs.GPADIR.bit.GPIO25	 = 0;	// input
	GpioCtrlRegs.GPAPUD.bit.GPIO26	 = 1;	// Disable pull-up
	GpioCtrlRegs.GPAMUX2.bit.GPIO26	 = 1;	// GPIO9 Ecap3
	GpioCtrlRegs.GPADIR.bit.GPIO26	 = 0;	// input
	GpioCtrlRegs.GPAPUD.bit.GPIO9	 = 1;	// Disable pull-up
	GpioCtrlRegs.GPAMUX1.bit.GPIO9	 = 3;	// GPIO26 Ecap3
	GpioCtrlRegs.GPADIR.bit.GPIO9	 = 0;	// input
	Xint_SETUP();

	//  SPI PIN
		GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 3;    // SPICLKB  (I/O)
		GpioCtrlRegs.GPADIR.bit.GPIO14  = 1;	// Output
		GpioCtrlRegs.GPAMUX2.bit.GPIO24 = 3;	// SPISIMOB  (I/O)
		GpioCtrlRegs.GPADIR.bit.GPIO24  = 1;	// Output
		GpioCtrlRegs.GPAMUX2.bit.GPIO27 = 3;	//SPISTEB   (I/O)
		GpioCtrlRegs.GPADIR.bit.GPIO27  = 1;	// Output

	//	Test LED
		GpioCtrlRegs.GPAPUD.bit.GPIO10	 = 0;	// Pull-up
		GpioDataRegs.GPACLEAR.bit.GPIO10 = 1;	// Low
		GpioCtrlRegs.GPAMUX1.bit.GPIO10	 = 0;	// GPIO
		GpioCtrlRegs.GPADIR.bit.GPIO10	 = 1;	// output

		GpioCtrlRegs.GPAPUD.bit.GPIO11 	 = 0;	// Pull-up
		GpioDataRegs.GPACLEAR.bit.GPIO11 = 1;	// Low
		GpioCtrlRegs.GPAMUX1.bit.GPIO11	 = 0;	// GPIO
		GpioCtrlRegs.GPADIR.bit.GPIO11	 = 1;	// output

		GpioCtrlRegs.GPBPUD.bit.GPIO54 	 = 0;	// Pull-up
		GpioDataRegs.GPBCLEAR.bit.GPIO54 = 1;	// Low
		GpioCtrlRegs.GPBMUX2.bit.GPIO54	 = 0;	// GPIO
		GpioCtrlRegs.GPBDIR.bit.GPIO54	 = 1;	// output

		GpioCtrlRegs.GPBPUD.bit.GPIO34 	 = 0;	// Pull-up
		GpioDataRegs.GPBCLEAR.bit.GPIO34 = 1;	// Low
		GpioCtrlRegs.GPBMUX1.bit.GPIO34	 = 0;	// GPIO
		GpioCtrlRegs.GPBDIR.bit.GPIO34	 = 1;	// output

		GpioCtrlRegs.GPBPUD.bit.GPIO39 	 = 0;	// Pull-up
		GpioDataRegs.GPBCLEAR.bit.GPIO39 = 1;	// Low
		GpioCtrlRegs.GPBMUX1.bit.GPIO39	 = 0;	// GPIO
		GpioCtrlRegs.GPBDIR.bit.GPIO39	 = 1;	// output

		GpioCtrlRegs.GPBPUD.bit.GPIO53 	 = 0;	// Pull-up
		GpioDataRegs.GPBSET.bit.GPIO53 	 = 1;	// Low
		GpioCtrlRegs.GPBMUX2.bit.GPIO53	 = 0;	// GPIO
		GpioCtrlRegs.GPBDIR.bit.GPIO53	 = 1;	// output



	//	ePWM2 pins
		GpioCtrlRegs.GPAPUD.bit.GPIO2  = 0;		// Disable pull-up
		GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;		// EPWM2A
		GpioCtrlRegs.GPAPUD.bit.GPIO3  = 0;		// Disable pull-up
		GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;		// EPWM2B

	//	ePWM3 pins
		GpioCtrlRegs.GPAPUD.bit.GPIO4  = 0;		// Disable pull-up
		GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;		// EPWM3A
		GpioCtrlRegs.GPAPUD.bit.GPIO5  = 0;		// Disable pull-up
		GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;		// EPWM3B
          EDIS;


}

inline void Delay(unsigned long Delay_Counter)
{
 	for( ; Delay_Counter > 0; Delay_Counter--)
		NOP();
}

