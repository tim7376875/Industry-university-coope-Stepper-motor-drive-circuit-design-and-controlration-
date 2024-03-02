#include "DSP28x_Project.h"
#include <stdio.h>
#include<string.h>
#include<math.h>


#define A1_On()     GpioDataRegs.GPASET.bit.GPIO10=1
#define B1_On()     GpioDataRegs.GPASET.bit.GPIO11=1
#define A2_On()     GpioDataRegs.GPBSET.bit.GPIO54=1
#define B2_On()     GpioDataRegs.GPBSET.bit.GPIO34=1
#define A1_Off()    GpioDataRegs.GPACLEAR.bit.GPIO10=1
#define B1_Off()    GpioDataRegs.GPACLEAR.bit.GPIO11=1
#define A2_Off()    GpioDataRegs.GPBCLEAR.bit.GPIO54=1
#define B2_Off()    GpioDataRegs.GPBCLEAR.bit.GPIO34=1

#define NSleep_On()     GpioDataRegs.GPBSET.bit.GPIO39=1

#define LED_On()     GpioDataRegs.GPBSET.bit.GPIO53=1
#define LED_Off()    GpioDataRegs.GPBCLEAR.bit.GPIO53=1

volatile int dt;
static const unsigned int act_stepmotor[] =
{ 1, 200, 10000,  // case1:馬達正轉，200為步數，
                  // 1萬為與下一步間隔時間0.01秒
  3, 5000000,     // case3:馬達停止，100萬為停止5秒                
  0               // case0:程序結束
};

int i = 0, j = 0, act, step;

/////////////////////////////////////////////////////////
extern unsigned int RamfuncsLoadStart;   //extern unsigned int 抹除暫存器的值
extern unsigned int RamfuncsLoadEnd;
extern unsigned int RamfuncsRunStart;
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
/////////////////////////////////////////////////////////


int main(void){
    InitSysCtrl();
    ///////////////////////////////////////////////////////////////
    MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);
    InitFlash();
    ///////////////////////////////////////////////////////////////
    //InitGpio();
    EALLOW;                                 //關閉寫保護
    GpioCtrlRegs.GPAMUX1.all = 0x0000;      //GPIO0~15設為GPIO功能
    GpioCtrlRegs.GPAMUX2.all = 0x0000;
    GpioCtrlRegs.GPBMUX1.all = 0x0000;
    GpioCtrlRegs.GPBMUX2.all = 0x0000;
    GpioCtrlRegs.GPADIR.all = 0xFFFF;       //A組GPIO皆設為輸出
    GpioCtrlRegs.GPBDIR.all = 0xFFFF;       //B組GPIO皆設為輸出

    //  Test LED
    GpioCtrlRegs.GPAPUD.bit.GPIO10   = 0;   // Pull-up
    GpioDataRegs.GPACLEAR.bit.GPIO10 = 1;   // Low
    GpioCtrlRegs.GPAMUX1.bit.GPIO10  = 0;   // GPIO
    GpioCtrlRegs.GPADIR.bit.GPIO10   = 1;   // output

    GpioCtrlRegs.GPAPUD.bit.GPIO11   = 0;   // Pull-up
    GpioDataRegs.GPACLEAR.bit.GPIO11 = 1;   // Low
    GpioCtrlRegs.GPAMUX1.bit.GPIO11  = 0;   // GPIO
    GpioCtrlRegs.GPADIR.bit.GPIO11   = 1;   // output

    GpioCtrlRegs.GPBPUD.bit.GPIO54   = 0;   // Pull-up
    GpioDataRegs.GPBCLEAR.bit.GPIO54 = 1;   // Low
    GpioCtrlRegs.GPBMUX2.bit.GPIO54  = 0;   // GPIO
    GpioCtrlRegs.GPBDIR.bit.GPIO54   = 1;   // output

    GpioCtrlRegs.GPBPUD.bit.GPIO34   = 0;   // Pull-up
    GpioDataRegs.GPBCLEAR.bit.GPIO34 = 1;   // Low
    GpioCtrlRegs.GPBMUX1.bit.GPIO34  = 0;   // GPIO
    GpioCtrlRegs.GPBDIR.bit.GPIO34   = 1;   // output

    GpioCtrlRegs.GPBPUD.bit.GPIO39   = 0;   // Pull-up
    GpioDataRegs.GPBCLEAR.bit.GPIO39 = 1;   // Low
    GpioCtrlRegs.GPBMUX1.bit.GPIO39  = 0;   // GPIO
    GpioCtrlRegs.GPBDIR.bit.GPIO39   = 1;   // output

    GpioCtrlRegs.GPBPUD.bit.GPIO53   = 0;   // Pull-up
    GpioDataRegs.GPBSET.bit.GPIO53   = 1;   // Low
    GpioCtrlRegs.GPBMUX2.bit.GPIO53  = 0;   // GPIO
    GpioCtrlRegs.GPBDIR.bit.GPIO53   = 1;   // output

    EDIS;                                   //開啟寫保護
    GpioDataRegs.GPADAT.all = 0x0000;
    GpioDataRegs.GPBDAT.all = 0x0000;
    /////////////////////////////////////////////////////////////////////
    InitPieCtrl();
    InitPieVectTable();  //初始化中斷向量表
    /////////////////////////////////////////////////////////////////////

    IER = 0x0000;
    IFR = 0x0000;
    DINT;

    NSleep_On();

    while(1){
        act = act_stepmotor[i];
        switch(act){
        case 0:   //結束
            i = 0;
            break;

        case 1:   //馬達正轉
            i++;
            step = act_stepmotor[i];
            i++;
            while(step > 0){
                j++;
                if(j == 4) j=0;
                if(j == 0){
                    A1_On(); B1_On(); A2_Off();  B2_Off();
                    dt = act_stepmotor[i];
                    DELAY_US(100000);
                    step--;
                }
                else if(j == 1){
                    A1_Off(); B1_On(); A2_On();  B2_Off();
                    dt = act_stepmotor[i];
                    DELAY_US(100000);
                    step--;
                }
                else if(j == 2){
                    A1_Off(); B1_Off(); A2_On();  B2_On();
                    dt = act_stepmotor[i];
                    DELAY_US(100000);
                    step--;
                }
                else if(j == 3){
                    A1_On(); B1_Off(); A2_Off();  B2_On();
                    dt = act_stepmotor[i];
                    DELAY_US(100000);
                    step--;
                }
            }
            i++;
            break;

        case 2:   //馬達反轉
            i++;
            step = act_stepmotor[i];
            i++;
            while(step > 0){
                j--;
                if(j < 0) j = 3;
                if(j == 0){
                    A1_On(); B1_On(); A2_Off();  B2_Off();
                    dt = act_stepmotor[i];
                    DELAY_US(100000);
                    step--;
                }
                else if(j == 1){
                    A1_Off(); B1_On(); A2_On();  B2_Off();
                    dt = act_stepmotor[i];
                    DELAY_US(100000);
                    step--;
                }
                else if(j == 2){
                    A1_Off(); B1_Off(); A2_On();  B2_On();
                    dt = act_stepmotor[i];
                    DELAY_US(100000);
                    step--;
                }
                else if(j == 3){
                    A1_On(); B1_Off(); A2_Off();  B2_On();
                    dt = act_stepmotor[i];
                    DELAY_US(100000);
                    step--;
                }
            }
            i++;
            break;
        case 3:   //馬達停止
            i++;
            dt = act_stepmotor[i];
            A1_Off(); B1_Off(); A2_Off();  B2_Off();
            DELAY_US(5000000);
            i++;
            break;
        }
    }
}
