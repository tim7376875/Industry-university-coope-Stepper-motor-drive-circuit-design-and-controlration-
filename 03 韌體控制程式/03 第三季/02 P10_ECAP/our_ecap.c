/*
 * our_ecap.c
 *
 *  Created on: 2022/11/2
 *      Author: USER
 */

#include "our_ecap.h"

void InitECapture(void)
{
	ECAP1_init();
	ECAP2_init();
	ECAP3_init();
}

void ECAP1_init(void){
	ECap1Regs.ECEINT.all = 0x0000;     //禁止所有cap中斷
	ECap1Regs.ECCLR.all = 0xFFFF;      //清除所有flag
	ECap1Regs.ECCTL1.bit.CAPLDEN = 0;  //禁止載入CAP1~CAP4的值
	ECap1Regs.ECCTL2.bit.TSCTRSTOP = 0;//TSCTR停止計數

	ECap1Regs.ECCTL2.bit.CONT_ONESHT = 1;  //單次模式
	ECap1Regs.ECCTL2.bit.STOP_WRAP = 3;    //捕獲事件4發生後停止
	ECap1Regs.ECCTL2.bit.SWSYNC = 1;       //同步所有的Ecap時鐘
    //配置ECap1Regs、ECap2Regs捕獲單元的極性
    /*
     *      ____      ____      ____
     *     |    |    |    |    |    |
     * ____|    |____|    |____|    |____
     *     ↑    ↓    ↑    ↓
     *     A    B    C    D
     *
     * */
	ECap1Regs.ECCTL1.bit.CAP1POL = 0;          //在Rising edge時觸發捕獲事件
	ECap1Regs.ECCTL1.bit.CAP2POL = 1;          //在Falling edge時觸發捕獲事件
	ECap1Regs.ECCTL1.bit.CAP3POL = 0;          //在Rising edge時觸發捕獲事件
	ECap1Regs.ECCTL1.bit.CAP4POL = 1;          //在Falling edge時觸發捕獲事件
	ECap1Regs.ECCTL1.bit.CTRRST1 = 0;          //完成此次捕獲後不重置計數器
	ECap1Regs.ECCTL1.bit.CTRRST2 = 0;          //完成此次捕獲後不重置計數器
	ECap1Regs.ECCTL1.bit.CTRRST3 = 0;          //完成此次捕獲後不重置計數器
	ECap1Regs.ECCTL1.bit.CTRRST4 = 1;          //完成此次捕獲後重置計數器
	ECap1Regs.ECCTL2.bit.SYNCI_EN = 1;
	ECap1Regs.ECCTL2.bit.SYNCO_SEL = 0;
	ECap1Regs.ECCTL1.bit.CAPLDEN = 1;          //使能載入CAP1~CAP4的值

	ECap1Regs.ECCTL2.bit.CAP_APWM = 0;   //工作在CAP模式
	ECap1Regs.ECCTL2.bit.REARM = 1;
}

void ECAP2_init(void){
	ECap2Regs.ECEINT.all = 0x0000;     //禁止所有cap中斷
	ECap2Regs.ECCLR.all = 0xFFFF;      //清除所有flag
	ECap2Regs.ECCTL1.bit.CAPLDEN = 0;  //禁止載入CAP1~CAP4的值
	ECap2Regs.ECCTL2.bit.TSCTRSTOP = 0;//TSCTR停止計數

	ECap2Regs.ECCTL2.bit.CONT_ONESHT = 1;  //單次模式
	ECap2Regs.ECCTL2.bit.STOP_WRAP = 3;    //捕獲事件4發生後停止
	ECap2Regs.ECCTL2.bit.SWSYNC = 1;       //同步所有的Ecap時鐘

	ECap2Regs.ECCTL1.bit.CAP1POL = 0;          //在Rising edge時觸發捕獲事件
	ECap2Regs.ECCTL1.bit.CAP2POL = 1;          //在Falling edge時觸發捕獲事件
	ECap2Regs.ECCTL1.bit.CAP3POL = 0;          //在Rising edge時觸發捕獲事件
	ECap2Regs.ECCTL1.bit.CAP4POL = 1;          //在Falling edge時觸發捕獲事件
	ECap2Regs.ECCTL1.bit.CTRRST1 = 0;          //完成此次捕獲後不重置計數器
	ECap2Regs.ECCTL1.bit.CTRRST2 = 0;          //完成此次捕獲後不重置計數器
	ECap2Regs.ECCTL1.bit.CTRRST3 = 0;          //完成此次捕獲後不重置計數器
	ECap2Regs.ECCTL1.bit.CTRRST4 = 1;          //完成此次捕獲後重置計數器
	ECap2Regs.ECCTL2.bit.SYNCI_EN = 1;
	ECap2Regs.ECCTL2.bit.SYNCO_SEL = 0;
	ECap2Regs.ECCTL1.bit.CAPLDEN = 1;          //使能載入CAP1~CAP4的值

	ECap2Regs.ECCTL2.bit.CAP_APWM = 0;   //工作在CAP模式
	ECap2Regs.ECCTL2.bit.REARM = 1;
}

void ECAP3_init(void){
	ECap3Regs.ECEINT.all = 0x0000;     //禁止所有cap中斷
	ECap3Regs.ECCLR.all = 0xFFFF;      //清除所有flag
	ECap3Regs.ECCTL1.bit.CAPLDEN = 0;  //禁止載入CAP1~CAP4的值
	ECap3Regs.ECCTL2.bit.TSCTRSTOP = 0;//TSCTR停止計數

	ECap3Regs.ECCTL2.bit.CONT_ONESHT = 0;  //連續模式
	ECap3Regs.ECCTL2.bit.STOP_WRAP = 1;    //捕獲事件2發生後停止
	ECap3Regs.ECCTL2.bit.SWSYNC = 1;       //同步所有的Ecap時鐘
	//配置ECap3Regs捕獲單元的極性
	/*
	 *        ________
	 *       |        |
	 *  _____|        |______________
	 *       ↑        ↓
	 *       A        B
	 *
	 * */
	ECap3Regs.ECCTL1.bit.CAP1POL = 0;          //在Rising edge時觸發捕獲事件
	ECap3Regs.ECCTL1.bit.CAP2POL = 1;          //在Falling edge時觸發捕獲事件
	ECap3Regs.ECCTL1.bit.CTRRST1 = 0;          //完成此次捕獲後不重置計數器
	ECap3Regs.ECCTL1.bit.CTRRST2 = 1;          //完成此次捕獲後重置計數器
	ECap3Regs.ECCTL2.bit.SYNCI_EN = 1;
	ECap3Regs.ECCTL2.bit.SYNCO_SEL = 0;
	ECap3Regs.ECCTL1.bit.CAPLDEN = 1;          //使能載入CAP1~CAP4的值

	ECap3Regs.ECCTL2.bit.CAP_APWM = 0;         //工作在CAP模式
	ECap3Regs.ECCTL2.bit.REARM = 1;
	ECap3Regs.ECEINT.bit.CEVT2 = 1;            //使能捕獲事件2中斷
	ECap3Regs.ECCTL2.bit.TSCTRSTOP = 1;        //開始計數
}
