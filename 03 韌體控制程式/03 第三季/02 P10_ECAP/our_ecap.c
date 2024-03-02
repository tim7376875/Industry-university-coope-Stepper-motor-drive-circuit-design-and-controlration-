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
	ECap1Regs.ECEINT.all = 0x0000;     //�T��Ҧ�cap���_
	ECap1Regs.ECCLR.all = 0xFFFF;      //�M���Ҧ�flag
	ECap1Regs.ECCTL1.bit.CAPLDEN = 0;  //�T����JCAP1~CAP4����
	ECap1Regs.ECCTL2.bit.TSCTRSTOP = 0;//TSCTR����p��

	ECap1Regs.ECCTL2.bit.CONT_ONESHT = 1;  //�榸�Ҧ�
	ECap1Regs.ECCTL2.bit.STOP_WRAP = 3;    //����ƥ�4�o�ͫᰱ��
	ECap1Regs.ECCTL2.bit.SWSYNC = 1;       //�P�B�Ҧ���Ecap����
    //�t�mECap1Regs�BECap2Regs����椸������
    /*
     *      ____      ____      ____
     *     |    |    |    |    |    |
     * ____|    |____|    |____|    |____
     *     ��    ��    ��    ��
     *     A    B    C    D
     *
     * */
	ECap1Regs.ECCTL1.bit.CAP1POL = 0;          //�bRising edge��Ĳ�o����ƥ�
	ECap1Regs.ECCTL1.bit.CAP2POL = 1;          //�bFalling edge��Ĳ�o����ƥ�
	ECap1Regs.ECCTL1.bit.CAP3POL = 0;          //�bRising edge��Ĳ�o����ƥ�
	ECap1Regs.ECCTL1.bit.CAP4POL = 1;          //�bFalling edge��Ĳ�o����ƥ�
	ECap1Regs.ECCTL1.bit.CTRRST1 = 0;          //������������ᤣ���m�p�ƾ�
	ECap1Regs.ECCTL1.bit.CTRRST2 = 0;          //������������ᤣ���m�p�ƾ�
	ECap1Regs.ECCTL1.bit.CTRRST3 = 0;          //������������ᤣ���m�p�ƾ�
	ECap1Regs.ECCTL1.bit.CTRRST4 = 1;          //������������᭫�m�p�ƾ�
	ECap1Regs.ECCTL2.bit.SYNCI_EN = 1;
	ECap1Regs.ECCTL2.bit.SYNCO_SEL = 0;
	ECap1Regs.ECCTL1.bit.CAPLDEN = 1;          //�ϯ���JCAP1~CAP4����

	ECap1Regs.ECCTL2.bit.CAP_APWM = 0;   //�u�@�bCAP�Ҧ�
	ECap1Regs.ECCTL2.bit.REARM = 1;
}

void ECAP2_init(void){
	ECap2Regs.ECEINT.all = 0x0000;     //�T��Ҧ�cap���_
	ECap2Regs.ECCLR.all = 0xFFFF;      //�M���Ҧ�flag
	ECap2Regs.ECCTL1.bit.CAPLDEN = 0;  //�T����JCAP1~CAP4����
	ECap2Regs.ECCTL2.bit.TSCTRSTOP = 0;//TSCTR����p��

	ECap2Regs.ECCTL2.bit.CONT_ONESHT = 1;  //�榸�Ҧ�
	ECap2Regs.ECCTL2.bit.STOP_WRAP = 3;    //����ƥ�4�o�ͫᰱ��
	ECap2Regs.ECCTL2.bit.SWSYNC = 1;       //�P�B�Ҧ���Ecap����

	ECap2Regs.ECCTL1.bit.CAP1POL = 0;          //�bRising edge��Ĳ�o����ƥ�
	ECap2Regs.ECCTL1.bit.CAP2POL = 1;          //�bFalling edge��Ĳ�o����ƥ�
	ECap2Regs.ECCTL1.bit.CAP3POL = 0;          //�bRising edge��Ĳ�o����ƥ�
	ECap2Regs.ECCTL1.bit.CAP4POL = 1;          //�bFalling edge��Ĳ�o����ƥ�
	ECap2Regs.ECCTL1.bit.CTRRST1 = 0;          //������������ᤣ���m�p�ƾ�
	ECap2Regs.ECCTL1.bit.CTRRST2 = 0;          //������������ᤣ���m�p�ƾ�
	ECap2Regs.ECCTL1.bit.CTRRST3 = 0;          //������������ᤣ���m�p�ƾ�
	ECap2Regs.ECCTL1.bit.CTRRST4 = 1;          //������������᭫�m�p�ƾ�
	ECap2Regs.ECCTL2.bit.SYNCI_EN = 1;
	ECap2Regs.ECCTL2.bit.SYNCO_SEL = 0;
	ECap2Regs.ECCTL1.bit.CAPLDEN = 1;          //�ϯ���JCAP1~CAP4����

	ECap2Regs.ECCTL2.bit.CAP_APWM = 0;   //�u�@�bCAP�Ҧ�
	ECap2Regs.ECCTL2.bit.REARM = 1;
}

void ECAP3_init(void){
	ECap3Regs.ECEINT.all = 0x0000;     //�T��Ҧ�cap���_
	ECap3Regs.ECCLR.all = 0xFFFF;      //�M���Ҧ�flag
	ECap3Regs.ECCTL1.bit.CAPLDEN = 0;  //�T����JCAP1~CAP4����
	ECap3Regs.ECCTL2.bit.TSCTRSTOP = 0;//TSCTR����p��

	ECap3Regs.ECCTL2.bit.CONT_ONESHT = 0;  //�s��Ҧ�
	ECap3Regs.ECCTL2.bit.STOP_WRAP = 1;    //����ƥ�2�o�ͫᰱ��
	ECap3Regs.ECCTL2.bit.SWSYNC = 1;       //�P�B�Ҧ���Ecap����
	//�t�mECap3Regs����椸������
	/*
	 *        ________
	 *       |        |
	 *  _____|        |______________
	 *       ��        ��
	 *       A        B
	 *
	 * */
	ECap3Regs.ECCTL1.bit.CAP1POL = 0;          //�bRising edge��Ĳ�o����ƥ�
	ECap3Regs.ECCTL1.bit.CAP2POL = 1;          //�bFalling edge��Ĳ�o����ƥ�
	ECap3Regs.ECCTL1.bit.CTRRST1 = 0;          //������������ᤣ���m�p�ƾ�
	ECap3Regs.ECCTL1.bit.CTRRST2 = 1;          //������������᭫�m�p�ƾ�
	ECap3Regs.ECCTL2.bit.SYNCI_EN = 1;
	ECap3Regs.ECCTL2.bit.SYNCO_SEL = 0;
	ECap3Regs.ECCTL1.bit.CAPLDEN = 1;          //�ϯ���JCAP1~CAP4����

	ECap3Regs.ECCTL2.bit.CAP_APWM = 0;         //�u�@�bCAP�Ҧ�
	ECap3Regs.ECCTL2.bit.REARM = 1;
	ECap3Regs.ECEINT.bit.CEVT2 = 1;            //�ϯஷ��ƥ�2���_
	ECap3Regs.ECCTL2.bit.TSCTRSTOP = 1;        //�}�l�p��
}
