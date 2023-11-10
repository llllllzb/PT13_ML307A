/*
 * app_key.c
 *
 *  Created on: Jun 15, 2022
 *      Author: nimo
 */
#include "app_key.h"
#include "app_net.h"

keyctrl_t pwkkey;

void keyInit(void)
{
	memset(&pwkkey, 0, sizeof(pwkkey));
}

static int8_t shutDownId = -1;
void systemShutDown(void)
{
	SYS_POWER_OFF;	
}

void systemShutDownSuccess(void)
{
	if (shutDownId == -1)
	{
		stopTimer(shutDownId);
		shutDownId = -1;
	}
}

void systemShutDownTimeout(void)
{
	shutDownId = -1;
	systemShutDown();
}

void keySosScan(void)
{
    static uint32_t cnt = 0;
    static uint8_t laststatus = 0;//�����ǰ����ɿ��� �����500ms֮�����������ȥ�ͻ��laststatus��1
    static uint8_t once = 0;//0����⵽�ɿ�������  1:�Ѿ��ж�Ϊһ�ְ�����ʽ�� ������ɿ���������ȥ�ǲ����ж��µİ�����ʽ��  ���ڷ�ֹһֱ���ż�����
    static int8_t clicktime ;//���֮��ļ��
    static uint8_t clickcnt = 1;//�������

    if (SYS_PWROFF_DET == KEY_ON)//���ְ��������»��߱����¹�
    {
        cnt++;
        pwkkey.down = 1;
        //�ж�Ϊ����
        if (cnt >= 30 && once == 0)
        {
            pwkkey.longpress = 1;
            once = 1;
            LogMessage(DEBUG_ALL, "Key press");
        }

        if (laststatus && once == 0)
        {
            laststatus = 0;
            clickcnt++;
        }
        //��������
        if (clickcnt == 3 && once == 0)
        {
            once = 1;
            pwkkey.triclick = 1;
            clickcnt = 0;
            
            LogMessage(DEBUG_ALL, "Key Triclick");
        }
        if(cnt >= 30)
        {
            clickcnt = 0;
        }
        clicktime = 30;
    }
    //�ɿ�����
    if (SYS_PWROFF_DET)
    {
    	pwkkey.down = 0;
        clicktime--;
        cnt = 0;
        once = 0;
        if (clicktime >= 0 )
        {
            laststatus = 1;
        }
        else
        {
            laststatus = 0;
            clickcnt = 1;
        }
        if (clicktime <= -100)//��ֹ����ͷ��
        {
            clicktime = -1;
        }
    }
    //LogPrintf(DEBUG_ALL, "soskeycnt=%d", cnt);
}


void keyExcuteByStatus(void)
{
	if (pwkkey.clossprocess)
	{
		pwkkey.triclick = 0;
		return;
	}
	if (pwkkey.triclick)
	{
		pwkkey.clossprocess = 1;
		netRequestSet(NET_REQUEST_TTS_CTL);
		addCmdTTS(TTS_SHUTDOWN);
		if (shutDownId != -1)
		{
			shutDownId = startTimer(300, systemShutDownTimeout, 0);
		}
	}
	pwkkey.triclick = 0;
}

void keyTask(void)
{
	keySosScan();
	keyExcuteByStatus();
	portWdtFeed();
}

