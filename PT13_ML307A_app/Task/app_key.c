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
    static uint8_t laststatus = 0;//作用是按下松开后 如果在500ms之内如果还按下去就会把laststatus置1
    static uint8_t once = 0;//0：检测到松开过按键  1:已经判断为一种按键方式了 如果不松开继续按下去是不会判断新的按键方式的  用于防止一直按着键不放
    static int8_t clicktime ;//点击之间的间隔
    static uint8_t clickcnt = 1;//点击次数

    if (SYS_PWROFF_DET == KEY_ON)//发现按键被按下或者被按下过
    {
        cnt++;
        pwkkey.down = 1;
        //判断为长按
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
        //连按三下
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
    //松开按键
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
        if (clicktime <= -100)//防止减过头了
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

