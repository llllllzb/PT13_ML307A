#include <app_protocol.h>
#include "app_instructioncmd.h"

#include "app_peripheral.h"

#include "app_gps.h"
#include "app_kernal.h"
#include "app_net.h"
#include "app_param.h"
#include "app_sys.h"
#include "app_task.h"
#include "app_server.h"
#include "app_jt808.h"
#include "app_mir3da.h"
#include "app_key.h"
const instruction_s insCmdTable[] =
{
    {PARAM_INS, "PARAM"},
    {STATUS_INS, "STATUS"},
    {VERSION_INS, "VERSION"},
    {SERVER_INS, "SERVER"},
    {HBT_INS, "HBT"},
    {MODE_INS, "MODE"},
    {POSITION_INS, "123"},
    {APN_INS, "APN"},
    {UPS_INS, "UPS"},
    {LOWW_INS, "LOWW"},
    {LED_INS, "LED"},
    {POITYPE_INS, "POITYPE"},
    {RESET_INS, "RESET"},
    {UTC_INS, "UTC"},
    {DEBUG_INS, "DEBUG"},
    {ACCCTLGNSS_INS, "ACCCTLGNSS"},
    {ACCDETMODE_INS, "ACCDETMODE"},
    {FENCE_INS, "FENCE"},
    {FACTORY_INS, "FACTORY"},
    {ICCID_INS, "ICCID"},
    {SETAGPS_INS, "SETAGPS"},
    {JT808SN_INS, "JT808SN"},
    {HIDESERVER_INS, "HIDESERVER"},
    {BLESERVER_INS, "BLESERVER"},
    {SOS_INS, "SOS"},
    {CENTER_INS, "CENTER"},
    {SOSALM_INS, "SOSALM"},
    {TIMER_INS, "TIMER"},
    {BLELINKFAIL_INS, "BLELINKFAIL"},
    {ALARMMODE_INS, "ALARMMODE"},
    {AGPSEN_INS, "AGPSEN"},
    {SETWIFIMAC_INS, "SETWIFIMAC"},
    {SETBLEPSW_INS, "SETBLEPSW"},
    {NONWIFIPARAM_INS, "NONWIFIPARAM"},
    {WITHWIFIPARAM_INS, "WITHWIFIPARAM"},
    {RSWIFIFENCE_INS, "RSWIFIFENCE"},
    {PETLED_INS, "PETLED"},
    {PETSPK_INS, "PETSPK"},
    {UART_INS, "UART"},
    {PETDEBUG_INS, "PETDEBUG"},
    {STEPPARAM_INS, "STEPPARAM"},
    {SYSTEMSHUTDOWN_INS, "SYSTEMSHUTDOWN"},
    {VOLUME_INS, "VOLUME"},
    {SPORTS_INS, "SPORTS"},
    {PETBELL_INS, "PETBELL"},
    {DOWNLOADFILE_INS, "DOWNLOADFILE"},
    {MUSICLIST_INS, "MUSICLIST"},
};

static insMode_e mode123;
static insParam_s param123;
static uint8_t serverType;

/*关于指令延迟回复*/
static insMode_e lastmode;
insParam_s lastparam;
int rspTimeOut = -1;


static void sendMsgWithMode(uint8_t *buf, uint16_t len, insMode_e mode, void *param)
{
    insParam_s *insparam;


    switch (mode)
    {
        case DEBUG_MODE:
            LogMessage(DEBUG_FACTORY, "----------Content----------");
            LogMessage(DEBUG_FACTORY, (char *)buf);
            LogMessage(DEBUG_FACTORY, "------------------------------");
            break;
        case SMS_MODE:
            if (param != NULL)
            {
                insparam = (insParam_s *)param;
                sendMessage(buf, len, insparam->telNum);
                startTimer(15, deleteAllMessage, 0);
            }
            break;
        case NET_MODE:
            if (param != NULL)
            {
                insparam = (insParam_s *)param;
                insparam->data = buf;
                insparam->len = len;
                protocolSend(insparam->link, PROTOCOL_21, insparam);
            }
            break;
        case BLE_MODE:
        	if (param != NULL)
        	{
        	    char debug[156] = { 0 };
        	    uint16_t debuglen = 0;
        		debuglen = len + 3 > 155 ? 155 : len + 3;
        		sprintf(debug, "RE:%s", buf);
        		debug[debuglen] = 0;
	        	insparam = (insParam_s *)param;
	        	LogPrintf(DEBUG_ALL, "%s len:%d", debug, debuglen);
	            appSendNotifyData(insparam->bleConhandle, debug, debuglen);
            }
            break;
        case JT808_MODE:
			jt808MessageSend(buf, len);
        	break;
    }
}

void instructionRespone(char *message)
{
    if (rspTimeOut == -1)
        return;
    char buf[50];
    sprintf(buf, "%s", message);
    setLastInsid();
    sendMsgWithMode((uint8_t *)buf, strlen(buf), lastmode, &lastparam);
    stopTimer(rspTimeOut);
    rspTimeOut = -1;
}

static void doParamInstruction(ITEM *item, char *message)
{
    uint8_t i;
    uint8_t debugMsg[15];
    if (sysparam.protocol == JT808_PROTOCOL_TYPE)
    {
        byteToHexString(dynamicParam.jt808sn, debugMsg, 6);
        debugMsg[12] = 0;
        sprintf(message + strlen(message), "JT808SN:%s;SN:%s;IP:%s:%u;", debugMsg, dynamicParam.SN, sysparam.jt808Server,
                sysparam.jt808Port);
    }
    else
    {
        sprintf(message + strlen(message), "SN:%s;IP:%s:%d;", dynamicParam.SN, sysparam.Server, sysparam.ServerPort);
    }
    sprintf(message + strlen(message), "APN:%s;", sysparam.apn);
    sprintf(message + strlen(message), "UTC:%s%d;", sysparam.utc >= 0 ? "+" : "", sysparam.utc);
    switch (sysparam.MODE)
    {
        case MODE1:
        case MODE21:
            if (sysparam.MODE == MODE1)
            {
                sprintf(message + strlen(message), "Mode1:");

            }
            else
            {
                sprintf(message + strlen(message), "Mode21:");

            }
            for (i = 0; i < 5; i++)
            {
                if (sysparam.AlarmTime[i] != 0xFFFF)
                {
                    sprintf(message + strlen(message), " %.2d:%.2d", sysparam.AlarmTime[i] / 60, sysparam.AlarmTime[i] % 60);
                }

            }
            sprintf(message + strlen(message), ", Every %d day;", sysparam.gapDay);
            break;
        case MODE2:
            if (sysparam.gpsuploadgap != 0)
            {
                if (sysparam.gapMinutes == 0)
                {
                    //检测到震动，n 秒上送
                    sprintf(message + strlen(message), "Mode2: %dS;", sysparam.gpsuploadgap);
                }
                else
                {
                    //检测到震动，n 秒上送，未震动，m分钟自动上送
                    sprintf(message + strlen(message), "Mode2: %dS,%dM;", sysparam.gpsuploadgap, sysparam.gapMinutes);

                }
            }
            else
            {
                if (sysparam.gapMinutes == 0)
                {
                    //保持在线，不上送
                    sprintf(message + strlen(message), "Mode2: online;");
                }
                else
                {
                    //保持在线，不检测震动，每隔m分钟，自动上送
                    sprintf(message + strlen(message), "Mode2: %dM;", sysparam.gapMinutes);
                }
            }
            break;
        case MODE3:
            sprintf(message + strlen(message), "Mode3: %d minutes;", sysparam.gapMinutes);
            break;
        case MODE23:
            sprintf(message + strlen(message), "Mode23: %d minutes;", sysparam.gapMinutes);
            break;
       	case MODE4:
			sprintf(message + strlen(message), "Mode4: gpsgap %dmin, wifi %dmin, %dstep, %dstep;", 
			sysparam.mode4GapMin, sysparam.wifiCheckGapMin_in, sysparam.wifiCheckGapStep_in,sysparam.wifiCheckGapStep_out);
       		break;
    }

    sprintf(message + strlen(message), "StartUp:%u;RunTime:%u;", dynamicParam.startUpCnt, dynamicParam.runTime);

}
static void doStatusInstruction(ITEM *item, char *message)
{
    gpsinfo_s *gpsinfo;
    moduleGetCsq();
    portUpdateStep();
    sprintf(message, "OUT-V=%.2fV;", sysinfo.outsidevoltage);
    //sprintf(message + strlen(message), "BAT-V=%.2fV;", sysinfo.insidevoltage);
    if (sysinfo.gpsOnoff)
    {
        gpsinfo = getCurrentGPSInfo();
        sprintf(message + strlen(message), "GPS=%s;", gpsinfo->fixstatus ? "Fixed" : "Invalid");
        sprintf(message + strlen(message), "PDOP=%.2f;", gpsinfo->pdop);
    }
    else
    {
        sprintf(message + strlen(message), "GPS=Close;");
    }

    sprintf(message + strlen(message), "ACC=%s;", getTerminalAccState() > 0 ? "On" : "Off");
    sprintf(message + strlen(message), "SIGNAL=%d;", getModuleRssi());
    sprintf(message + strlen(message), "BATTERY=%s;", getTerminalChargeState() > 0 ? "Charging" : "Uncharged");
    sprintf(message + strlen(message), "LOGIN=%s;", primaryServerIsReady() > 0 ? "Yes" : "No");
    sprintf(message + strlen(message), "Gsensor=%s;", read_gsensor_id() == 0x13 ? "OK" : "ERR");
    sprintf(message + strlen(message), "RUNNINGTIME=%dmin;", dynamicParam.runningtime / 60);
    sprintf(message + strlen(message), "STEP=%d;", dynamicParam.step);
    if (sysinfo.safeAreaFlag == SAFE_AREA_UNKNOW)
    	sprintf(message + strlen(message), "SAFEAREA=Unknow");
    else if (sysinfo.safeAreaFlag == SAFE_AREA_IN)
    	sprintf(message + strlen(message), "SAFEAREA=In");
    else sprintf(message + strlen(message), "SAFEAREA=Out");
}


static void serverChangeCallBack(void)
{
    jt808ServerReconnect();
    privateServerReconnect();
}

static void doServerInstruction(ITEM *item, char *message)
{
    if (item->item_data[2][0] != 0 && item->item_data[3][0] != 0)
    {
        serverType = atoi(item->item_data[1]);
        if (serverType == JT808_PROTOCOL_TYPE)
        {
            strncpy((char *)sysparam.jt808Server, item->item_data[2], 50);
            stringToLowwer(sysparam.jt808Server, strlen(sysparam.jt808Server));
            sysparam.jt808Port = atoi((const char *)item->item_data[3]);
            sprintf(message, "Update jt808 domain %s:%d;", sysparam.jt808Server, sysparam.jt808Port);

        }
        else
        {
            strncpy((char *)sysparam.Server, item->item_data[2], 50);
            stringToLowwer(sysparam.Server, strlen(sysparam.Server));
            sysparam.ServerPort = atoi((const char *)item->item_data[3]);
            sprintf(message, "Update domain %s:%d;", sysparam.Server, sysparam.ServerPort);
        }
	    if (serverType == JT808_PROTOCOL_TYPE)
	    {
	        sysparam.protocol = JT808_PROTOCOL_TYPE;
	        dynamicParam.jt808isRegister = 0;
	    }
	    else
	    {
	        sysparam.protocol = ZT_PROTOCOL_TYPE;
	    }
	    paramSaveAll();
	    dynamicParamSaveAll();
        startTimer(30, serverChangeCallBack, 0);
    }
    else
    {
        sprintf(message, "Update domain fail,please check your param");
    }

}


static void doVersionInstruction(ITEM *item, char *message)
{
    sprintf(message, "Version:%s;Compile:%s %s;", EEPROM_VERSION, __DATE__, __TIME__);

}
static void doHbtInstruction(ITEM *item, char *message)
{
    if (item->item_data[1][0] == 0 || item->item_data[1][0] == '?')
    {
        sprintf(message, "The time of the heartbeat interval is %d seconds;", sysparam.heartbeatgap);
    }
    else
    {
        sysparam.heartbeatgap = (uint8_t)atoi((const char *)item->item_data[1]);
        paramSaveAll();
        sprintf(message, "Update the time of the heartbeat interval to %d seconds;", sysparam.heartbeatgap);
    }

}
static void doModeInstruction(ITEM *item, char *message)
{
    uint8_t workmode, i, j, timecount = 0, gapday = 1;
    uint16_t mode1time[7];
    uint16_t mode4time;
    uint16_t valueofminute;
    if (item->item_data[1][0] == 0 || item->item_data[1][0] == '?')
    {
    	if (sysparam.MODE == MODE4)
    	{
			sprintf(message, "Mode%d gpsgap %d min, wifi %dmin, %dstep, %dstep;", 
			sysparam.MODE, sysparam.mode4GapMin, sysparam.wifiCheckGapMin_in, sysparam.wifiCheckGapStep_in,sysparam.wifiCheckGapStep_out);
			
    	}
    	else
    	{
        	sprintf(message, "Mode%d,%d,%d", sysparam.MODE, sysparam.gpsuploadgap, sysparam.gapMinutes);
        }
    }
    else
    {
        sysinfo.runStartTick = sysinfo.sysTick;
        workmode = atoi(item->item_data[1]);
        gpsRequestClear(GPS_REQUEST_GPSKEEPOPEN_CTL);
        switch (workmode)
        {
            case 1:
            case 21:
                //内容项如果大于2，说明有时间或日期
                if (item->item_cnt > 2)
                {
                    for (i = 0; i < item->item_cnt - 2; i++)
                    {
                        if (strlen(item->item_data[2 + i]) <= 4 && strlen(item->item_data[2 + i]) >= 3)
                        {
                            valueofminute = atoi(item->item_data[2 + i]);
                            mode1time[timecount++] = valueofminute / 100 * 60 + valueofminute % 100;
                        }
                        else
                        {
                            gapday = atoi(item->item_data[2 + i]);
                        }
                    }

                    for (i = 0; i < (timecount - 1); i++)
                    {
                        for (j = 0; j < (timecount - 1 - i); j++)
                        {
                            if (mode1time[j] > mode1time[j + 1]) //相邻两个元素作比较，如果前面元素大于后面，进行交换
                            {
                                valueofminute = mode1time[j + 1];
                                mode1time[j + 1] = mode1time[j];
                                mode1time[j] = valueofminute;
                            }
                        }
                    }

                }

                for (i = 0; i < 5; i++)
                {
                    sysparam.AlarmTime[i] = 0xFFFF;
                }
                //重现写入AlarmTime
                for (i = 0; i < timecount; i++)
                {
                    sysparam.AlarmTime[i] = mode1time[i];
                }
                if (gapday == 0)
                {
					gapday = 1;
                }
                sysparam.gapDay = gapday;
                if (workmode == 1)
                {
                    terminalAccoff();
                    if (gpsRequestGet(GPS_REQUEST_ACC_CTL))
                    {
                        gpsRequestClear(GPS_REQUEST_ACC_CTL);
                    }
                    sysparam.MODE = MODE1;
                    portGsensorCtl(0);
                }
                else
                {
                    sysparam.MODE = MODE21;
                    portGsensorCtl(0);
                }
                sprintf(message, "Change to Mode%d,and work on at", workmode);
                for (i = 0; i < timecount; i++)
                {
                    sprintf(message + strlen(message), " %.2d:%.2d", sysparam.AlarmTime[i] / 60, sysparam.AlarmTime[i] % 60);
                }
                sprintf(message + strlen(message), ",every %d day;", gapday);
                portSetNextAlarmTime();
                break;
            case 2:
                portGsensorCtl(1);
                sysparam.gpsuploadgap = (uint8_t)atoi((const char *)item->item_data[2]);
                sysparam.gapMinutes = (uint16_t)atoi((const char *)item->item_data[3]);
                sysparam.MODE = MODE2;
                if (sysparam.gpsuploadgap == 0)
                {
                    gpsRequestClear(GPS_REQUEST_ACC_CTL);
                    //运动不自动传GPS
                    if (sysparam.gapMinutes == 0)
                    {

                        sprintf(message, "The device switches to mode 2 without uploading the location");
                    }
                    else
                    {
                        sprintf(message, "The device switches to mode 2 and uploads the position every %d minutes all the time",
                                sysparam.gapMinutes);
                    }
                }
                else
                {
                    if (getTerminalAccState())
                    {
                        if (sysparam.gpsuploadgap < GPS_UPLOAD_GAP_MAX && sysparam.gpsuploadgap != 0)
                        {
                            gpsRequestSet(GPS_REQUEST_ACC_CTL);
                        }
                        else
                        {
                            gpsRequestClear(GPS_REQUEST_ACC_CTL);
                        }
                    }
                    else
                    {
                        gpsRequestClear(GPS_REQUEST_ACC_CTL);
                    }
                    if (sysparam.accctlgnss == 0)
                    {
                        gpsRequestSet(GPS_REQUEST_GPSKEEPOPEN_CTL);
                    }
                    if (sysparam.gapMinutes == 0)
                    {
                        sprintf(message, "The device switches to mode 2 and uploads the position every %d seconds when moving",
                                sysparam.gpsuploadgap);

                    }
                    else
                    {
                        sprintf(message,
                                "The device switches to mode 2 and uploads the position every %d seconds when moving, and every %d minutes when standing still",
                                sysparam.gpsuploadgap, sysparam.gapMinutes);
                    }

                }
                break;
            case 3:
            case 23:
                sysparam.gapMinutes = (uint16_t)atoi(item->item_data[2]);
                if (sysparam.gapMinutes < 5)
                {
                    sysparam.gapMinutes = 5;
                }
				if (sysparam.gapMinutes >= 10080 )
				{
					sysparam.gapMinutes = 10080;
				}

                if (workmode == MODE3)
                {
                    terminalAccoff();
                    if (gpsRequestGet(GPS_REQUEST_ACC_CTL))
                    {
                        gpsRequestClear(GPS_REQUEST_ACC_CTL);
                    }
                    sysparam.MODE = MODE3;
                    portGsensorCtl(0);
                }
                else
                {
                    sysparam.MODE = MODE23;
                    portGsensorCtl(1);
                }
                sprintf(message, "Change to mode %d and update the startup interval time to %d minutes;", workmode,
                        sysparam.gapMinutes);
                break;

            case 4:
				sysparam.MODE = MODE4;
				portGsensorCtl(1);
				if (item->item_data[2][0] != 0)
				{
					sysparam.mode4GapMin = (uint16_t)atoi(item->item_data[2]);
				}
				if (item->item_data[3][0] != 0)
				{
					sysparam.wifiCheckGapMin_in  = (uint16_t)atoi(item->item_data[3]);
					sysparam.wifiCheckGapMin_out = (uint16_t)atoi(item->item_data[3]);
				}
				if (item->item_data[4][0] != 0)
				{
					sysparam.wifiCheckGapStep_in = (uint16_t)atoi(item->item_data[4]);
				}
				if (item->item_data[5][0] != 0)
				{
					sysparam.wifiCheckGapStep_out = (uint16_t)atoi(item->item_data[5]);
				}
				if (sysinfo.outBleFenceFlag == 0)
				{
					terminalAccoff();
	                if (gpsRequestGet(GPS_REQUEST_ACC_CTL))
	                {
	                    gpsRequestClear(GPS_REQUEST_ACC_CTL);
	                }
	                gpsRequestClear(GPS_REQUEST_ALL);
	                lbsRequestClear();
	                wifiRequestClear(DEV_EXTEND_OF_MY | DEV_EXTEND_OF_BLE);
	                agpsRequestClear();
	                startTimer(50, changeMode4Callback, 0);
	           	}
				sprintf(message, "Change to mode %d, and wificheck every %dmin, %dstep, %dstep", workmode, sysparam.wifiCheckGapMin_in, 
					sysparam.wifiCheckGapStep_in, sysparam.wifiCheckGapStep_out);
            	break;
            default:
                strcpy(message, "Unsupport mode");
                break;
        }
        paramSaveAll();
    }
}




void dorequestSend123(void)
{
    char message[150] = {0};
    uint16_t year;
    uint8_t  month;
    uint8_t date;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    gpsinfo_s *gpsinfo;

    portGetRtcDateTime(&year, &month, &date, &hour, &minute, &second);
    gpsinfo = getCurrentGPSInfo();
    sprintf(message, "(%s)<Local Time:%.2d/%.2d/%.2d %.2d:%.2d:%.2d>http://maps.google.com/maps?q=%s%f,%s%f", dynamicParam.SN, \
            year, month, date, hour, minute, second, \
            gpsinfo->NS == 'N' ? "" : "-", gpsinfo->latitude, gpsinfo->EW == 'E' ? "" : "-", gpsinfo->longtitude);
    reCover123InstructionId();
    sendMsgWithMode((uint8_t *)message, strlen(message), mode123, &param123);
}

void do123Instruction(ITEM *item, insMode_e mode, void *param)
{
	char message[150] = {0};
    insParam_s *insparam;
    mode123 = mode;
    if (param != NULL)
    {
        insparam = (insParam_s *)param;
        param123.telNum = insparam->telNum;
        param123.link = insparam->link;
    }
    if (item->item_data[1][0] == 0 || item->item_data[1][0] == '?' || atoi(item->item_data[1]) < 3)
    {

//	    if (sysparam.poitype == 0)
//	    {
//	        lbsRequestSet(DEV_EXTEND_OF_MY);
//	        LogMessage(DEBUG_ALL, "Only LBS reporting");
//	    }
//	    else if (sysparam.poitype == 1)
//	    {
//	        lbsRequestSet(DEV_EXTEND_OF_MY);
//	        gpsRequestSet(GPS_REQUEST_UPLOAD_ONE);
//	        LogMessage(DEBUG_ALL, "LBS and GPS reporting");
//	    }
//	    else
//	    {
	        lbsRequestSet(DEV_EXTEND_OF_MY);
	        wifiRequestSet(DEV_EXTEND_OF_MY);
	        gpsRequestSet(GPS_REQUEST_UPLOAD_ONE);
	        LogMessage(DEBUG_ALL, "LBS ,WIFI and GPS reporting");
//	    }
	    netRequestSet(NET_REQUEST_CONNECT_ONE);
	    sysinfo.flag123 = 1;
	    save123InstructionId();
	    
    }

}


void doAPNInstruction(ITEM *item, char *message)
{
	if (item->item_data[1][0] == 0 || item->item_data[1][0] == '?')
	 {
		 sprintf(message, "APN:%s,APN User:%s,APN Password:%s,APN Authport:%d", sysparam.apn, sysparam.apnuser, sysparam.apnpassword, sysparam.apnAuthPort);
	 }
	 else
	 {
		 sysparam.apn[0] = 0;
		 sysparam.apnuser[0] = 0;
		 sysparam.apnpassword[0] = 0;
		 if (item->item_data[1][0] != 0 && item->item_cnt >= 2)
		 {
			 strcpy((char *)sysparam.apn, item->item_data[1]);
		 }
		 if (item->item_data[2][0] != 0 && item->item_cnt >= 3)
		 {
			 strcpy((char *)sysparam.apnuser, item->item_data[2]);
		 }
		 if (item->item_data[3][0] != 0 && item->item_cnt >= 4)
		 {
			 strcpy((char *)sysparam.apnpassword, item->item_data[3]);
		 }
		 if (item->item_data[4][0] != 0 && item->item_cnt >= 5)
		 {
			 sysparam.apnAuthPort = atoi(item->item_data[4]);
		 }
		 startTimer(50, moduleReset, 0);
		 paramSaveAll();
		 sprintf(message, "Update APN:%s,APN User:%s,APN Password:%s,APN Authport:%d", sysparam.apn, sysparam.apnuser, sysparam.apnpassword, sysparam.apnAuthPort);
	 }


}


void doUPSInstruction(ITEM *item, char *message)
{
    if (item->item_cnt >= 3)
    {
        strncpy((char *)bootparam.updateServer, item->item_data[1], 50);
        bootparam.updatePort = atoi(item->item_data[2]);
        bootParamSaveAll();
    }
    bootParamGetAll();
    sprintf(message, "The device will download the firmware from %s:%d in 8 seconds", bootparam.updateServer,
            bootparam.updatePort);
    bootparam.updateStatus = 1;
    strcpy(bootparam.SN, dynamicParam.SN);
    strcpy(bootparam.apn, sysparam.apn);
    strcpy(bootparam.apnuser, sysparam.apnuser);
    strcpy(bootparam.apnpassword, sysparam.apnpassword);
    strcpy(bootparam.codeVersion, EEPROM_VERSION);
    bootParamSaveAll();
        tmos_start_task(sysinfo.taskId, APP_TASK_CLOSE_MODULE_EVENT, MS1_TO_SYSTEM_TIME(3000));
    tmos_start_task(sysinfo.taskId, APP_TASK_RESET_EVENT, MS1_TO_SYSTEM_TIME(8000));
}

void doLOWWInstruction(ITEM *item, char *message)
{
    if (item->item_data[1][0] == 0 || item->item_data[1][0] == '?')
    {
        sysinfo.lowvoltage = sysparam.lowvoltage / 10.0;
        sprintf(message, "The low voltage param is %.1fV", sysinfo.lowvoltage);

    }
    else
    {
        sysparam.lowvoltage = atoi(item->item_data[1]);
        sysinfo.lowvoltage = sysparam.lowvoltage / 10.0;
        paramSaveAll();
        sprintf(message, "When the voltage is lower than %.1fV, an alarm will be generated", sysinfo.lowvoltage);
    }
}

void doLEDInstruction(ITEM *item, char *message)
{
    if (item->item_data[1][0] == 0 || item->item_data[1][0] == '?')
    {
        sprintf(message, "LED was %s", sysparam.ledctrl ? "open" : "close");

    }
    else
    {
        sysparam.ledctrl = atoi(item->item_data[1]);
        paramSaveAll();
        sprintf(message, "%s", sysparam.ledctrl ? "LED ON" : "LED OFF");
    }
}


void doPOITYPEInstruction(ITEM *item, char *message)
{
    if (item->item_data[1][0] == 0 || item->item_data[1][0] == '?')
    {
        switch (sysparam.poitype)
        {
            case 0:
                strcpy(message, "Current poitype is only LBS reporting");
                break;
            case 1:
                strcpy(message, "Current poitype is LBS and GPS reporting");
                break;
            case 2:
                strcpy(message, "Current poitype is LBS ,WIFI and GPS reporting");
                break;
        }
    }
    else
    {
        if (strstr(item->item_data[1], "AUTO") != NULL)
        {
            sysparam.poitype = 2;
        }
        else
        {
            sysparam.poitype = atoi(item->item_data[1]);
        }
        switch (sysparam.poitype)
        {
            case 0:
                strcpy(message, "Set to only LBS reporting");
                break;
            case 1:
                strcpy(message, "Set to LBS and GPS reporting");
                break;
            case 2:
                strcpy(message, "Set to LBS ,WIFI and GPS reporting");
                break;
            default:
                sysparam.poitype = 2;
                strcpy(message, "Unknow type,default set to LBS ,WIFI and GPS reporting");
                break;
        }
        paramSaveAll();

    }
}

void doResetInstruction(ITEM *item, char *message)
{
    sprintf(message, "System will reset after 8 seconds");
    tmos_start_task(sysinfo.taskId, APP_TASK_CLOSE_MODULE_EVENT, MS1_TO_SYSTEM_TIME(3000));
    tmos_start_task(sysinfo.taskId, APP_TASK_RESET_EVENT, MS1_TO_SYSTEM_TIME(8000));
}

void doUTCInstruction(ITEM *item, char *message)
{
    if (item->item_data[1][0] == 0 || item->item_data[1][0] == '?')
    {
        sprintf(message, "System time zone:UTC %s%d", sysparam.utc >= 0 ? "+" : "", sysparam.utc);
        updateRTCtimeRequest();
    }
    else
    {
        sysparam.utc = atoi(item->item_data[1]);
        updateRTCtimeRequest();
        LogPrintf(DEBUG_ALL, "utc=%d", sysparam.utc);
        if (sysparam.utc < -12 || sysparam.utc > 12)
            sysparam.utc = 8;
        paramSaveAll();
        sprintf(message, "Update the system time zone to UTC %s%d", sysparam.utc >= 0 ? "+" : "", sysparam.utc);
    }
}

void doDebugInstrucion(ITEM *item, char *message)
{
    uint16_t year;
    uint8_t  month;
    uint8_t date;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
	connectionInfoStruct *dev = getBeaconInfoAll();

    portGetRtcDateTime(&year, &month, &date, &hour, &minute, &second);

    sprintf(message, "Time:%.2d/%.2d/%.2d %.2d:%.2d:%.2d;", year, month, date, hour, minute, second);
    sprintf(message + strlen(message), "sysrun:%.2d:%.2d:%.2d;gpsRequest:%02X;gpslast:%.2d:%.2d:%.2d;",
            sysinfo.sysTick / 3600, sysinfo.sysTick % 3600 / 60, sysinfo.sysTick % 60, sysinfo.gpsRequest,
            sysinfo.gpsUpdatetick / 3600, sysinfo.gpsUpdatetick % 3600 / 60, sysinfo.gpsUpdatetick % 60);
    sprintf(message + strlen(message), "netreq:0x%02x;", sysinfo.netRequest);
	sprintf(message + strlen(message), "wifiEvt:0x%02x;", sysinfo.wifiExtendEvt);
	sprintf(message + strlen(message), "alarmReq:0x%02x;", sysinfo.alarmRequest);
	sprintf(message + strlen(message), "outBle:%d;", sysinfo.outBleFenceFlag);
	sprintf(message + strlen(message), "outWifi:%d;", sysinfo.outWifiFenceFlag);
	sprintf(message + strlen(message), "runFsm:%d;", sysinfo.runFsm);
	sprintf(message + strlen(message), "moduleFsm:%d;", getModuleStatus());
	sprintf(message + strlen(message), "ble:%d %d;", dev[0].useflag, dev[1].useflag);
    paramSaveAll();
    dynamicParamSaveAll();
}

void doACCCTLGNSSInstrucion(ITEM *item, char *message)
{
    if (item->item_data[1][0] == 0 || item->item_data[1][0] == '?')
    {
        sprintf(message, "%s", sysparam.accctlgnss ? "GPS is automatically controlled by the program" :
                "The GPS is always be on");
    }
    else
    {
        sysparam.accctlgnss = (uint8_t)atoi((const char *)item->item_data[1]);
        if (sysparam.MODE == MODE2)
        {
            if (sysparam.accctlgnss == 0)
            {
                gpsRequestSet(GPS_REQUEST_GPSKEEPOPEN_CTL);
            }
            else
            {
                gpsRequestClear(GPS_REQUEST_GPSKEEPOPEN_CTL);
            }
        }
        sprintf(message, "%s", sysparam.accctlgnss ? "GPS will be automatically controlled by the program" :
                "The GPS will always be on");
        paramSaveAll();
    }

}


void doAccdetmodeInstruction(ITEM *item, char *message)
{
    if (item->item_data[1][0] == 0 || item->item_data[1][0] == '?')
    {
        switch (sysparam.accdetmode)
        {
            case ACCDETMODE0:
                sprintf(message, "The device is use acc wire to determine whether ACC is ON or OFF.");
                break;
            case ACCDETMODE1:
                sprintf(message, "The device is use acc wire first and voltage second to determine whether ACC is ON or OFF.");
                break;
            case ACCDETMODE2:
                sprintf(message, "The device is use acc wire first and gsensor second to determine whether ACC is ON or OFF.");
                break;
        }

    }
    else
    {
        sysparam.accdetmode = atoi(item->item_data[1]);
        switch (sysparam.accdetmode)
        {
            case ACCDETMODE0:
                sprintf(message, "The device is use acc wire to determine whether ACC is ON or OFF.");
                break;
            case ACCDETMODE1:
                sprintf(message, "The device is use acc wire first and voltage second to determine whether ACC is ON or OFF.");
                break;
            case ACCDETMODE2:
                sprintf(message, "The device is use acc wire first and gsensor second to determine whether ACC is ON or OFF.");
                break;
            default:
                sysparam.accdetmode = ACCDETMODE2;
                sprintf(message,
                        "Unknow mode,Using acc wire first and voltage second to determine whether ACC is ON or OFF by default");
                break;
        }
        paramSaveAll();
    }
}

void doFenceInstrucion(ITEM *item, char *message)
{
    if (item->item_data[1][0] == 0 || item->item_data[1][0] == '?')
    {
        sprintf(message, "The static drift fence is %d meters", sysparam.fence);
    }
    else
    {
        sysparam.fence = atol(item->item_data[1]);
        paramSaveAll();
        sprintf(message, "Set the static drift fence to %d meters", sysparam.fence);
    }

}

void doIccidInstrucion(ITEM *item, char *message)
{
    sendModuleCmd(MCCID_CMD, NULL);
    sprintf(message, "ICCID:%s", getModuleICCID());
}

void doFactoryInstrucion(ITEM *item, char *message)
{
    if (my_strpach(item->item_data[1], "ZTINFO8888"))
    {
        paramDefaultInit(0);
        sprintf(message, "Factory all successfully");
    }
    else
    {
        paramDefaultInit(1);
        sprintf(message, "Factory Settings restored successfully");
		appBondPsdCfg(atoi(sysparam.blePsw));
    }

}
void doSetAgpsInstruction(ITEM *item, char *message)
{
    if (item->item_data[1][0] == 0 || item->item_data[1][0] == '?')
    {
        sprintf(message, "Agps:%s,%d,%s,%s", sysparam.agpsServer, sysparam.agpsPort, sysparam.agpsUser, sysparam.agpsPswd);
    }
    else
    {
        if (item->item_data[1][0] != 0)
        {
            strcpy((char *)sysparam.agpsServer, item->item_data[1]);
            stringToLowwer((char *)sysparam.agpsServer, strlen(sysparam.agpsServer));
        }
        if (item->item_data[2][0] != 0)
        {
            sysparam.agpsPort = atoi(item->item_data[2]);
        }
        if (item->item_data[3][0] != 0)
        {
            strcpy((char *)sysparam.agpsUser, item->item_data[3]);
            stringToLowwer((char *)sysparam.agpsUser, strlen(sysparam.agpsUser));
        }
        if (item->item_data[4][0] != 0)
        {
            strcpy((char *)sysparam.agpsPswd, item->item_data[4]);
            stringToLowwer((char *)sysparam.agpsPswd, strlen(sysparam.agpsPswd));
        }
        paramSaveAll();
        sprintf(message, "Update Agps info:%s,%d,%s,%s", sysparam.agpsServer, sysparam.agpsPort, sysparam.agpsUser,
                sysparam.agpsPswd);
    }

}

static void doJT808SNInstrucion(ITEM *item, char *message)
{
    char senddata[40];
    uint8_t snlen;
    if (item->item_data[1][0] == 0 || item->item_data[1][0] == '?')
    {
        byteToHexString(dynamicParam.jt808sn, (uint8_t *)senddata, 6);
        senddata[12] = 0;
        sprintf(message, "JT808SN:%s", senddata);
    }
    else
    {
        snlen = strlen(item->item_data[1]);
        if (snlen > 12)
        {
            sprintf(message, "SN number too long");
        }
        else
        {
            jt808CreateSn(dynamicParam.jt808sn, (uint8_t *)item->item_data[1], snlen);
            byteToHexString(dynamicParam.jt808sn, (uint8_t *)senddata, 6);
            senddata[12] = 0;
            sprintf(message, "Update SN:%s", senddata);
            dynamicParam.jt808isRegister = 0;
            dynamicParam.jt808AuthLen = 0;
            jt808RegisterLoginInfo(dynamicParam.jt808sn, dynamicParam.jt808isRegister, dynamicParam.jt808AuthCode, dynamicParam.jt808AuthLen);
            dynamicParamSaveAll();
        }
    }
}

static void doHideServerInstruction(ITEM *item, char *message)
{
    if (item->item_data[1][0] == 0 || item->item_data[1][0] == '?')
    {
        sprintf(message, "hidden server %s:%d was %s", sysparam.hiddenServer, sysparam.hiddenPort,
                sysparam.hiddenServOnoff ? "on" : "off");
    }
    else
    {
        if (item->item_data[1][0] == '1')
        {
            sysparam.hiddenServOnoff = 1;
            if (item->item_data[2][0] != 0 && item->item_data[3][0] != 0)
            {
                strncpy((char *)sysparam.hiddenServer, item->item_data[2], 50);
                stringToLowwer(sysparam.hiddenServer, strlen(sysparam.hiddenServer));
                sysparam.hiddenPort = atoi((const char *)item->item_data[3]);
                sprintf(message, "Update hidden server %s:%d and enable it", sysparam.hiddenServer, sysparam.hiddenPort);
            }
            else
            {
                strcpy(message, "please enter your param");
            }
        }
        else
        {
            sysparam.hiddenServOnoff = 0;
            strcpy(message, "Disable hidden server");
        }
        paramSaveAll();
    }
}


static void doBleServerInstruction(ITEM *item, char *message)
{
    if (item->item_data[1][0] == 0 || item->item_data[1][0] == '?')
    {
        sprintf(message, "ble server was %s:%d", sysparam.bleServer, sysparam.bleServerPort);
    }
    else
    {
        if (item->item_data[1][0] != 0 && item->item_data[2][0] != 0)
        {
            strncpy((char *)sysparam.bleServer, item->item_data[1], 50);
            stringToLowwer(sysparam.bleServer, strlen(sysparam.bleServer));
            sysparam.bleServerPort = atoi((const char *)item->item_data[2]);
            sprintf(message, "Update ble server %s:%d", sysparam.bleServer, sysparam.bleServerPort);
            paramSaveAll();
        }
        else
        {
            strcpy(message, "please enter your param");
        }

    }
}


void doTimerInstrucion(ITEM *item, char *message)
{
    if (item->item_data[1][0] == 0 || item->item_data[1][0] == '?')
    {
        sprintf(message, "Current timer is %d", sysparam.gpsuploadgap);
    }
    else
    {
        sysparam.gpsuploadgap = (uint16_t)atoi((const char *)item->item_data[1]);
        sprintf(message, "Update timer to %d", sysparam.gpsuploadgap);
        if (sysparam.MODE == MODE2 || sysparam.MODE == MODE21 || sysparam.MODE == MODE23)
        {
            if (sysparam.gpsuploadgap == 0)
            {
                gpsRequestClear(GPS_REQUEST_ACC_CTL);
            }
            else
            {
                if (getTerminalAccState())
                {
                    if (sysparam.gpsuploadgap < GPS_UPLOAD_GAP_MAX)
                    {
                        gpsRequestSet(GPS_REQUEST_ACC_CTL);
                    }
                    else
                    {
                        gpsRequestClear(GPS_REQUEST_ACC_CTL);
                    }
                }
                else
                {
                    gpsRequestClear(GPS_REQUEST_ACC_CTL);
                }
            }
        }
        paramSaveAll();
    }
}

void doSOSInstruction(ITEM *item, char *messages, insMode_e mode, void *param)
{
//    uint8_t i, j, k;
//    insParam_s *insparam;
//
//    if (item->item_data[1][0] == 0 || item->item_data[1][0] == '?')
//    {
//        strcpy(messages, "Query sos number:");
//        for (i = 0; i < 3; i++)
//        {
//            sprintf(messages + strlen(messages), " %s", sysparam.sosNum[i][0] == 0 ? "NULL" : (char *)sysparam.sosNum[i]);
//        }
//    }
//    else
//    {
//
//        if (mode == SMS_MODE && param != NULL)
//        {
//            insparam = (insParam_s *)param;
//            if (my_strstr(insparam->telNum, (char *)sysparam.centerNum, strlen(insparam->telNum)) == 0)
//            {
//                strcpy(messages, "Operation failure,please use center number!");
//                return ;
//            }
//        }
//        if (item->item_data[1][0] == 'A' || item->item_data[1][0] == 'a')
//        {
//            for (i = 0; i < 3; i++)
//            {
//                for (j = 0; j < 19; j++)
//                {
//                    sysparam.sosNum[i][j] = item->item_data[2 + i][j];
//                }
//                sysparam.sosNum[i][19] = 0;
//            }
//            strcpy(messages, "Add sos number:");
//            for (i = 0; i < 3; i++)
//            {
//                sprintf(messages + strlen(messages), " %s", sysparam.sosNum[i][0] == 0 ? "NULL" : (char *)sysparam.sosNum[i]);
//            }
//        }
//        else if (item->item_data[1][0] == 'D' || item->item_data[1][0] == 'd')
//        {
//            j = strlen(item->item_data[2]);
//            if (j < 20 && j > 0)
//            {
//                k = 0;
//                for (i = 0; i < 3; i++)
//                {
//                    if (my_strpach((char *)sysparam.sosNum[i], item->item_data[2]))
//                    {
//                        sprintf(messages + strlen(messages), "Delete %s OK;", (char *)sysparam.sosNum[i]);
//                        sysparam.sosNum[i][0] = 0;
//                        k = 1;
//                    }
//                }
//                if (k == 0)
//                {
//                    sprintf(messages, "Delete %s Fail,no this number", item->item_data[2]);
//                }
//            }
//            else
//            {
//                strcpy(messages, "Invalid sos number");
//            }
//        }
//        else
//        {
//            strcpy(messages, "Invalid option");
//        }
//        paramSaveAll();
//    }
}

void doCenterInstruction(ITEM *item, char *messages, insMode_e mode, void *param)
{
//    uint8_t j, k;
//    insParam_s *insparam;
//    if (item->item_data[1][0] == 0 || item->item_data[1][0] == '?')
//    {
//        strcpy(messages, "Query center number:");
//        sprintf(messages + strlen(messages), " %s", sysparam.centerNum[0] == 0 ? "NULL" : (char *)sysparam.centerNum);
//    }
//    else
//    {
//        if (mode == SMS_MODE && param != NULL)
//        {
//            insparam = (insParam_s *)param;
//            if (my_strstr(insparam->telNum, (char *)sysparam.centerNum, strlen(insparam->telNum)) == 0)
//            {
//                strcpy(messages, "Operation failure,please use center number!");
//                return;
//            }
//        }
//        if (item->item_data[1][0] == 'A' || item->item_data[1][0] == 'a')
//        {
//            for (j = 0; j < 19; j++)
//            {
//                sysparam.centerNum[j] = item->item_data[2][j];
//            }
//            sysparam.centerNum[19] = 0;
//            strcpy(messages, "Add center number:");
//            sprintf(messages + strlen(messages), " %s", sysparam.centerNum[0] == 0 ? "NULL" : (char *)sysparam.centerNum);
//        }
//        else if (item->item_data[1][0] == 'D' || item->item_data[1][0] == 'd')
//        {
//            j = strlen(item->item_data[2]);
//            if (j < 20 && j > 0)
//            {
//                k = 0;
//                if (my_strpach((char *)sysparam.centerNum, item->item_data[2]))
//                {
//                    sprintf(messages + strlen(messages), "Delete %s OK;", (char *)sysparam.centerNum);
//                    sysparam.centerNum[0] = 0;
//                    k = 1;
//                }
//                if (k == 0)
//                {
//                    sprintf(messages, "Delete %s Fail,no this number", item->item_data[2]);
//                }
//            }
//            else
//            {
//                strcpy(messages, "Invalid center number");
//            }
//        }
//        else
//        {
//            strcpy(messages, "Invalid option");
//        }
//        paramSaveAll();
//    }
}

void doSosAlmInstruction(ITEM *item, char *message)
{
    if (item->item_data[1][0] == 0 || item->item_data[1][0] == '?')
    {
        switch (sysparam.sosalm)
        {
            case ALARM_TYPE_NONE:
                sprintf(message, "Query:%s", "SOS alarm was closed");
                break;
            case ALARM_TYPE_GPRS:
                sprintf(message, "Query:the device will %s when sos occur", "only send gprs");
                break;
            case ALARM_TYPE_GPRS_SMS:
                sprintf(message, "Query:the device will %s when sos occur", "send gprs and sms");
                break;
            case ALARM_TYPE_GPRS_SMS_TEL:
                sprintf(message, "Query:the device will %s when sos occur", "send gprs,sms and call sos number");
                break;
            default:
                strcpy(message, "Unknow");
                break;
        }
    }
    else
    {
        sysparam.sosalm = atoi(item->item_data[1]);
        switch (sysparam.sosalm)
        {
            case ALARM_TYPE_NONE:
                strcpy(message, "Close sos function");
                break;
            case ALARM_TYPE_GPRS:
                sprintf(message, "The device will %s when sos occur", "only send gprs");
                break;
            case ALARM_TYPE_GPRS_SMS:
                sprintf(message, "The device will %s when sos occur", "send gprs and sms");
                break;
            case ALARM_TYPE_GPRS_SMS_TEL:
                sprintf(message, "The device will %s when sos occur", "send gprs,sms and call sos number");
                break;
            default:
                sprintf(message, "%s", "Unknow status,change to send gprs by default");
                sysparam.sosalm = ALARM_TYPE_GPRS;
                break;
        }
        paramSaveAll();
    }
}


static void doBlelinkfailInstruction(ITEM *item, char *message)
{
    if (item->item_data[1][0] == 0 || item->item_data[1][0] == '?')
    {
        sprintf(message + strlen(message), "Ble fail count:%u", sysparam.bleLinkFailCnt);
    }
    else
    {
        sysparam.bleLinkFailCnt = atoi(item->item_data[1]);
        paramSaveAll();
        sprintf(message + strlen(message), "Set ble fail count %d OK", sysparam.bleLinkFailCnt);
    }
}

static void doAlarmModeInstrucion(ITEM *item, char *message)
{
    if (item->item_data[1][0] == 0 || item->item_data[1][0] == '?')
    {
        sprintf(message, "The light-sensing alarm function was %s", sysparam.ldrEn ? "enable" : "disable");
    }
    else
    {
        if (my_strpach(item->item_data[1], "L1"))
        {
            sysparam.ldrEn = 1;
            sprintf(message, "Enables the light-sensing alarm function successfully");

        }
        else if (my_strpach(item->item_data[1], "L0"))
        {
            sysparam.ldrEn = 0;
            sprintf(message, "Disable the light-sensing alarm function successfully");

        }
        else
        {
            sysparam.ldrEn = 1;
            sprintf(message, "Unknow cmd,enable the light-sensing alarm function by default");

        }
        paramSaveAll();

    }
}


static void doAgpsenInstrution(ITEM *item, char *message)
{
	if (item->item_data[1][0] == 0 || item->item_data[1][0] == '?')
	{
		sprintf(message, "Agpsen is %s", sysparam.agpsen ? "On" : "Off");
	}
	else
	{
		sysparam.agpsen = atoi(item->item_data[1]);
		sprintf(message, "Agpsen is %s", sysparam.agpsen ? "On" : "Off");
		paramSaveAll();
	}
}

static void doSetWIFIMacInstrution(ITEM *item, char *message)
{
	uint8_t i, j, k, cnt;
	char wifimac[20] = {0};
	if (item->item_data[1][0] == 0 || item->item_data[1][0] == '?')
	{
		strcpy(message, "Wifi list:");
		for (i = 0; i < sizeof(sysparam.wifiList) / sizeof(sysparam.wifiList[0]); i++)
		{
			byteToHexString(sysparam.wifiList[i], (uint8_t *)wifimac, 12);
			wifimac[12] = 0;
			sprintf(message + strlen(message), " %s;", wifimac);
		}
	}
	else
	{
		if (strlen(item->item_data[1]) == 12)
		{
			tmos_memset(sysparam.wifiList, 0, sizeof(sysparam.wifiList));
			strcpy(message, "Update wifi list:");
		}
		for (i = 0; i < sizeof(sysparam.wifiList) / sizeof(sysparam.wifiList[0]); i++)
		{
			if (strlen(item->item_data[i]) != 12)
            {
                continue;
            }
			k = 0;
			for (j = 0; j < 12; j++)
			{
				if (j % 2 == 0 && j > 0)
				{
					wifimac[k++] = ':';
				}
				tmos_memcpy(wifimac + k, item->item_data[i] + j, 1);
				k++;
			}
			wifimac[k] = 0;
			sprintf(message + strlen(message), " %s;", wifimac);
			changeHexStringToByteArray(sysparam.wifiList[i - 1], item->item_data[i], 12);
			/*插入wifi列表*/
			cnt++;
		}
		if (cnt == 0)
		{
			tmos_memset(sysparam.wifiList, 0, sizeof(sysparam.wifiList));
			strcpy(message, "Disable wifi fence and clear the wifi list");
		}
		paramSaveAll();
	}
}

//static void doSetWIFIMacInstrution(ITEM *item, char *message)
//{
//	uint8_t i, j, k, cnt;
//	char wifimac[20] = {0};
//	if (item->item_data[1][0] == 0 || item->item_data[1][0] == '?')
//	{
//		if (sysparam.wifiCheckGapMin != 0)
//		{
//			strcpy(message, "Wifi list:");
//			for (i = 0; i < sizeof(sysparam.wifiList) / sizeof(sysparam.wifiList[0]); i++)
//			{
//				byteToHexString(sysparam.wifiList[i], (uint8_t *)wifimac, 12);
//				wifimac[12] = 0;
//				sprintf(message + strlen(message), " %s;", wifimac);
//			}
//			sprintf(message + strlen(message), " and wifi check gap is %d min", sysparam.wifiCheckGapMin);
//		}
//		else
//		{
//			strcpy(message, "Wifi fence is disable");
//		}
//	}
//	else
//	{
//		if (strlen(item->item_data[1]) == 12)
//		{
//			tmos_memset(sysparam.wifiList, 0, sizeof(sysparam.wifiList));
//			strcpy(message, "Update wifi list:");
//		}		
//		for (i = 0; i < sizeof(sysparam.wifiList) / sizeof(sysparam.wifiList[0]); i++)
//		{
//			if (strlen(item->item_data[i]) != 12)
//            {
//                continue;
//            }
//			k = 0;
//			for (j = 0; j < 12; j++)
//			{
//				if (j % 2 == 0 && j > 0)
//				{
//					wifimac[k++] = ':';
//				}
//				tmos_memcpy(wifimac + k, item->item_data[i] + j, 1);
//				k++;
//			}
//			wifimac[k] = 0;
//			sprintf(message + strlen(message), " %s;", wifimac);
//			changeHexStringToByteArray(sysparam.wifiList[i - 1], item->item_data[i], 12);
//			/*插入wifi列表*/
//			cnt++;
//		}
//		if (item->item_cnt >= 2)
//		{
//			if (strlen(item->item_data[item->item_cnt - 1]) > 0 && strlen(item->item_data[item->item_cnt - 1]) < 6)
//			{
//				sysparam.wifiCheckGapMin = (uint16_t)atoi(item->item_data[item->item_cnt - 1]);
//				sprintf(message + strlen(message), "Update wifi check gap to %d min", sysparam.wifiCheckGapMin);
//			}
//		}
//		if (cnt == 0 && item->item_cnt == 2 && sysparam.wifiCheckGapMin == 0)
//		{
//			tmos_memset(sysparam.wifiList, 0, sizeof(sysparam.wifiList));
//			strcpy(message, "Disable wifi fence and clear the wifi list");
//		}
//		paramSaveAll();
//	}
//}

static void doSetBlePswInstruction(ITEM *item, char *message)
{
	if (item->item_data[1][0] == 0 || item->item_data[1][0] == '?')
	{
		sprintf(message, "Ble password is %s", sysparam.blePsw);
	}
	else
	{
		if (strlen(item->item_data[1]) != 6)
		{
			strcpy(message, "Please enter right password");
			return;
		}
		memcpy(sysparam.blePsw, item->item_data[1], 6);
		sysparam.blePsw[6] = 0;
		sprintf(message, "Update ble password %s", sysparam.blePsw);
		paramSaveAll();
		appBondPsdCfg(atoi(sysparam.blePsw));
	}
}

static void doNonWifiParamInstruction(ITEM *item, char *message)
{
	if (item->item_data[1][0] == 0 || item->item_data[1][0] == '?')
	{
		sprintf(message, "Non wifi param is %dMin, %dStep", sysparam.wifiCheckGapMin_out, sysparam.wifiCheckGapStep_out);
	}
	else
	{
		if (item->item_data[1][0] != 0)
		{
			sysparam.wifiCheckGapMin_out = (uint16_t)atoi(item->item_data[1]);
		}
		if (item->item_data[2][0] != 0)
		{
			sysparam.wifiCheckGapStep_out = (uint16_t)atoi(item->item_data[2]);
		}
		sprintf(message, "Update non wifi param to %dMin, %dStep", sysparam.wifiCheckGapMin_out, sysparam.wifiCheckGapStep_out);
		paramSaveAll();
	}
}

static void doWithWifiParamInstruction(ITEM *item, char *message)
{
	if (item->item_data[1][0] == 0 || item->item_data[1][0] == '?')
	{
		sprintf(message, "With wifi param is %dMin, %dStep", sysparam.wifiCheckGapMin_in, sysparam.wifiCheckGapStep_in);
	}
	else
	{
		if (item->item_data[1][0] != 0)
		{
			sysparam.wifiCheckGapMin_in = (uint16_t)atoi(item->item_data[1]);
		}
		if (item->item_data[2][0] != 0)
		{
			sysparam.wifiCheckGapStep_in = (uint16_t)atoi(item->item_data[2]);
		}
		sprintf(message, "Update with wifi param to %dMin, %dStep", sysparam.wifiCheckGapMin_in, sysparam.wifiCheckGapStep_in);
		paramSaveAll();
	}
}

static void doRsWifiFenceInstruction(ITEM *item, char *message)
{
	resetSafeArea();
	strcpy(message, "Succeeded in entering the security zone");
}

static void doPetLedInstruction(ITEM *item, char *message)
{
	if (item->item_data[1][0] == 0 || item->item_data[1][0] == '?')
	{
		sprintf(message, "Pet led is %s", sysinfo.petledOnoff ? "On" : "Off");
	}
	else
	{
		if (atoi(item->item_data[1]) == 0)
		{
			sysinfo.petledOnoff = 0;
			ledSetPeriod(PETLED1, 0, 1);
		}
		else
		{
			sysinfo.petledOnoff = 1;
			ledSetPeriod(PETLED1, 1, 1);
		}
		sprintf(message, "%s the pet led", sysinfo.petledOnoff ? "Open" : "Close");
	}
}

static void doPetSpkInstruction(ITEM *item, char *message)
{
	if (item->item_data[1][0] == 0 || item->item_data[1][0] == '?')
	{
		sprintf(message, "Pet speaker content %s", sysinfo.ttsContent);
	}
	else
	{
		if (sysinfo.petBellOnoff)
		{
			strcpy(message, "The bell is playing, please try again later");
		}
		else
		{
			if (item->item_data[1][0] != 0)
			{
				sysinfo.petSpkCnt = atoi(item->item_data[1]);
			}
			if (item->item_data[2][0] != 0)
			{
				sysinfo.petSpkGap = atoi(item->item_data[2]);
			}
			if (sysinfo.petSpkCnt == 0)
			{
				sysinfo.petSpkCnt = 1;
			}
			if (sysinfo.petSpkGap == 0)
			{
				sysinfo.petSpkGap = 5;
			}
			tmos_memset(sysinfo.ttsContent, 0, 60);
			sprintf(message, "Open the pet speaker %d times every %d seconds", sysinfo.petSpkCnt, sysinfo.petSpkGap);

			uint8_t len = strlen(item->item_data[3]) > 60 ? 60 : strlen(item->item_data[3]);
			changeHexStringToByteArray(sysinfo.ttsContent, item->item_data[3], len / 2);
			sysinfo.ttsContent[len / 2] = 0;
			sysinfo.ttsContentLen = len / 2;
			LogPrintf(DEBUG_ALL, "Cnt :%d Gap:%d buff:%s  %s,len:%d %d", sysinfo.petSpkCnt, sysinfo.petSpkGap, sysinfo.ttsContent, item->item_data[3], len, strlen(item->item_data[3]));
			netRequestSet(NET_REQUEST_TTS_CTL);
			sprintf(message + strlen(message), " and play content is[%s] ,len is %d", sysinfo.ttsContent, sysinfo.ttsContentLen);
		}
	}	
}

static void doUartInstruction(ITEM *item, char *message)
{
	if (atoi(item->item_data[1]) == 0)
	{
		portUartCfg(APPUSART2, 0, 115200, NULL);
		sysinfo.logLevel = 0;
		portSyspwkOffGpioCfg();
		strcpy(message, "Close uart 2");
	}
	else
	{
		portUartCfg(APPUSART2, 1, 115200, doDebugRecvPoll);
		strcpy(message, "Open uart 2");
	}
}

static void doPetDebugInstruction(ITEM *item, char *message)
{
	connectionInfoStruct *devInfo;
	devInfo = getBeaconInfoAll();
	sprintf(message, "dev[0]socksuccess:%d dev[1]socksuccess:%d ", devInfo[0].sockFlag, devInfo[1].sockFlag);
	sprintf(message + strlen(message), "Master sn:%s", sysinfo.masterSn);
	sprintf(message + strlen(message), "dev[0]upTick:%d dev[1]Uptick:%d ", devInfo[0].updateTick, devInfo[1].updateTick);
}

static void doStepParamInstruction(ITEM *item, char *message)
{
    if (item->item_data[1][0] == 0 || item->item_data[1][0] == '?')
    {
        sprintf(message, "Gsensor param range is 0x%x, step fliter is 0x%x, sm threshold is 0x%x", sysparam.range, sysparam.stepFliter, sysparam.smThrd);
    }
    else
    {
    	portGsensorCtl(0);
		if (item->item_data[1][0] != 0)
		{
			sysparam.range = strtol(item->item_data[1], NULL, 16);
		}
		if (item->item_data[2][0] != 0)
		{
			sysparam.stepFliter = strtol(item->item_data[2], NULL, 16);
		}
		if (item->item_data[3][0] != 0)
		{
			sysparam.smThrd = strtol(item->item_data[3], NULL, 16);
		}
		portGsensorCtl(1);
        sprintf(message, "Update param range to 0x%x, step fliter to 0x%x, sm threshold is 0x%x", sysparam.range, sysparam.stepFliter, sysparam.smThrd);
        paramSaveAll();
    }
}

static void doSystemShutDownInstruction(ITEM *item, char *message)
{
	systemShutDownCallback();
	strcpy(message, "systemShutDownCallback");
}

static void doVolumeInstruction(ITEM *item, char *message)
{
    if (item->item_data[1][0] == 0 || item->item_data[1][0] == '?')
    {
        sprintf(message, "volume is %d", sysparam.volume);
    }
    else
    {
        if (item->item_data[1][0] != 0)
        {
            sysparam.volume = (uint8_t)atoi(item->item_data[1]);
            if (sysparam.volume > 15)
            {
                sysparam.volume = 15;
            }
            ttsVolumeCfg(sysparam.volume);
        }
        sprintf(message, "Update volume to %d", sysparam.volume);
        paramSaveAll();
    }
}


static void doSportsInstruction(ITEM *item, char *message)
{
    sysinfo.mode123RunTick = 0;
	sysinfo.mode123Min = atoi(item->item_data[1]);
	sysinfo.mode123GpsFre = atoi(item->item_data[2]);
	if (sysinfo.mode123Min == 0)
	{
		sysinfo.mode123Min = 3;
	}
	if (sysinfo.mode123GpsFre == 0)
	{
		sysinfo.mode123GpsFre = 10;
	}
	lbsRequestSet(DEV_EXTEND_OF_MY);
	gpsRequestSet(GPS_REQUEST_123_CTL | GPS_REQUEST_UPLOAD_ONE);
	netRequestSet(NET_REQUEST_CONNECT_ONE);
	sprintf(message, "Device gps work %d min, and acquisition positon every %d seconds", sysinfo.mode123Min, sysinfo.mode123GpsFre);
}

static void doPetbellInstruction(ITEM *item, char *message)
{
	if (item->item_data[1][0] == 0 || item->item_data[1][0] == '?')
	{
		sprintf(message, "Pet bell is %s, music number is %d", sysinfo.petBellOnoff ? "On" : "Off", sysparam.musicNum);
	}
	else
	{
	
		if (sysinfo.petSpkCnt > 0)
		{
			strcpy(message, "Pet spk is playing, try it again later");
		}
		else 
		{
			sysinfo.petBellOnoff = atoi(item->item_data[1]);
			sysparam.musicNum = atoi(item->item_data[2]);
			if (sysparam.musicNum < 3)
			{
				if (sysparam.musicfile[sysparam.musicNum] == 0)
				{
					sprintf(message, "No this music,please download");
				}
				else
				{
					if (sysinfo.petBellOnoff != 0)
					{
						netRequestSet(NET_REQUEST_TTS_CTL);
						sprintf(message, "Enable pet bell playing music[%d] %d times success", sysparam.musicNum, sysinfo.petBellOnoff);
					}
					else 
					{
						netRequestClear(NET_REQUEST_TTS_CTL);
						strcpy(message, "Disable pet bell");
					}
					paramSaveAll();
					
				}
			}
			else
			{
				sprintf(message, "Please enter true music number");
			}			
		}
	}
}

static void downloadFileRspTimeOut(void)
{
	if (rspTimeOut != -1)
	{
		instructionRespone("DownloadFile:Time out");
	}
	rspTimeOut = -1;
}

static uint8_t doDownloadFileInstruction(ITEM *item, char *message)
{
	if (getModuleStatus() != OFFLINE_STATUS && getModuleStatus() != NORMAL_STATUS)
	{
		strcpy(message, "Net is not ok");
		return 1;
	}
	else if (item->item_data[1] == 0)
	{
		strcpy(message, "Please enter http url");
		return 1;
	}
	else if (item->item_data[2] == 0)
	{
		strcpy(message, "Please enter file number");
		return 1;
	}
	else 
	{
		if (my_strstr(item->item_data[1], ".mp3", strlen(item->item_data[1])) || 
			my_strstr(item->item_data[1], ".amr", strlen(item->item_data[1])))
		{
			sysinfo.downloadNum = atoi(item->item_data[2]);
			if (sysinfo.downloadNum >= 3)
				sysinfo.downloadNum = 0;
			downloadAudioToFile(item->item_data[1], sysinfo.downloadNum);
			sprintf(message, "Go to [%s] to download file in number %d music file", item->item_data[1], sysinfo.downloadNum);
			if (rspTimeOut == -1)
	        {
				rspTimeOut = startTimer(300, downloadFileRspTimeOut, 0);
	        }
			return 0;
		}
		else
		{
			strcpy(message, "Http url is too long");
			return 1;
		}
	}
	
}

static void doMusicListInstruction(ITEM *item, char *message)
{
	

}

/*--------------------------------------------------------------------------------------*/
static void doinstruction(int16_t cmdid, ITEM *item, insMode_e mode, void *param)
{
    char message[384] = {0};
    message[0] = 0;
    uint8_t ret = 1;
    insParam_s *debugparam;
    switch (cmdid)
    {
        case PARAM_INS:
            doParamInstruction(item, message);
            break;
        case STATUS_INS:
            doStatusInstruction(item, message);
            break;
        case VERSION_INS:
            doVersionInstruction(item, message);
            break;
        case SERVER_INS:
            doServerInstruction(item, message);
            break;
        case HBT_INS:
            doHbtInstruction(item, message);
            break;
        case MODE_INS:
            doModeInstruction(item, message);
            break;
        case POSITION_INS:
            do123Instruction(item, mode, param);
            break;
        case APN_INS:
            doAPNInstruction(item, message);
            break;
        case UPS_INS:
            doUPSInstruction(item, message);
            break;
        case LOWW_INS:
            doLOWWInstruction(item, message);
            break;
        case LED_INS:
            doLEDInstruction(item, message);
            break;
        case POITYPE_INS:
            doPOITYPEInstruction(item, message);
            break;
        case RESET_INS:
            doResetInstruction(item, message);
            break;
        case UTC_INS:
            doUTCInstruction(item, message);
            break;
        case DEBUG_INS:
            doDebugInstrucion(item, message);
            break;
        case ACCCTLGNSS_INS:
            doACCCTLGNSSInstrucion(item, message);
            break;
        case ACCDETMODE_INS:
            doAccdetmodeInstruction(item, message);
            break;
        case FENCE_INS:
            doFenceInstrucion(item, message);
            break;
        case FACTORY_INS:
            doFactoryInstrucion(item, message);
            break;
        case ICCID_INS:
            doIccidInstrucion(item, message);
            break;
        case SETAGPS_INS:
            doSetAgpsInstruction(item, message);
            break;
        case JT808SN_INS:
            doJT808SNInstrucion(item, message);
            break;
        case HIDESERVER_INS:
            doHideServerInstruction(item, message);
            break;
        case TIMER_INS:
            doTimerInstrucion(item, message);
            break;
        case SOS_INS:
            doSOSInstruction(item, message, mode, param);
            break;
        case CENTER_INS:
            doCenterInstruction(item, message, mode, param);
            break;
        case SOSALM_INS:
            doSosAlmInstruction(item, message);
            break;
        case BLELINKFAIL_INS:
			doBlelinkfailInstruction(item, message);
        	break;
        case ALARMMODE_INS:
			doAlarmModeInstrucion(item, message);
        	break;
        case AGPSEN_INS:
        	doAgpsenInstrution(item, message);
        	break;
       	case SETWIFIMAC_INS:
			doSetWIFIMacInstrution(item, message);
       		break;
       	case SETBLEPSW_INS:
			doSetBlePswInstruction(item, message);
       		break;
       	case NONWIFIPARAM_INS:
			doNonWifiParamInstruction(item, message);
       		break;
       	case WITHWIFIPARAM_INS:
			doWithWifiParamInstruction(item, message);
       		break;
		case RSWIFIFENCE_INS:
			doRsWifiFenceInstruction(item, message);
			break;
       	case PETLED_INS:
			doPetLedInstruction(item, message);
       		break;
       	case PETSPK_INS:
			doPetSpkInstruction(item, message);
       		break;
       	case UART_INS:
			doUartInstruction(item, message);
       		break;
       	case STEPPARAM_INS:
       	    doStepParamInstruction(item, message);
       	    break;
       	case PETDEBUG_INS:
			doPetDebugInstruction(item, message);
       		break;
       	case SYSTEMSHUTDOWN_INS:
			doSystemShutDownInstruction(item, message);
       		break;
       	case VOLUME_INS:
       	    doVolumeInstruction(item, message);
       	    break;
       	case SPORTS_INS:
			doSportsInstruction(item, message);
       		break;
       	case PETBELL_INS:
			doPetbellInstruction(item, message);
       		break;
       	case DOWNLOADFILE_INS:
       	    if (param != NULL)
        	{
	           	debugparam = (insParam_s *)param;
	           	lastparam.link = debugparam->link;
	           	lastmode = mode;
           	}
       		getLastInsid();
			ret = doDownloadFileInstruction(item, message);
       		break;
       	case MUSICLIST_INS:
			
       		break;
        default:
            if (mode == SMS_MODE || mode == BLE_MODE)
            {
            	if (mode == SMS_MODE)
            	{
                	deleteAllMessage();
                }
                return ;
            }
            snprintf(message, 50, "Unsupport CMD:%s;", item->item_data[0]);
            break;
    }
    if (cmdid != POSITION_INS && ret)
    {
    	sendMsgWithMode((uint8_t *)message, strlen(message), mode, param);
    }
}

static int16_t getInstructionid(uint8_t *cmdstr)
{
    uint16_t i = 0;
    for (i = 0; i < sizeof(insCmdTable) / sizeof(insCmdTable[0]); i++)
    {
        if (mycmdPatch(insCmdTable[i].cmdstr, cmdstr))
        {
            return insCmdTable[i].cmdid;
        }
    }
    return -1;
}

void instructionParser(uint8_t *str, uint16_t len, insMode_e mode, void *param)
{
    ITEM item;
    int16_t cmdid;
    stringToItem(&item, str, len);
    strToUppper(item.item_data[0], strlen(item.item_data[0]));
    cmdid = getInstructionid((uint8_t *)item.item_data[0]);
    doinstruction(cmdid, &item, mode, param);
}

