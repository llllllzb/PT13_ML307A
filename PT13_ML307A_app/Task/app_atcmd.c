#include "app_protocol.h"
#include "app_atcmd.h"
#include "app_db.h"
#include "app_gps.h"
#include "app_instructioncmd.h"
#include "app_kernal.h"
#include "app_mir3da.h"
#include "app_net.h"
#include "app_param.h"
#include "app_port.h"
#include "app_sys.h"
#include "app_socket.h"
#include "app_server.h"
#include "aes.h"
#include "app_jt808.h"
#include "app_task.h"
#include "app_peripheral.h"
#include "app_bleprotocol.h"

/*
 * 指令集
 */
const CMDTABLE atcmdtable[] =
{
    {AT_DEBUG_CMD, "DEBUG"},
    {AT_SMS_CMD, "SMS"},
    {AT_NMEA_CMD, "NMEA"},
    {AT_ZTSN_CMD, "ZTSN"},
    {AT_IMEI_CMD, "IMEI"},
    {AT_FMPC_NMEA_CMD, "FMPC_NMEA"},
    {AT_FMPC_BAT_CMD, "FMPC_BAT"},
    {AT_FMPC_GSENSOR_CMD, "FMPC_GSENSOR"},
    {AT_FMPC_ACC_CMD, "FMPC_ACC"},
    {AT_FMPC_GSM_CMD, "FMPC_GSM"},
    {AT_FMPC_RELAY_CMD, "FMPC_RELAY"},
    {AT_FMPC_LDR_CMD, "FMPC_LDR"},
    {AT_FMPC_ADCCAL_CMD, "FMPC_ADCCAL"},
    {AT_FMPC_CSQ_CMD, "FMPC_CSQ"},
    {AT_FMPC_IMSI_CMD, "FMPC_IMSI"},
    {AT_FMPC_CHKP_CMD, "FMPC_CHKP"},
    {AT_FMPC_CM_CMD, "FMPC_CM"},
    {AT_FMPC_CMGET_CMD, "FMPC_CMGET"},
    {AT_FMPC_EXTVOL_CMD, "FMPC_EXTVOL"},
};
/**************************************************
@bref		查找指令
@param
@return
@note
**************************************************/

static int16_t getatcmdid(uint8_t *cmdstr)
{
    uint16_t i = 0;
    for (i = 0; i < sizeof(atcmdtable) / sizeof(atcmdtable[0]); i++)
    {
        if (mycmdPatch(cmdstr, (uint8_t *)atcmdtable[i].cmdstr) != 0)
            return atcmdtable[i].cmdid;
    }
    return -1;
}

/**************************************************
@bref		AT^DEBUG 指令解析
@param
@return
@note
**************************************************/

static void doAtdebugCmd(uint8_t *buf, uint16_t len)
{
	uint8_t param[50] = {0};
	unsigned char msg[100];
    int8_t ret;
    ITEM item;
    stringToItem(&item, buf, len);
    strToUppper(item.item_data[0], strlen(item.item_data[0]));

    if (mycmdPatch((uint8_t *)item.item_data[0], (uint8_t *)"SUSPEND"))
    {
        LogMessage(DEBUG_LOW, "Suspend all task");
        systemTaskSuspend(sysinfo.sysTaskId);
    }
    else if (mycmdPatch((uint8_t *)item.item_data[0], (uint8_t *)"RESUME"))
    {
        LogMessage(DEBUG_LOW, "Resume all task");
        systemTaskResume(sysinfo.sysTaskId);
    }
    else if (mycmdPatch((uint8_t *)item.item_data[0], (uint8_t *)"DTRH"))
    {
        WAKEMODULE;
        LogMessage(DEBUG_ALL, "DTR high");
    }
    else if (mycmdPatch((uint8_t *)item.item_data[0], (uint8_t *)"DTRL"))
    {
        SLEEPMODULE;
        LogMessage(DEBUG_ALL, "DTR low");
    }
    else if (mycmdPatch((uint8_t *)item.item_data[0], (uint8_t *)"POWERKEYH"))
    {
        PORT_PWRKEY_H;
        LogMessage(DEBUG_ALL, "Power key hight");
    }
    else if (mycmdPatch((uint8_t *)item.item_data[0], (uint8_t *)"POWERKEYL"))
    {
        PORT_PWRKEY_L;
        LogMessage(DEBUG_ALL, "Power key low");
    }
    else if (mycmdPatch((uint8_t *)item.item_data[0], (uint8_t *)"SUPPLYON"))
    {
        PORT_SUPPLY_ON;
        LogMessage(DEBUG_ALL, "supply on");
    }
    else if (mycmdPatch((uint8_t *)item.item_data[0], (uint8_t *)"SUPPLYOFF"))
    {
        PORT_SUPPLY_OFF;
        LogMessage(DEBUG_ALL, "supply off");
    }
    else if (mycmdPatch((uint8_t *)item.item_data[0], (uint8_t *)"PWRON"))
    {
		SYS_POWER_ON;
    }
    else if (mycmdPatch((uint8_t *)item.item_data[0], (uint8_t *)"PWROFF"))
    {
		SYS_POWER_OFF;
    }
    
    else if (mycmdPatch((uint8_t *)item.item_data[0], (uint8_t *)"GPSCLOSE"))
    {
        sysinfo.gpsRequest = 0;
    }
    else if (mycmdPatch((uint8_t *)item.item_data[0], (uint8_t *)"MODULEOFF"))
    {
        modulePowerOff();
    }
    else if (mycmdPatch((uint8_t *)item.item_data[0], (uint8_t *)"MODULERESET"))
    {
        moduleReset();
    }
    else if (mycmdPatch((uint8_t *)item.item_data[0], (uint8_t *)"MODULEON"))
    {
        modulePowerOn();
    }
    else if (mycmdPatch((uint8_t *)item.item_data[0], (uint8_t *)"SLEEP"))
    {
        portSleepEn();
    }
    else if (mycmdPatch((uint8_t *)item.item_data[0], (uint8_t *)"GSOPEN"))
    {
		portGsensorCtl(1);
    }
    else if (mycmdPatch((uint8_t *)item.item_data[0], (uint8_t *)"GSCLOSE"))
    {
		portGsensorCtl(0);
    }
    else if (mycmdPatch((uint8_t *)item.item_data[0], (uint8_t *)"RSTHIGH"))
    {
		PORT_RSTKEY_H;
    }
    else if (mycmdPatch((uint8_t *)item.item_data[0], (uint8_t *)"RSTLOW"))
    {
		PORT_RSTKEY_L;
    }
    else if (mycmdPatch((uint8_t *)item.item_data[0], (uint8_t *)"LEDON"))
    {
		LED1_ON;
    }
    else if (mycmdPatch((uint8_t *)item.item_data[0], (uint8_t *)"LEDOFF"))
    {
		LED1_OFF;
    }
    else if (mycmdPatch((uint8_t *)item.item_data[0], (uint8_t *)"GPSON"))
    {
		gpsOpen();
    }
    else if (mycmdPatch((uint8_t *)item.item_data[0], (uint8_t *)"GPSOFF"))
    {
		gpsClose();
    }
    else if (mycmdPatch((uint8_t *)item.item_data[0], (uint8_t *)"BONDPSD"))
    {
		appBondPsdCfg(atoi(item.item_data[1]));
    }
    else if (mycmdPatch((uint8_t *)item.item_data[0], (uint8_t *)"SHOWBOND"))
    {
		appShowBondDecv();
    }
    else if (mycmdPatch((uint8_t *)item.item_data[0], (uint8_t *)"TASKID"))
    {
		appBeaconShowTaskId();
    }
    else if (mycmdPatch((uint8_t *)item.item_data[0], (uint8_t *)"BLEPSW"))
    {
    	memcpy(param, item.item_data[1], 6);
    	param[6] = 0;
		bleGeneratePsdProtocol(param);
    }
	else if (mycmdPatch((uint8_t *)item.item_data[0], (uint8_t *)"WIFI"))
    {
		wifiRequestSet(DEV_EXTEND_OF_FENCE);
    }
    else if (mycmdPatch((uint8_t *)item.item_data[0], (uint8_t *)"STEP"))
    {
    	LogPrintf(DEBUG_ALL, "STEP:%d", getStep());
    }
	else if (mycmdPatch((uint8_t *)item.item_data[0], (uint8_t *)"THREE"))
    {
	    s16_m x,y,z;
		mir3da_read_data(&x,&y,&z);
		LogPrintf(DEBUG_ALL, "x:%d y:%d z:%d",x,y,z);
    }
    else if (mycmdPatch((uint8_t *)item.item_data[0], (uint8_t *)"BLE"))
    {
		appPeripheralInit();
    }
    else if (mycmdPatch((uint8_t *)item.item_data[0], (uint8_t *)"SN"))
    {
		memcpy(dynamicParam.SN, item.item_data[1], 15);
		dynamicParam.SN[15] = 0;
		dynamicParamSaveAll();
    }
    else if (mycmdPatch((uint8_t *)item.item_data[0], (uint8_t *)"CHINA"))
    {
    	enc_unicode_to_utf8_one((unsigned long)item.item_data[1], msg, 1);
		LogPrintf(DEBUG_ALL, "%x %x %x %x %x", item.item_data[1][0], msg[0],msg[1],msg[2],msg[3]);
    }
    else if (mycmdPatch((uint8_t *)item.item_data[0], (uint8_t *)"SHUTDOWN"))
    {
		netRequestSet(NET_REQUEST_TTS_CTL);
		addCmdTTS(TTS_SHUTDOWN);

    }
    else
    {
        if (item.item_data[0][0] >= '0' && item.item_data[0][0] <= '9')
        {
            sysinfo.logLevel = item.item_data[0][0] - '0';
            LogPrintf(DEBUG_NONE, "Debug LEVEL:%d OK", sysinfo.logLevel);
        }
    }
}
/**************************************************
@bref		AT^FMPC_ADCCAL 指令解析
@param
@return
@note
**************************************************/

static void atCmdFmpcAdccalParser(insMode_e mode, void *param)
{
    float x;
    char buff[100] = { 0 };
    x = portGetAdcVol(ADC_CHANNEL);
    LogPrintf(DEBUG_ALL, "vCar:%.2f", x);
    sysparam.adccal = 4.0 / x;
    paramSaveAll();
    sprintf(buff, "Update the ADC calibration parameter to %f", sysparam.adccal);
    LogMessage(DEBUG_FACTORY, buff);
    if (param != NULL)
    {
		sendMsgWithMode((uint8_t *)buff, strlen(buff), mode, param);
    }
}

/**************************************************
@bref		AT^NMEA 指令解析
@param
@return
@note
**************************************************/

static void atCmdNmeaParser(uint8_t *buf, uint16_t len, insMode_e mode, void *param)
{
    char buff[200] = { 0 };
    gpsinfo_s *gpsinfo;
    uint8_t i = 0;
    if (my_strstr((char *)buf, "ON", len))
    {
    	gpsinfo = getCurrentGPSInfo();
        hdGpsGsvCtl(1);
        LogMessage(DEBUG_FACTORY, "NMEA ON OK");
        sysinfo.nmeaOutPutCtl = 1;
        gpsRequestSet(GPS_REQUEST_DEBUG);
        if (param != NULL)
        {
			strcpy(buff, "GPS CN:");
			for (i = 0; i < sizeof(gpsinfo->gpsCn); i++)
			{
				if (gpsinfo->gpsCn[i] != 0)
					sprintf(buff + strlen(buff), "%d;", gpsinfo->gpsCn);
			}
			strcpy(buff + strlen(buff), "BD CN:");
			for (i = 0; i < sizeof(gpsinfo->beidouCn); i++)
			{
				if (gpsinfo->beidouCn[i] != 0)
					sprintf(buff + strlen(buff), "%d;", gpsinfo->beidouCn);
			}
			
			sendMsgWithMode((uint8_t *)buff, strlen(buff), mode, param);
        }
    }
    else
    {
        LogMessage(DEBUG_FACTORY, "NMEA OFF OK");
        gpsRequestClear(GPS_REQUEST_ALL);
        sysinfo.nmeaOutPutCtl = 0;
    }
}


/**************************************************
@bref		AT^FMPC_GSENSOR 指令解析
@param
@return
@note
**************************************************/

static void atCmdFMPCgsensorParser(insMode_e mode, void *param)
{
	char buff[100] = { 0 };
	int id;
    id = read_gsensor_id();
    if (param != NULL)
    {
		if (id == -1)
		{
			strcpy(buff, "Read gsensor chip id error");
		}
		else
		{
			sprintf(buff, "GSENSOR Chk OK. ID=0x%X", id);
			sendMsgWithMode((uint8_t *)buff, strlen(buff), mode, param);
		}
    }
}

/**************************************************
@bref		ZTSN 指令
@param
	buf
	len
@return
@note
**************************************************/

static void atCmdZTSNParser(uint8_t *buf, uint16_t len)
{

}


/**************************************************
@bref		IMEI 指令
@param
	buf
	len
@return
@note
**************************************************/

void atCmdIMEIParser(insMode_e mode, void *param)
{
    char buff[100] = { 0 };
    LogPrintf(DEBUG_FACTORY, "ZTINFO:%s:%s:%s", dynamicParam.SN, getModuleIMEI(), EEPROM_VERSION);
    if (param != NULL)
    {
		sprintf(buff, "ZTINFO:%s:%s:%s", dynamicParam.SN, getModuleIMEI(), EEPROM_VERSION);
		sendMsgWithMode((uint8_t *)buff, strlen(buff), mode, param);
    }
}

//unsigned char nmeaCrc(char *str, int len)
//{
//    int i, index, size;
//    unsigned char crc;
//    crc = str[1];
//    index = getCharIndex((uint8_t *)str, len, '*');
//    size = len - index;
//    for (i = 2; i < len - size; i++)
//    {
//        crc ^= str[i];
//    }
//    return crc;
//}

/**************************************************
@bref		FMPC_NMEA 指令
@param
	buf
	len
@return
@note
**************************************************/

static void atCmdFmpcNmeaParser(uint8_t *buf, uint16_t len, insMode_e mode, void *param)
{
    char buff[200] = { 0 };
    gpsinfo_s *gpsinfo;
    uint8_t i = 0;
    if (my_strstr((char *)buf, "ON", len))
    {
    	gpsinfo = getCurrentGPSInfo();
        hdGpsGsvCtl(1);
        LogMessage(DEBUG_FACTORY, "NMEA ON OK");
        sysinfo.nmeaOutPutCtl = 1;
        gpsRequestSet(GPS_REQUEST_DEBUG);
        if (param != NULL)
        {
			strcpy(buff, "GPS CN:");
			for (i = 0; i < sizeof(gpsinfo->gpsCn); i++)
			{
				if (gpsinfo->gpsCn[i] != 0)
					sprintf(buff + strlen(buff), "%d;", gpsinfo->gpsCn);
			}
			strcpy(buff + strlen(buff), "BD CN:");
			for (i = 0; i < sizeof(gpsinfo->beidouCn); i++)
			{
				if (gpsinfo->beidouCn[i] != 0)
					sprintf(buff + strlen(buff), "%d;", gpsinfo->beidouCn);
			}
			
			sendMsgWithMode((uint8_t *)buff, strlen(buff), mode, param);
        }
    }
    else
    {
        LogMessage(DEBUG_FACTORY, "NMEA OFF OK");
        gpsRequestClear(GPS_REQUEST_ALL);
        sysinfo.nmeaOutPutCtl = 0;
    }
}
/**************************************************
@bref		FMPC_BAT 指令
@param
	buf
	len
@return
@note
**************************************************/

static void atCmdFmpcBatParser(insMode_e mode, void *param)
{
	char buff[50] = { 0 };
    queryBatVoltage();
    LogPrintf(DEBUG_FACTORY, "Vbat: %.3fv", sysinfo.insidevoltage);
    if (param != NULL)
    {
		sprintf(buff, "Vbat: %.3fv", sysinfo.insidevoltage);
		sendMsgWithMode((uint8_t *)buff, strlen(buff), mode, param);
    }
}
/**************************************************
@bref		FMPC_ACC 指令
@param
	buf
	len
@return
@note
**************************************************/

static void atCmdFmpcAccParser(void)
{
    LogPrintf(DEBUG_FACTORY, "ACC is %s", ACC_READ == ACC_STATE_ON ? "ON" : "OFF");
}

/**************************************************
@bref		FMPC_GSM  指令
@param
	buf
	len
@return
@note
**************************************************/

static void atCmdFmpcGsmParser(insMode_e mode, void *param)
{
	char buff[50] = { 0 };
    if (isModuleRunNormal())
    {
        strcpy(buff, "GSM SERVICE OK");
    }
    else
    {
        strcpy(buff, "GSM SERVICE FAIL");
    }
    LogMessage(DEBUG_FACTORY, buff);
    if (param != NULL)
    {
		sendMsgWithMode((uint8_t *)buff, strlen(buff), mode, param);
    }
}

/**************************************************
@bref		FMPC_CSQ 指令
@param
	buf
	len
@return
@note
**************************************************/

static void atCmdFmpcCsqParser(insMode_e mode, void *param)
{
	char buff[50] = { 0 };
    sendModuleCmd(CSQ_CMD, NULL);
    sprintf(buff, "+CSQ: %d,99\r\nOK", getModuleRssi());
    LogMessage(DEBUG_FACTORY, buff);
    if (param != NULL)
    {
		sendMsgWithMode((uint8_t *)buff, strlen(buff), mode, param);
    }
}


/**************************************************
@bref		FMPC_IMSI 指令
@param
	buf
	len
@return
@note
**************************************************/

static void atCmdFmpcIMSIParser(insMode_e mode, void *param)
{
	char buff[100] = { 0 };
    sendModuleCmd(CIMI_CMD, NULL);
    sendModuleCmd(MCCID_CMD, NULL);
    sendModuleCmd(CGSN_CMD, "1");
    sprintf(buff, "FMPC_IMSI_RSP OK, IMSI=%s&&%s&&%s", dynamicParam.SN, getModuleIMSI(), getModuleICCID());
	LogMessage(DEBUG_FACTORY, buff);
	if (param != NULL)
    {
		sendMsgWithMode((uint8_t *)buff, strlen(buff), mode, param);
    }
}

/**************************************************
@bref		FMPC_CHKP 指令
@param
	buf
	len
@return
@note
**************************************************/

static void atCmdFmpcChkpParser(insMode_e mode, void *param)
{
	char buff[100] = { 0 };
    sprintf(buff, "+FMPC_CHKP:%s,%s:%d", dynamicParam.SN, sysparam.Server, sysparam.ServerPort);
    LogMessage(DEBUG_FACTORY, buff);
	if (param != NULL)
    {
		sendMsgWithMode((uint8_t *)buff, strlen(buff), mode, param);
    }

}

/**************************************************
@bref		FMPC_CM 指令
@param
	buf
	len
@return
@note
**************************************************/

static void atCmdFmpcCmParser(insMode_e mode, void *param)
{
	char buff[100] = { 0 };
    sysparam.cm = 1;
    paramSaveAll();
    strcpy(buff, "CM OK");
    LogMessage(DEBUG_FACTORY, buff);
	if (param != NULL)
    {
		sendMsgWithMode((uint8_t *)buff, strlen(buff), mode, param);
    }

}
/**************************************************
@bref		FMPC_CMGET 指令
@param
@return
@note
**************************************************/

static void atCmdCmGetParser(insMode_e mode, void *param)
{
	char buff[100] = { 0 };
    if (sysparam.cm == 1)
    {
        strcpy(buff, "CM OK");
    }
    else
    {
        strcpy(buff, "CM FAIL");
    }
    LogMessage(DEBUG_FACTORY, buff);
	if (param != NULL)
    {
		sendMsgWithMode((uint8_t *)buff, strlen(buff), mode, param);
    }
}
/**************************************************
@bref		FMPC_EXTVOL 指令
@param
@return
@note
**************************************************/

static void atCmdFmpcExtvolParser(insMode_e mode, void *param)
{
	char buff[100] = { 0 };
    sprintf(buff, "+FMPC_EXTVOL: %.2f", sysinfo.outsidevoltage);
    LogMessage(DEBUG_FACTORY, buff);
	if (param != NULL)
    {
		sendMsgWithMode((uint8_t *)buff, strlen(buff), mode, param);
    }
}

/**************************************************
@bref		FMPC_EXTVOL 指令
@param
@return
@note
**************************************************/

static void atCmdFMPCLdrParase(void)
{

}

/**************************************************
@bref		AT 指令解析
@param
@return
@note
**************************************************/

void atCmdParserFunction(uint8_t *buf, uint16_t len)
{
    int ret, cmdlen, cmdid;
    char cmdbuf[51];
    //识别AT^
    if (buf[0] == 'A' && buf[1] == 'T' && buf[2] == '^')
    {
        LogMessageWL(DEBUG_ALL, (char *)buf, len);
        ret = getCharIndex(buf, len, '=');
        if (ret < 0)
        {
            ret = getCharIndex(buf, len, '\r');

        }
        if (ret >= 0)
        {
            cmdlen = ret - 3;
            if (cmdlen < 50)
            {
                strncpy(cmdbuf, (const char *)buf + 3, cmdlen);
                cmdbuf[cmdlen] = 0;
                cmdid = getatcmdid((uint8_t *)cmdbuf);
                switch (cmdid)
                {
                    case AT_SMS_CMD:
                        instructionParser(buf + ret + 1, len - ret - 1, DEBUG_MODE, NULL);
                        break;
                    case AT_DEBUG_CMD:
                        doAtdebugCmd(buf + ret + 1, len - ret - 1);
                        break;
                    case AT_NMEA_CMD:
                        atCmdNmeaParser(buf + ret + 1, len - ret - 1, DEBUG_MODE, NULL);
                        break;
                    case AT_ZTSN_CMD:
                        atCmdZTSNParser((uint8_t *)buf + ret + 1, len - ret - 1);
                        break;
                    case AT_IMEI_CMD:
                        atCmdIMEIParser(0, NULL);
                        break;
                    case AT_FMPC_NMEA_CMD :
                        atCmdFmpcNmeaParser((uint8_t *)buf + ret + 1, len - ret - 1, DEBUG_MODE, NULL);
                        break;
                    case AT_FMPC_BAT_CMD :
                        atCmdFmpcBatParser(0, NULL);
                        break;
                    case AT_FMPC_GSENSOR_CMD :
                        atCmdFMPCgsensorParser(0, NULL);
                        break;
                    case AT_FMPC_ACC_CMD :
                        atCmdFmpcAccParser();
                        break;
                    case AT_FMPC_GSM_CMD :
                        atCmdFmpcGsmParser(0, NULL);
                        break;
                    case AT_FMPC_ADCCAL_CMD:
                        atCmdFmpcAdccalParser(0, NULL);
                        break;
                    case AT_FMPC_CSQ_CMD:
                        atCmdFmpcCsqParser(0, NULL);
                        break;
                    case AT_FMPC_IMSI_CMD:
                        atCmdFmpcIMSIParser(0, NULL);
                        break;
                    case AT_FMPC_CHKP_CMD:
                        atCmdFmpcChkpParser(0, NULL);
                        break;
                    case AT_FMPC_CM_CMD:
                        atCmdFmpcCmParser(0, NULL);
                        break;
                    case AT_FMPC_CMGET_CMD:
                        atCmdCmGetParser(0, NULL);
                        break;
                    case AT_FMPC_EXTVOL_CMD:
                        atCmdFmpcExtvolParser(0, NULL);
                        break;
                    case AT_FMPC_LDR_CMD:
						atCmdFMPCLdrParase();
                    	break;
                    default:
                        LogMessage(DEBUG_ALL, "Unknown Cmd");
                        break;
                }
            }
        }
    }
    else
    {
        createNode(buf, len, 0);
    }
}

/**************************************************
@bref		蓝牙通道的AT调试指令解析
@param
@return
@note
**************************************************/

void atCmdBleParserFun(uint8_t *buf, uint16_t len, insMode_e mode, void *param)
{
    int ret, cmdlen, cmdid;
    char cmdbuf[51];

    //识别AT^
    if (buf[0] == 'A' && buf[1] == 'T' && buf[2] == '^')
    {
        LogMessageWL(DEBUG_ALL, (char *)buf, len);
        ret = getCharIndex(buf, len, '=');
        
        if (ret < 0)
        {	
        	//由于是蓝牙发送\r\n不方便 
            //ret = getCharIndex(buf, len, '\r');
            //所以直接取全长
            ret = len;
        }
        if (ret >= 0)
        {
            cmdlen = ret - 3;
            if (cmdlen < 50)
            {
                strncpy(cmdbuf, (const char *)buf + 3, cmdlen);
                cmdbuf[cmdlen] = 0;
                cmdid = getatcmdid((uint8_t *)cmdbuf);
                switch (cmdid)
                {
                    case AT_SMS_CMD:
                        instructionParser(buf + ret + 1, len - ret - 1, mode, (void *)param);
                        break;
                    case AT_DEBUG_CMD:
                        doAtdebugCmd(buf + ret + 1, len - ret - 1);
                        break;
                    case AT_NMEA_CMD:
                        atCmdNmeaParser(buf + ret + 1, len - ret - 1, mode, (void *)param);
                        break;

                    case AT_IMEI_CMD:
                        atCmdIMEIParser(mode, (void *)param);
                        break;
                    case AT_FMPC_NMEA_CMD :
                        atCmdFmpcNmeaParser((uint8_t *)buf + ret + 1, len - ret - 1, mode, (void *)param);
                        break;
                    case AT_FMPC_BAT_CMD :
                        atCmdFmpcBatParser(mode, (void *)param);
                        break;
                    case AT_FMPC_GSENSOR_CMD :
                        atCmdFMPCgsensorParser(mode, (void *)param);
                        break;
                    case AT_FMPC_ACC_CMD :
                        atCmdFmpcAccParser();
                        break;
                    case AT_FMPC_GSM_CMD :
                        atCmdFmpcGsmParser(mode, (void *)param);
                        break;
                    case AT_FMPC_ADCCAL_CMD:
                        atCmdFmpcAdccalParser(mode, (void *)param);
                        break;
                    case AT_FMPC_CSQ_CMD:
                        atCmdFmpcCsqParser(mode, (void *)param);
                        break;
                    case AT_FMPC_IMSI_CMD:
                        atCmdFmpcIMSIParser(mode, (void *)param);
                        break;
                    case AT_FMPC_CHKP_CMD:
                        atCmdFmpcChkpParser(mode, (void *)param);
                        break;
                    case AT_FMPC_CM_CMD:
                        atCmdFmpcCmParser(mode, (void *)param);
                        break;
                    case AT_FMPC_CMGET_CMD:
                        atCmdCmGetParser(mode, (void *)param);
                        break;
                    case AT_FMPC_EXTVOL_CMD:
                        atCmdFmpcExtvolParser(mode, (void *)param);
                        break;
                    case AT_FMPC_LDR_CMD:
						atCmdFMPCLdrParase();
                    	break;
                    default:
                        LogMessage(DEBUG_ALL, "Unknown Cmd");
                        break;
                }
            }
        }
    }
    else
    {
        createNode(buf, len, 0);
    }
}


