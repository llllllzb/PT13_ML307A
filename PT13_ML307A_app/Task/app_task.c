#include <app_protocol.h>
#include "app_task.h"
#include "app_mir3da.h"
#include "app_atcmd.h"
#include "app_gps.h"
#include "app_instructioncmd.h"
#include "app_kernal.h"
#include "app_net.h"
#include "app_net.h"
#include "app_param.h"
#include "app_port.h"
#include "app_sys.h"
#include "app_socket.h"
#include "app_server.h"
#include "app_jt808.h"
#include "app_central.h"
#include "app_peripheral.h"
#include "app_key.h"
#include "app_db.h"
#define SYS_LED1_ON       LED1_ON
#define SYS_LED1_OFF      LED1_OFF


static SystemLEDInfo sysledinfo;
motionInfo_s motionInfo;
static bleScanTry_s bleTry;


/**************************************************
@bref		bit0 置位，布防
@param
@return
@note
**************************************************/
void terminalDefense(void)
{
    sysinfo.terminalStatus |= 0x01;
}

/**************************************************
@bref		bit0 清除，撤防
@param
@return
@note
**************************************************/
void terminalDisarm(void)
{
    sysinfo.terminalStatus &= ~0x01;
}
/**************************************************
@bref		获取运动或静止状态
@param
@return
	>0		运动
	0		静止
@note
**************************************************/

uint8_t getTerminalAccState(void)
{
    return (sysinfo.terminalStatus & 0x02);

}

/**************************************************
@bref		bit1 置位，运动，accon
@param
@return
@note
**************************************************/

void terminalAccon(void)
{
    sysinfo.terminalStatus |= 0x02;
    jt808UpdateStatus(JT808_STATUS_ACC, 1);
}

/**************************************************
@bref		bit1 清除，静止，accoff
@param
@return
@note
**************************************************/
void terminalAccoff(void)
{
    sysinfo.terminalStatus &= ~0x02;
    jt808UpdateStatus(JT808_STATUS_ACC, 0);
}

/**************************************************
@bref		bit2 置位，充电
@param
@return
@note
**************************************************/

void terminalCharge(void)
{
    sysinfo.terminalStatus |= 0x04;
}
/**************************************************
@bref		bit2 清除，未充电
@param
@return
@note
**************************************************/

void terminalunCharge(void)
{
    sysinfo.terminalStatus &= ~0x04;
}

/**************************************************
@bref		获取充电状态
@param
@return
	>0		充电
	0		未充电
@note
**************************************************/

uint8_t getTerminalChargeState(void)
{
    return (sysinfo.terminalStatus & 0x04);
}

/**************************************************
@bref		bit 3~5 报警信息
@param
@return
@note
**************************************************/

void terminalAlarmSet(TERMINAL_WARNNING_TYPE alarm)
{
    sysinfo.terminalStatus &= ~(0x38);
    sysinfo.terminalStatus |= (alarm << 3);
}

/**************************************************
@bref		bit6 置位，已定位
@param
@return
@note
**************************************************/

void terminalGPSFixed(void)
{
    sysinfo.terminalStatus |= 0x40;
}

/**************************************************
@bref		bit6 清除，未定位
@param
@return
@note
**************************************************/

void terminalGPSUnFixed(void)
{
    sysinfo.terminalStatus &= ~0x40;
}

/**************************************************
@bref		LED1 运行任务
@param
@return
@note
**************************************************/

static void sysLed1Run(void)
{
    static uint8_t tick = 0;


    if (sysledinfo.sys_led1_on_time == 0)
    {
        SYS_LED1_OFF;
        return;
    }
    else if (sysledinfo.sys_led1_off_time == 0)
    {
        SYS_LED1_ON;
        return;
    }

    tick++;
    if (sysledinfo.sys_led1_onoff == 1) //on status
    {
        SYS_LED1_ON;
        if (tick >= sysledinfo.sys_led1_on_time)
        {
            tick = 0;
            sysledinfo.sys_led1_onoff = 0;
        }
    }
    else   //off status
    {
        SYS_LED1_OFF;
        if (tick >= sysledinfo.sys_led1_off_time)
        {
            tick = 0;
            sysledinfo.sys_led1_onoff = 1;
        }
    }
}


static void sysLed2Run(void)
{
    static uint8_t tick = 0;


    if (sysledinfo.sys_led2_on_time == 0)
    {
        LED2_OFF;
        return;
    }
    else if (sysledinfo.sys_led2_off_time == 0)
    {
        LED2_ON;
        return;
    }

    tick++;
    if (sysledinfo.sys_led2_onoff == 1) //on status
    {
        LED2_ON;
        if (tick >= sysledinfo.sys_led2_on_time)
        {
            tick = 0;
            sysledinfo.sys_led2_onoff = 0;
        }
    }
    else   //off status
    {
        LED2_OFF;
        if (tick >= sysledinfo.sys_led2_off_time)
        {
            tick = 0;
            sysledinfo.sys_led2_onoff = 1;
        }
    }
}

/**************************************************
@bref		设置灯的闪烁频率
@param
@return
@note
**************************************************/

void ledSetPeriod(uint8_t ledtype, uint8_t on_time, uint8_t off_time)
{
    if (ledtype == GPSLED1)
    {
        //系统信号灯
        sysledinfo.sys_led1_on_time = on_time;
        sysledinfo.sys_led1_off_time = off_time;
    }
    else if (ledtype == PETLED1)
    {
		sysledinfo.sys_led2_on_time = on_time;
        sysledinfo.sys_led2_off_time = off_time;
    }
}

/**************************************************
@bref		更新系统灯状态
@param
@return
@note
**************************************************/

void ledStatusUpdate(uint8_t status, uint8_t onoff)
{
    if (onoff == 1)
    {
        sysinfo.sysLedState |= status;
    }
    else
    {
        sysinfo.sysLedState &= ~status;
    }
    if ((sysinfo.sysLedState & SYSTEM_LEN_IDLE) == SYSTEM_LEN_IDLE)
    {
    	//10s1闪
    	ledSetPeriod(GPSLED1, 10, 90);
	    if ((sysinfo.sysLedState & SYSTEM_LED_RUN) == SYSTEM_LED_RUN)
	    {
	        //慢闪
	        ledSetPeriod(GPSLED1, 10, 10);
	        if ((sysinfo.sysLedState & SYSTEM_LED_NETOK) == SYSTEM_LED_NETOK)
	        {
	            //常亮
	            ledSetPeriod(GPSLED1, 1, 9);
	            if ((sysinfo.sysLedState & SYSTEM_LED_GPSOK) == SYSTEM_LED_GPSOK)
	            {
	                //普通灯常亮
	                ledSetPeriod(GPSLED1, 1, 0);
	            }
	        }

	    }
    }
    else
    {
        SYS_LED1_OFF;
        ledSetPeriod(GPSLED1, 0, 1);
    }
}

/**************************************************
@bref		灯控任务
@param
@return
@note
**************************************************/

static void ledTask(void)
{
	sysLed2Run();
	if (sysparam.ledctrl == 0)
	{
		SYS_LED1_OFF;
		return;
	}

    sysLed1Run();
    
}
/**************************************************
@bref		gps开启请求
@param
@return
@note
**************************************************/
void gpsRequestSet(uint32_t flag)
{
    LogPrintf(DEBUG_ALL, "gpsRequestSet==>0x%04X", flag);
    sysinfo.gpsRequest |= flag;
}

/**************************************************
@bref		gps清除请求
@param
@return
@note
**************************************************/

void gpsRequestClear(uint32_t flag)
{
    LogPrintf(DEBUG_ALL, "gpsRequestClear==>0x%04X", flag);
    sysinfo.gpsRequest &= ~flag;
}

uint32_t gpsRequestGet(uint32_t flag)
{
    return sysinfo.gpsRequest & flag;
}

/**************************************************
@bref		gps任务状态机切换
@param
@return
@note
**************************************************/

static void gpsChangeFsmState(uint8_t state)
{
    sysinfo.gpsFsm = state;
}


/**************************************************
@bref		gps数据接收
@param
@return
@note
**************************************************/

void gpsUartRead(uint8_t *msg, uint16_t len)
{
    static uint8_t gpsRestore[UART_RECV_BUFF_SIZE + 1];
    static uint16_t size = 0;
    uint16_t i, begin;
    if (len + size > UART_RECV_BUFF_SIZE)
    {
        size = 0;
    }
    memcpy(gpsRestore + size, msg, len);
    size += len;
    begin = 0;
    for (i = 0; i < size; i++)
    {
        if (gpsRestore[i] == '\n')
        {
            if (sysinfo.nmeaOutPutCtl)
            {
                LogWL(DEBUG_GPS, gpsRestore + begin, i - begin);
                LogWL(DEBUG_GPS, "\r\n", 2);
                
            }
            nmeaParser(gpsRestore + begin, i - begin);
            begin = i + 1;
        }
    }
    if (begin != 0)
    {
        memmove(gpsRestore, gpsRestore + begin, size - begin);
        size -= begin;
    }
}

static void gpsCfg(void)
{
    char param[50];
	//关闭GSV
    //sprintf(param, "$CCMSG,GSV,1,0,*1A\r\n");
    sprintf(param, "$PCAS03,1,0,1,0,1,0,0,0,0,0,0,0,0,0*03\r\n");
    portUartSend(&usart3_ctl, (uint8_t *)param, strlen(param));
	sprintf(param, "$PCAS03,,,,,,,,,,,1*1F\r\n");
	portUartSend(&usart3_ctl, (uint8_t *)param, strlen(param));

    LogMessage(DEBUG_ALL, "gps config nmea output");
}
/**************************************************
@bref		切换中科微波特率为115200
@param
@return
@note
**************************************************/

//$PCAS03,1,0,1,1,1,0,0,0,0,0,0,0,0,0*02
static void changeGPSBaudRate(void)
{
    char param[50];
    sprintf(param, "$PCAS01,5*19\r\n");
    portUartSend(&usart3_ctl, (uint8_t *)param, strlen(param));
    portUartCfg(APPUSART3, 1, 115200, gpsUartRead);
    LogMessage(DEBUG_ALL, "gps config baudrate to 115200");
    startTimer(10, gpsCfg, 0);
}
/**************************************************
@bref		中科微热启动配置
@param
@return
@note
**************************************************/

static void gpsWarmStart(void)
{
	char param[50];
	//热启动
	sprintf(param, "$PCAS10,0*1C\r\n");
	portUartSend(&usart3_ctl, (uint8_t *)param, strlen(param));
	LogMessage(DEBUG_ALL, "Gps config warm start");
    startTimer(10, changeGPSBaudRate, 0);
}
/**************************************************
@bref		开启gps
@param
@return
@note
**************************************************/

static void gpsOpen(void)
{
	portGpsGpioCfg(1);
	DelayMs(1);
	GPSPWR_ON;
    GPSLNA_ON;
    portUartCfg(APPUSART3, 1, 9600, gpsUartRead);
    startTimer(10, changeGPSBaudRate, 0);
    sysinfo.gpsUpdatetick = sysinfo.sysTick;
    sysinfo.gpsOnoff = 1;
    gpsChangeFsmState(GPSWATISTATUS);
    gpsClearCurrentGPSInfo();
    ledStatusUpdate(SYSTEM_LED_GPSOK, 0);
    moduleSleepCtl(0);
    LogMessage(DEBUG_ALL, "gpsOpen");
	sysinfo.ephemerisFlag = 0;
}
/**************************************************
@bref		等待gps稳定
@param
@return
@note
**************************************************/

static void gpsWait(void)
{
    static uint8_t runTick = 0;
    if (++runTick >= 5)
    {
        runTick = 0;
        gpsChangeFsmState(GPSOPENSTATUS);
        if (sysinfo.ephemerisFlag == 0)
        {
			agpsRequestSet();
            
        }
    }
}

/**************************************************
@bref		关闭gps
@param
@return
@note
**************************************************/

static void gpsClose(void)
{
    GPSPWR_OFF;
    GPSLNA_OFF;
    portUartCfg(APPUSART3, 0, 115200, NULL);
	DelayMs(1);
	portGpsGpioCfg(0);
    sysinfo.rtcUpdate = 0;
    sysinfo.gpsOnoff = 0;
    gpsClearCurrentGPSInfo();
    terminalGPSUnFixed();
    gpsChangeFsmState(GPSCLOSESTATUS);
    ledStatusUpdate(SYSTEM_LED_GPSOK, 0);
	clearLastPoint();
    LogMessage(DEBUG_ALL, "gpsClose");
}


/**************************************************
@bref		保存上一次gps位置
@param
@return
@note
**************************************************/
void saveGpsHistory(void)
{
	gpsinfo_s *gpsinfo;
    float latitude, longtitude;
    gpsinfo = getLastFixedGPSInfo();
    if (gpsinfo->fixstatus != 0)
    {
        latitude = gpsinfo->latitude;
        longtitude = gpsinfo->longtitude;
        if (gpsinfo->NS == 'S')
        {
            if (latitude > 0)
            {
                latitude *= -1;
            }
        }
        if (gpsinfo->EW == 'W')
        {
            if (longtitude > 0)
            {
                longtitude *= -1;
            }
        }
        dynamicParam.saveLat = latitude;
        dynamicParam.saveLon = longtitude;
        LogPrintf(DEBUG_ALL, "Save Latitude:%lf,Longtitude:%lf", dynamicParam.saveLat, dynamicParam.saveLon);
		dynamicParamSaveAll();
    }
}


/**************************************************
@bref		gps控制任务
@param
@return
@note
**************************************************/

static void gpsRequestTask(void)
{
    gpsinfo_s *gpsinfo;

    switch (sysinfo.gpsFsm)
    {
        case GPSCLOSESTATUS:
            //有设备请求开关
            if (sysinfo.canRunFlag != 1)
            {
            	break;
            }
            if (sysinfo.gpsRequest != 0)
            {
                gpsOpen();
            }
            break;
        case GPSWATISTATUS:
            gpsWait();
            break;
        case GPSOPENSTATUS:
            gpsinfo = getCurrentGPSInfo();
            if (gpsinfo->fixstatus)
            {
                ledStatusUpdate(SYSTEM_LED_GPSOK, 1);
                lbsRequestClear();
                wifiRequestClear(DEV_EXTEND_OF_MY | DEV_EXTEND_OF_BLE);
            }
            else
            {
                ledStatusUpdate(SYSTEM_LED_GPSOK, 0);                
            }
            if (sysinfo.gpsRequest == 0 || (sysinfo.sysTick - sysinfo.gpsUpdatetick) >= 20)
            {
            	if (sysinfo.gpsRequest == 0)
            	{
					saveGpsHistory();
					agpsRequestClear();
            	}
                gpsClose();
            }
            break;
        default:
            gpsChangeFsmState(GPSCLOSESTATUS);
            break;
    }
}


/**************************************************
@bref		上送一个gps位置
@param
@return
@note
**************************************************/

static void gpsUplodOnePointTask(void)
{
    gpsinfo_s *gpsinfo;
    static uint16_t runtick = 0;
    static uint8_t uploadtick = 0;
    //判断是否有请求该事件
    if (sysinfo.gpsOnoff == 0)
        return;
    if (gpsRequestGet(GPS_REQUEST_UPLOAD_ONE) == 0)
    {
        runtick = 0;
        uploadtick = 0;
        return;
    }
    gpsinfo = getCurrentGPSInfo();
    runtick++;
    if (gpsinfo->fixstatus == 0)
    {
        uploadtick = 0;
        if (runtick >= 180)
        {
            runtick = 0;
            uploadtick = 0;
            gpsRequestClear(GPS_REQUEST_UPLOAD_ONE);
            if (netRequestGet(NET_REQUEST_CONNECT_ONE) && gpsRequestGet(GPS_REQUEST_123_CTL) == 0)
            {
				netRequestClear(NET_REQUEST_CONNECT_ONE);
            }
        }
        return;
    }
    runtick = 0;
    if (++uploadtick >= 10)
    {
        uploadtick = 0;
        if (sysinfo.flag123)
        {
            dorequestSend123();
            if (sysinfo.mode123Min == 0)
            {
            	//resetSafeArea();
            }
        }
        protocolSend(NORMAL_LINK, PROTOCOL_12, getCurrentGPSInfo());
        jt808SendToServer(TERMINAL_POSITION, getCurrentGPSInfo());
        gpsRequestClear(GPS_REQUEST_UPLOAD_ONE);
		if (netRequestGet(NET_REQUEST_CONNECT_ONE) && gpsRequestGet(GPS_REQUEST_123_CTL) == 0)
		{
			netRequestClear(NET_REQUEST_CONNECT_ONE);
		}

    }

}

/**************************************************
@bref		报警上送请求
@param
@return
@note
**************************************************/
void alarmRequestSet(uint16_t request)
{
    LogPrintf(DEBUG_ALL, "alarmRequestSet==>0x%04X", request);
    sysinfo.alarmRequest |= request;
    if (sysinfo.alarmRequest != 0)
    	netRequestSet(NET_REQUEST_ALARM_ONE);
}
/**************************************************
@bref		清除报警上送
@param
@return
@note
**************************************************/

void alarmRequestClear(uint16_t request)
{
    LogPrintf(DEBUG_ALL, "alarmRequestClear==>0x%04X", request);
    sysinfo.alarmRequest &= ~request;
}

/**************************************************
@bref		报警任务
@param
@return
@note
**************************************************/

void alarmRequestTask(void)
{
    uint8_t alarm;
    static uint8_t tick = 0;
    if (primaryServerIsReady() == 0 || sysinfo.alarmRequest == 0)
    {
    	tick = 0;
    	if (sysinfo.alarmRequest == 0)
    	{
			netRequestClear(NET_REQUEST_ALARM_ONE);
    	}
        return;
    }
    if (sysinfo.alarmRequest != 0)
    {
		if (tick++ >= 180)
		{
			sysinfo.alarmRequest = 0;
			netRequestClear(NET_REQUEST_ALARM_ONE);
			tick = 0;
		}
    }
    if (getTcpNack() != 0)
    {
        return;
    }
    //感光报警
    if (sysinfo.alarmRequest & ALARM_LIGHT_REQUEST)
    {
        alarmRequestClear(ALARM_LIGHT_REQUEST);
        LogMessage(DEBUG_ALL, "alarmRequestTask==>Light Alarm");
        terminalAlarmSet(TERMINAL_WARNNING_LIGHT);
        alarm = 0;
        protocolSend(NORMAL_LINK, PROTOCOL_16, &alarm);
    }

    //低电报警
    if (sysinfo.alarmRequest & ALARM_LOWV_REQUEST)
    {
        alarmRequestClear(ALARM_LOWV_REQUEST);
        LogMessage(DEBUG_ALL, "alarmRequestTask==>LowVoltage Alarm");
        terminalAlarmSet(TERMINAL_WARNNING_LOWV);
        alarm = 0;
        protocolSend(NORMAL_LINK, PROTOCOL_16, &alarm);
    }

    //断电报警
    if (sysinfo.alarmRequest & ALARM_LOSTV_REQUEST)
    {
        alarmRequestClear(ALARM_LOSTV_REQUEST);
        LogMessage(DEBUG_ALL, "alarmRequestTask==>lostVoltage Alarm");
        terminalAlarmSet(TERMINAL_WARNNING_LOSTV);
        alarm = 0;
        protocolSend(NORMAL_LINK, PROTOCOL_16, &alarm);
    }

    //SOS报警
    if (sysinfo.alarmRequest & ALARM_SOS_REQUEST)
    {
        alarmRequestClear(ALARM_SOS_REQUEST);
        LogMessage(DEBUG_ALL, "alarmRequestTask==>SOS Alarm");
        terminalAlarmSet(TERMINAL_WARNNING_SOS);
        alarm = 0;
        protocolSend(NORMAL_LINK, PROTOCOL_16, &alarm);
    }

    //急加速报警
    if (sysinfo.alarmRequest & ALARM_ACCLERATE_REQUEST)
    {
        alarmRequestClear(ALARM_ACCLERATE_REQUEST);
        LogMessage(DEBUG_ALL, "alarmRequestTask==>Rapid Accleration Alarm");
        alarm = 9;
        protocolSend(NORMAL_LINK, PROTOCOL_16, &alarm);
    }
    //急减速报警
    if (sysinfo.alarmRequest & ALARM_DECELERATE_REQUEST)
    {
        alarmRequestClear(ALARM_DECELERATE_REQUEST);
        LogMessage(DEBUG_ALL, "alarmRequestTask==>Rapid Deceleration Alarm");
        alarm = 10;
        protocolSend(NORMAL_LINK, PROTOCOL_16, &alarm);
    }

    //急左转报警
    if (sysinfo.alarmRequest & ALARM_RAPIDLEFT_REQUEST)
    {
        alarmRequestClear(ALARM_RAPIDLEFT_REQUEST);
        LogMessage(DEBUG_ALL, "alarmRequestTask==>Rapid LEFT Alarm");
        alarm = 11;
        protocolSend(NORMAL_LINK, PROTOCOL_16, &alarm);
    }

    //急右转报警
    if (sysinfo.alarmRequest & ALARM_RAPIDRIGHT_REQUEST)
    {
        alarmRequestClear(ALARM_RAPIDRIGHT_REQUEST);
        LogMessage(DEBUG_ALL, "alarmRequestTask==>Rapid RIGHT Alarm");
        alarm = 12;
        protocolSend(NORMAL_LINK, PROTOCOL_16, &alarm);
    }
    //进入安全区域
    if (sysinfo.alarmRequest & ALARM_ENTERSAFEAREA_REQUEST)
    {
        alarmRequestClear(ALARM_ENTERSAFEAREA_REQUEST);
        LogMessage(DEBUG_ALL, "alarmRequestTask==>Enter safe area Alarm");
        alarm = 52;
        protocolSend(NORMAL_LINK, PROTOCOL_16, &alarm);
    }
    //离开安全区域
    if (sysinfo.alarmRequest & ALARM_LEAVESAFEAREA_REQUEST)
    {
        alarmRequestClear(ALARM_LEAVESAFEAREA_REQUEST);
        LogMessage(DEBUG_ALL, "alarmRequestTask==>Leave safe area Alarm");
        alarm = 53;
        protocolSend(NORMAL_LINK, PROTOCOL_16, &alarm);
    }
}



/**************************************************
@bref		更新运动或静止状态
@param
	src 		检测来源
	newState	新状态
@note
**************************************************/

static void motionStateUpdate(motion_src_e src, motionState_e newState)
{
    char type[20];


    if (motionInfo.motionState == newState)
    {
        return;
    }
    motionInfo.motionState = newState;
    switch (src)
    {
        case ACC_SRC:
            strcpy(type, "acc");
            break;
        case VOLTAGE_SRC:
            strcpy(type, "voltage");
            break;
        case GSENSOR_SRC:
            strcpy(type, "gsensor");
            break;
       	case SYS_SRC:
       		strcpy(type, "sys");
			LogPrintf(DEBUG_ALL, "Device %s , detected by %s", newState == MOTION_MOVING ? "moving" : "static", type);
            return;
        default:
            return;
            break;
    }
    LogPrintf(DEBUG_ALL, "Device %s , detected by %s", newState == MOTION_MOVING ? "moving" : "static", type);

    if (newState)
    {
//        netResetCsqSearch();
//        gpsRequestSet(GPS_REQUEST_UPLOAD_ONE);
//        gpsRequestSet(GPS_REQUEST_ACC_CTL); 
        terminalAccon();
//        hiddenServerCloseClear();
    }
    else
    {
//        gpsRequestSet(GPS_REQUEST_UPLOAD_ONE);
//        gpsRequestClear(GPS_REQUEST_ACC_CTL);
        terminalAccoff();
//        updateRTCtimeRequest();
    }
//    if (primaryServerIsReady())
//    {
//        protocolInfoResiter(getBatteryLevel(), sysinfo.outsidevoltage > 5.0 ? sysinfo.outsidevoltage : sysinfo.insidevoltage,
//                            dynamicParam.startUpCnt, dynamicParam.runTime);
//        protocolSend(NORMAL_LINK, PROTOCOL_13, NULL);
//        jt808SendToServer(TERMINAL_POSITION, getLastFixedGPSInfo());
//    }
}


/**************************************************
@bref       震动中断
@param
@note
**************************************************/

void motionOccur(void)
{
    motionInfo.tapInterrupt++;
}

/**************************************************
@bref       tapCnt 大小
@param
@note
**************************************************/

uint8_t motionGetSize(void)
{
    return sizeof(motionInfo.tapCnt);
}
/**************************************************
@bref		统计每一秒的中断次数
@param
@note
**************************************************/

static void motionCalculate(void)
{
    motionInfo.ind = (motionInfo.ind + 1) % sizeof(motionInfo.tapCnt);
    motionInfo.tapCnt[motionInfo.ind] = motionInfo.tapInterrupt;
    motionInfo.tapInterrupt = 0;
}
/**************************************************
@bref		获取这最近n秒的震动次数
@param
@note
**************************************************/

static uint16_t motionGetTotalCnt(uint8_t n)
{
    uint16_t cnt;
    uint8_t i;
    cnt = 0;
    for (i = 0; i < n; i++)
    {
        cnt += motionInfo.tapCnt[(motionInfo.ind + sizeof(motionInfo.tapCnt) - i) % sizeof(motionInfo.tapCnt)];
    }
    return cnt;
}

/**************************************************
@bref       检测单位时间内振动频率，判断是否运动
@param
@note
**************************************************/

static uint16_t motionCheckOut(uint8_t sec)
{
    uint8_t i;
    uint16_t validCnt;

    validCnt = 0;
    if (sec == 0 || sec > sizeof(motionInfo.tapCnt))
    {
        return 0;
    }
    for (i = 0; i < sec; i++)
    {
        if (motionInfo.tapCnt[(motionInfo.ind + sizeof(motionInfo.tapCnt) - i) % sizeof(motionInfo.tapCnt)] != 0)
        {
            validCnt++;
        }
    }
    return validCnt;
}

void motionClear(void)
{
	LogMessage(DEBUG_ALL, "motionClear==>OK");
	memset(motionInfo.tapCnt, 0, sizeof(motionInfo.tapCnt));
}

/**************************************************
@bref		获取运动状态
@param
@note
**************************************************/

motionState_e motionGetStatus(void)
{
    return motionInfo.motionState;
}

/**************************************************
@bref		清除运动状态
@param
@note
**************************************************/

void motionInit(void)
{
	memset(&motionInfo, 0, sizeof(motionInfo_s));
}

/**************************************************
@bref		运动和静止的判断
@param
@note
**************************************************/

static void motionCheckTask(void)
{
    static uint16_t gsStaticTick = 0;
    static uint16_t autoTick = 0;
    static uint8_t  accOnTick = 0;
    static uint8_t  accOffTick = 0;
    static uint8_t fixTick = 0;

    static uint8_t  volOnTick = 0;
    static uint8_t  volOffTick = 0;
    static uint8_t bfFlag = 0;
    static uint8_t bfTick = 0;
    static uint8_t lTick = 0, hTick = 0;
    static uint8_t vFlag = 0;
    static uint8_t motionState = 0;
    gpsinfo_s *gpsinfo;

    uint16_t totalCnt, staticTime;

    motionCalculate();

    if (sysparam.MODE == MODE21 || sysparam.MODE == MODE23)
    {
        staticTime = 180;
    }
    else
    {
        staticTime = 180;
    }



    if (sysparam.MODE == MODE1 || sysparam.MODE == MODE3 )
    {
        motionStateUpdate(SYS_SRC, MOTION_STATIC);
        gsStaticTick = 0;
        return ;
    }

    //保持运动状态时，如果gap大于Max，则周期性上报gps
    if (getTerminalAccState() && sysparam.gpsuploadgap >= GPS_UPLOAD_GAP_MAX)
    {
        if (++autoTick >= sysparam.gpsuploadgap)
        {
            autoTick = 0;
            gpsRequestSet(GPS_REQUEST_UPLOAD_ONE);
        }
    }
    else
    {
        autoTick = 0;
    }
    totalCnt = motionCheckOut(sysparam.gsdettime);
//        LogPrintf(DEBUG_ALL, "motionCheckOut=%d,%d,%d,%d,%d", totalCnt, sysparam.gsdettime, sysparam.gsValidCnt,
//                  sysparam.gsInvalidCnt, motionState);

    if (totalCnt >= sysparam.gsValidCnt && sysparam.gsValidCnt != 0)
    {
        motionState = 1;
    }
    else if (totalCnt <= sysparam.gsInvalidCnt)
    {
        motionState = 0;
    }



    if (ACC_READ == ACC_STATE_ON)
    {
        //线永远是第一优先级
        if (++accOnTick >= 10)
        {
            accOnTick = 0;
            motionStateUpdate(ACC_SRC, MOTION_MOVING);
        }
        accOffTick = 0;
        return;
    }
    accOnTick = 0;
    if (sysparam.accdetmode == ACCDETMODE0)
    {
        //仅由acc线控制
        if (++accOffTick >= 10)
        {
            accOffTick = 0;
            motionStateUpdate(ACC_SRC, MOTION_STATIC);
        }
        return;
    }

    if (sysparam.accdetmode == ACCDETMODE1 || sysparam.accdetmode == ACCDETMODE3)
    {
        //由acc线+电压控制
        if (sysinfo.outsidevoltage >= sysparam.accOnVoltage)
        {
            if (++volOnTick >= 5)
            {
                vFlag = 1;
                volOnTick = 0;
                motionStateUpdate(VOLTAGE_SRC, MOTION_MOVING);
            }
        }
        else
        {
            volOnTick = 0;
        }

        if (sysinfo.outsidevoltage < sysparam.accOffVoltage)
        {
            if (++volOffTick >= 15)
            {
                vFlag = 0;
                volOffTick = 0;
                if (sysparam.accdetmode == ACCDETMODE1)
                {
                    motionStateUpdate(MOTION_MOVING, MOTION_STATIC);
                }
            }
        }
        else
        {
            volOffTick = 0;
        }
        if (sysparam.accdetmode == ACCDETMODE1 || vFlag != 0)
        {
            return;
        }
    }
    //剩下的，由acc线+gsensor控制

    if (motionState)
    {
        motionStateUpdate(GSENSOR_SRC, MOTION_MOVING);
    }
    if (motionState == 0)
    {
        if (sysinfo.gpsOnoff)
        {
            gpsinfo = getCurrentGPSInfo();
            if (gpsinfo->fixstatus && gpsinfo->speed >= 7)
            {
                if (++fixTick >= 5)
                {
                    gsStaticTick = 0;
                }
            }
            else
            {
                fixTick = 0;
            }
        }
        gsStaticTick++;
        if (gsStaticTick >= staticTime)
        {
            motionStateUpdate(GSENSOR_SRC, MOTION_STATIC);
        }
    }
    else
    {
        gsStaticTick = 0;
    }
}

/**************************************************
@bref		运动检测任务
@param
@return
@note
**************************************************/

void movingStatusCheck(void)
{
	static uint8 dettick = 0;
	static uint8 flag = 0;
	static uint32_t last = 0, now = 0;
	static uint8_t staticTick = 0;
	if (sysinfo.gsensorOnoff == 0)
	{
		dettick    = 0;
		flag 	   = 0;
		last 	   = 0;
		now 	   = 0;
		staticTick = 0;
		motionStateUpdate(SYS_SRC, MOTION_STATIC);
		return;
	}
	if (flag == 0)
	{
		flag = 1;
		last = portUpdateStep();
	}
	//LogPrintf(DEBUG_ALL, "last:%d now:%d", last, now);
	if (dettick++ >= 10)
	{
		dettick = 0;
		now = portUpdateStep();
		if (ABS(now - last) >= 5)
		{
			motionStateUpdate(GSENSOR_SRC, MOTION_MOVING);
			staticTick = 0;
		}
		else
		{
			if (staticTick++ >= 10)
			{
				motionStateUpdate(GSENSOR_SRC, MOTION_STATIC);
				staticTick = 0;
			}
		}
		last = now;
	}
	if (motionInfo.motionState == MOTION_MOVING)
	{
		dynamicParam.runningtime++;
	}
}


/**************************************************
@bref		电压检测任务
@param
@return
@note
**************************************************/

static void voltageCheckTask(void)
{
    static uint16_t lowpowertick = 0;
    static uint8_t  lowwflag = 0;
    float x;
    static uint8_t protectTick = 0;
    uint8_t ret = 0;
    if (sysinfo.adcOnoff == 0)
	{
		return;
	}
	portAdcCfg(1);
    x = portGetAdcVol(ADC_CHANNEL);
    sysinfo.outsidevoltage = x * sysparam.adccal;
    sysinfo.insidevoltage  = sysinfo.outsidevoltage;

    //LogPrintf(DEBUG_ALL, "x:%.2f, outsidevoltage:%.2f", x, sysinfo.outsidevoltage);

	//电池保护
    if (sysinfo.outsidevoltage < 2.4 && sysinfo.canRunFlag == 1)
    {
		protectTick++;
		if (protectTick >= 5)
		{
			protectTick = 0;
			sysinfo.canRunFlag = 0;
			portDebugUartCfg(1);
			LogPrintf(DEBUG_ALL, "Batvoltage is lowwer than %.2f", sysinfo.outsidevoltage);
			portDebugUartCfg(0);
		}
    }
	else if (sysinfo.outsidevoltage >= 2.7 && sysinfo.canRunFlag == 0)
	{
		protectTick++;
		if (protectTick >= 5)
		{
			protectTick = 0;
			sysinfo.canRunFlag = 1;
			portDebugUartCfg(1);
			LogPrintf(DEBUG_ALL, "Batvoltage is more than %.2f", sysinfo.outsidevoltage);
			portDebugUartCfg(0); 
		}
		
	}
	else
	{
		protectTick = 0;
	}
    
//    //低电报警
//    if (sysinfo.outsidevoltage < sysinfo.lowvoltage)
//    {
//        lowpowertick++;
//        if (lowpowertick >= 30)
//        {
//            if (lowwflag == 0)
//            {
//                lowwflag = 1;
//                LogPrintf(DEBUG_ALL, "power supply too low %.2fV", sysinfo.outsidevoltage);
//                //低电报警
//                jt808UpdateAlarm(JT808_LOWVOLTAE_ALARM, 1);
//                alarmRequestSet(ALARM_LOWV_REQUEST);
//                lbsRequestSet(DEV_EXTEND_OF_MY);
//                wifiRequestSet(DEV_EXTEND_OF_MY);
//                gpsRequestSet(GPS_REQUEST_UPLOAD_ONE);
//            }
//
//        }
//    }
//    else
//    {
//        lowpowertick = 0;
//    }
//
//
//    if (sysinfo.outsidevoltage >= sysinfo.lowvoltage + 0.5)
//    {
//        lowwflag = 0;
//        jt808UpdateAlarm(JT808_LOWVOLTAE_ALARM, 0);
//    }


}

/**************************************************
@bref		模式状态机切换
@param
@return
@note
**************************************************/

static void changeModeFsm(uint8_t fsm)
{
    sysinfo.runFsm = fsm;
    LogPrintf(DEBUG_ALL, "changeModeFsm==>%d", fsm);
}

/**************************************************
@bref		快速关闭
@param
@return
@note
**************************************************/

static void modeShutDownQuickly(void)
{
    static uint16_t delaytick = 0;
    
    if (sysinfo.gpsRequest == 0 && sysinfo.alarmRequest    == 0 \
    							&& sysinfo.wifiExtendEvt   == 0 \
    							&& sysinfo.lbsRequest      == 0 \
    							&& sysinfo.netRequest      == 0 )
    {
        delaytick++;
        if (delaytick >= 20)
        {
            LogMessage(DEBUG_ALL, "modeShutDownQuickly==>shutdown");
            delaytick = 0;
            changeModeFsm(MODE_STOP); //执行完毕，关机
        }
    }
    else
    {
        delaytick = 0;
    }
}

/**************************************************
@bref		mode4切回在网模式
@param
@return
@note
**************************************************/

static void mode4CloseSocketQuickly(void)
{
	static uint16_t tick = 0;
	
	if (isModuleRunNormal())
	{
		if (sysinfo.gpsRequest == 0 && sysinfo.alarmRequest == 0 && sysinfo.wifiRequest == 0 && sysinfo.lbsRequest == 0 && sysinfo.netRequest == 0)
		{
			tick++;
			if (tick >= 15)
			{
				LogMessage(DEBUG_ALL, "mode4CloseSocketQuickly==>Offline");
				changeMode4Callback();
				tick = 0;
			}
		}
		else
		{
			tick = 0;
		}
		sysinfo.nonetTick = 0;
	}
	else if (isModuleRunNormal() == 0 && isModuleOfflineStatus() == 0)
	{
		sysinfo.nonetTick++;
		tick = 0;
		LogPrintf(DEBUG_ALL, "sysinfo.nonetTick:%d", sysinfo.nonetTick);
		if (sysinfo.nonetTick >= 270)
		{
			sysinfo.nonetTick = 0;
			LogMessage(DEBUG_ALL, "mode4CloseSocketQuickly==>Shut down");
			modeTryToStop();
		}
	}
	else
	{
		sysinfo.nonetTick = 0;
		tick = 0;
	}
}

/**************************************************
@bref		mode4d模组管理
@param
@return
@note
关闭模组不关闭kernal，因为还要计步数
**************************************************/

void mode4NetRequestTask(void)
{
	static uint16_t delaytick = 0;
	static uint16_t offlinetick = 0;

	switch (sysinfo.moduleFsm)
	{
		case MODULE_STATUS_CLOSE:
			if (sysinfo.netRequest != 0)
			{
				portAdcCfg(1);
				if (isModulePowerOff()) modulePowerOn();
				portLedGpioCfg(1);
				sysinfo.moduleFsm = MODULE_STATUS_OPEN;
			}
			delaytick = 0;
			break;
		case MODULE_STATUS_OPEN:
			/* 如果回到蓝牙围栏则关闭模组 */
		    if (sysinfo.gpsRequest == 0 && sysinfo.alarmRequest    == 0 \
										&& sysinfo.wifiRequest     == 0 \
										&& sysinfo.lbsRequest      == 0 \
										&& sysinfo.netRequest      == 0 \
										&& sysinfo.outBleFenceFlag == 0)
			{
				delaytick++;
				if (delaytick >= 10)
				{
					//LogMessage(DEBUG_ALL, "mode4CloseModuleQuickly==>ok");
					delaytick = 0;
					modulePowerOff();
					portAdcCfg(0);
					portLedGpioCfg(0);
					portSpkGpioCfg(0);
					sysinfo.moduleFsm = MODULE_STATUS_CLOSE;
				}
			}
			/* 如果回到wifi围栏则切换到在网模式 */
			if (isModuleRunNormal())
			{
				if (sysinfo.netRequest == NET_REQUEST_OFFLINE) 
				{
					offlinetick++;
					if (offlinetick >= 10) {
						changeMode4Callback();
						offlinetick = 0;
					}
				}
			}
			break;
		default:
			sysinfo.moduleFsm = MODULE_STATUS_CLOSE;
			break;
	}
}

/**************************************************
@bref		运行-》关机
@param
@return
@note
**************************************************/

void modeTryToStop(void)
{
    sysinfo.gpsRequest = 0;
    sysinfo.alarmRequest = 0;
    sysinfo.wifiRequest = 0;
    sysinfo.lbsRequest = 0;
    netRequestClear(NET_REQUEST_ALL);
    changeModeFsm(MODE_STOP);
    LogMessage(DEBUG_ALL, "modeTryToStop");
}

/**************************************************
@bref		待机
@param
@return
@note
**************************************************/

void modeTryToDone(void)
{
	sysinfo.gpsRequest = 0;
    sysinfo.alarmRequest = 0;
    sysinfo.wifiRequest = 0;
    sysinfo.lbsRequest = 0;
    netRequestClear(NET_REQUEST_ALL);
    changeModeFsm(MODE_DONE);
    LogMessage(DEBUG_ALL, "modeTryToDone");
}

/**************************************************
@bref		启动扫描
@param
@return
@note
**************************************************/

static void modeScan(void)
{
//    static uint8_t runTick = 0;
//    scanList_s  *list;
//    if (sysparam.leavealm == 0 || (sysparam.MODE != MODE1 && sysparam.MODE != MODE3))
//    {
//        runTick = 0;
//        changeModeFsm(MODE_START);
//        return;
//    }
//    if (runTick == 1)
//    {
//        portFsclkChange(1);
//        bleCentralDiscovery();
//    }
//    else if (runTick >= 20)
//    {
//        runTick = 0;
//        list = scanListGet();
//        if (list->cnt == 0)
//        {
//            alarmRequestSet(ALARM_LEAVE_REQUEST);
//        }
//        changeModeFsm(MODE_START);
//    }
//    runTick++;
}


/**************************************************
@bref		蓝牙状态机切换
@param
@return
@note
**************************************************/

static void bleChangeFsm(modeChoose_e fsm)
{
    bleTry.runFsm = fsm;
    bleTry.runTick = 0;
}

/**************************************************
@bref		扫描完成回调
@param
@return
@note
**************************************************/

void bleScanCallBack(deviceScanList_s *list)
{
    uint8_t i;
    for (i = 0; i < list->cnt; i++)
    {
        if (my_strpach(list->list[i].broadcaseName, "AUTO"))
        {
            LogPrintf(DEBUG_ALL, "Find Ble [%s],rssi:%d", list->list[i].broadcaseName, list->list[i].rssi);
            tmos_memcpy(bleTry.mac, list->list[i].addr, 6);
            bleTry.addrType = list->list[i].addrType;
            bleChangeFsm(BLE_CONN);
            return;
        }
    }
    LogMessage(DEBUG_ALL, "no find my ble");
    bleChangeFsm(BLE_SCAN);
}

/**************************************************
@bref		连接完成调
@param
@return
@note
**************************************************/

void bleConnCallBack(void)
{
    LogMessage(DEBUG_ALL, "connect success");
    bleChangeFsm(BLE_READY);
    dynamicParam.bleLinkCnt = 0;
    dynamicParamSaveAll();
    tmos_set_event(appCentralTaskId, APP_WRITEDATA_EVENT);
}

/**************************************************
@bref		连接完成调
@param
@return
@note
**************************************************/

void bleTryInit(void)
{
	tmos_memset(&bleTry, 0, sizeof(bleScanTry_s));
}

/**************************************************
@bref		模式选择，仅模式一与三下，蓝牙主机功能才起作用
@param
@return
@note
**************************************************/

static void modeChoose(void)
{

	bleChangeFsm(BLE_IDLE);
    changeModeFsm(MODE_START);
//	static uint8_t flag = 0;
//
//    if (sysparam.MODE != MODE1 && sysparam.MODE != MODE3)
//    {
//        bleChangeFsm(BLE_IDLE);
//        changeModeFsm(MODE_START);
//        return;
//    }
//    if (sysinfo.alarmRequest != 0)
//    {
//        bleChangeFsm(BLE_IDLE);
//        changeModeFsm(MODE_START);
//        return;
//    }
//    if (sysinfo.first == 0)
//    {
//		sysinfo.first = 1;
//		bleChangeFsm(BLE_IDLE);
//        changeModeFsm(MODE_START);
//        return;
//    }
//    if (sysparam.bleLinkFailCnt == 0)
//    {
//		bleChangeFsm(BLE_IDLE);
//        changeModeFsm(MODE_START);
////        dynamicParam.bleLinkCnt = 0;
////        dynamicParamSaveAll();
//        return;
//    }
//    if (flag == 0)
//    {
//    	flag = 1;
//		portFsclkChange(1);
//    }
//    switch (bleTry.runFsm)
//    {
//        case BLE_IDLE:
//            dynamicParam.startUpCnt++;
//            dynamicParam.bleLinkCnt++;
//            dynamicParamSaveAll();
//            wakeUpByInt(2, 30);
//            ledStatusUpdate(SYSTEM_LED_BLE, 1);
//             
//            portSetNextAlarmTime();
//            bleChangeFsm(BLE_SCAN);
//            bleTry.scanCnt = 0;
//            
//            break;
//        case BLE_SCAN:
//            bleTry.connCnt = 0;
//            if (bleTry.scanCnt++ < 3)
//            {
//                //启动扫描
//                centralStartDisc();
//                bleChangeFsm(BLE_SCAN_WAIT);
//            }
//            else
//            {
//                bleTry.scanCnt = 0;
//                //扫描失败
//                if (dynamicParam.bleLinkCnt >= sysparam.bleLinkFailCnt)
//                {
//                    LogPrintf(DEBUG_ALL, "scan fail==>%d", dynamicParam.bleLinkCnt);
//                    alarmRequestSet(ALARM_BLEALARM_REQUEST);
//                    dynamicParam.bleLinkCnt = 0;
//                    dynamicParamSaveAll();
//                    changeModeFsm(MODE_START);
//                    bleChangeFsm(BLE_IDLE);
//                    flag = 0;
//                }
//                else
//                {
//                    LogPrintf(DEBUG_ALL, "scan cnt==>%d", dynamicParam.bleLinkCnt);
//                    bleChangeFsm(BLE_DONE);
//                }
//
//            }
//            break;
//        case BLE_SCAN_WAIT:
//            //等待扫描结果
//            if (++bleTry.runTick >= 12)
//            {
//                bleChangeFsm(BLE_SCAN);
//            }
//            break;
//        case BLE_CONN:
//            //开始建立连接
//            if (bleTry.connCnt++ < 3)
//            {
//                centralEstablish(bleTry.mac, bleTry.addrType);
//                bleChangeFsm(BLE_CONN_WAIT);
//            }
//            else
//            {
//                bleTry.connCnt = 0;
//                if (dynamicParam.bleLinkCnt >= sysparam.bleLinkFailCnt)
//                {
//                    LogPrintf(DEBUG_ALL, "conn fail==>%d", dynamicParam.bleLinkCnt);
//                    dynamicParam.bleLinkCnt = 0;
//                    alarmRequestSet(ALARM_BLEALARM_REQUEST);
//                    dynamicParamSaveAll();
//                    changeModeFsm(MODE_START);
//                    bleChangeFsm(BLE_IDLE);
//                    flag = 0;
//                }
//                else
//                {
//                    bleChangeFsm(BLE_DONE);
//                }
//            }
//            break;
//        case BLE_CONN_WAIT:
//            //等待连接结果
//            if (++bleTry.runTick >= 12)
//            {
//                centralTerminate();
//                bleChangeFsm(BLE_CONN);
//            }
//            break;
//        case BLE_READY:
//            //蓝牙连接成功
//            if (++bleTry.runTick >= 20)
//            {
//                centralTerminate();
//                bleChangeFsm(BLE_DONE);
//            }
//            break;
//        case BLE_DONE:
//            POWER_OFF;
//            ledStatusUpdate(SYSTEM_LED_BLE, 0);
//            bleChangeFsm(BLE_IDLE);
//            changeModeFsm(MODE_DONE);
//            gpsRequestClear(GPS_REQUEST_ALL);
//            flag = 0;
//            break;
//        default:
//            bleChangeFsm(BLE_IDLE);
//            break;
//    }
}




/**************************************************
@bref		模式启动
@param
@return
@note
**************************************************/

static void modeStart(void)
{
    uint16_t year;
    uint8_t month, date, hour, minute, second;
    portGetRtcDateTime(&year, &month, &date, &hour, &minute, &second);
    sysinfo.runStartTick = sysinfo.sysTick;
    sysinfo.gpsuploadonepositiontime = 180;
    updateRTCtimeRequest();
    portFsclkChange(0);
    sysinfo.nonetTick = 0;
    switch (sysparam.MODE)
    {
        case MODE1:
            portGsensorCtl(0);
            dynamicParam.startUpCnt++;
            dynamicParamSaveAll();
            portSetNextAlarmTime();
            break;
        case MODE2:
            portGsensorCtl(1);
            if (sysparam.accctlgnss == 0)
            {
                gpsRequestSet(GPS_REQUEST_GPSKEEPOPEN_CTL);
            }
            break;
        case MODE3:
            portGsensorCtl(0);
            dynamicParam.startUpCnt++;
            dynamicParamSaveAll();
            break;
        case MODE21:
            portGsensorCtl(1);
            portSetNextAlarmTime();
            break;
        case MODE23:
            portGsensorCtl(1);
            break;
        /*离线模式*/
        case MODE4:
			portGsensorCtl(1);
		    netResetCsqSearch();
		    changeModeFsm(MODE_RUNING);
        	break;
        default:
            sysparam.MODE = MODE2;
            paramSaveAll();
            break;
    }
    LogPrintf(DEBUG_ALL, "modeStart==>%02d/%02d/%02d %02d:%02d:%02d", year, month, date, hour, minute, second);
    LogPrintf(DEBUG_ALL, "Mode:%d, startup:%d debug:%d %d", sysparam.MODE, dynamicParam.startUpCnt, sysparam.debug, dynamicParam.debug);
    netResetCsqSearch();
    ledStatusUpdate(SYSTEM_LED_RUN, 1);
    changeModeFsm(MODE_RUNING);
}

static void sysRunTimeCnt(void)
{
    static uint8_t runTick = 0;
    if (++runTick >= 180)
    {
        runTick = 0;
        dynamicParam.runTime++;
        dynamicParamSaveAll();
    }
}

void wifiCheckByStep(void)
{
	static uint8_t flag = 0;
	if (sysinfo.gsensorOnoff == 0)
		return;
	if (sysinfo.outBleFenceFlag == 0)
	{
		flag = 0;
		return;
	}
	if (sysparam.wifiCheckGapStep_out == 0 && sysparam.wifiCheckGapStep_in == 0)
	{
		sysinfo.alreadystep = 0;
		sysinfo.runningstep = 0;
		flag = 0;
		return;
	}
	if (flag == 0)
	{
		sysinfo.alreadystep = portUpdateStep();
		flag = 1;
		sysinfo.runningstep = 0;
		LogPrintf(DEBUG_ALL, "Update already step:%d", sysinfo.alreadystep);
	}
	sysinfo.runningstep = portUpdateStep(); 

	if (abs(sysinfo.runningstep - sysinfo.alreadystep) >= sysparam.wifiCheckGapStep_out && sysinfo.outWifiFenceFlag && sysparam.wifiCheckGapStep_out != 0)
	{
		flag = 0;
		/* 刷新一下搜WIFI时间 */
		sysinfo.outWifiTick = 0;
		sysinfo.inWifiTick = 0;
		LogPrintf(DEBUG_ALL, "[OUT]Reach the target number of steps==>%d", sysinfo.runningstep);
		//gpsRequestSet(GPS_REQUEST_UPLOAD_ONE);
		LogPrintf(DEBUG_BLE, "wifireq line:%d", __LINE__);
		wifiRequestSet(DEV_EXTEND_OF_FENCE);
		
	}
	else if (abs(sysinfo.runningstep - sysinfo.alreadystep) >= sysparam.wifiCheckGapStep_in && sysinfo.outWifiFenceFlag == 0 && sysparam.wifiCheckGapStep_in != 0)
	{
		flag = 0;
		/* 刷新一下搜WIFI时间 */
		sysinfo.outWifiTick = 0;
		sysinfo.inWifiTick = 0;
		LogPrintf(DEBUG_ALL, "[IN]Reach the target number of steps==>%d", sysinfo.runningstep);
		LogPrintf(DEBUG_BLE, "wifireq line:%d", __LINE__);
		wifiRequestSet(DEV_EXTEND_OF_FENCE);
		
	}
}

/**************************************************
@bref		模式运行
@param
@return
@note
**************************************************/

static void modeRun(void)
{
    static uint8_t runtick = 0;
    switch (sysparam.MODE)
    {
        case MODE1:
        case MODE3:
            //该模式下工作3分半钟
            if ((sysinfo.sysTick - sysinfo.runStartTick) >= 210)
            {
                gpsRequestClear(GPS_REQUEST_ALL);
                changeModeFsm(MODE_STOP);
            }
            modeShutDownQuickly();
            break;
        case MODE2:
            //该模式下每隔3分钟记录时长
            sysRunTimeCnt();
            gpsUploadPointToServer();
            break;
        case MODE21:
        case MODE23:
            //该模式下无gps请求时，自动关机
            sysRunTimeCnt();
            modeShutDownQuickly();
            gpsUploadPointToServer();
            break;
        case MODE4:
			sysRunTimeCnt();
			modeShutDownQuickly();
        	break;
        default:
            LogMessage(DEBUG_ALL, "mode change unknow");
            sysparam.MODE = MODE2;
            break;
    }
}

/**************************************************
@bref		模式接收
@param
@return
@note
**************************************************/

static void modeStop(void)
{
    if (sysparam.MODE == MODE1 || sysparam.MODE == MODE3)
    {
        portGsensorCtl(0);
    }
    LogMessage(DEBUG_ALL, "mode change to stop");
    ledStatusUpdate(SYSTEM_LED_RUN, 0);
    //modulePowerOff();
    changeModeFsm(MODE_DONE);
}


/**************************************************
@bref		等待唤醒模式
@param
@return
@note
**************************************************/

static void modeDone(void)
{
	static uint8_t motionTick = 0;

	bleTryInit();
    if (sysinfo.gpsRequest || sysinfo.netRequest)
    {
        motionTick = 0;
        volCheckRequestSet();
        if (sysparam.MODE == MODE1 || sysparam.MODE == MODE3)
        {
            changeModeFsm(MODE_CHOOSE);
        }
        else
        {
            changeModeFsm(MODE_START);
        }
        LogMessage(DEBUG_ALL, "modeDone==>Change to mode start");
    }
    else if (sysparam.MODE == MODE1 || sysparam.MODE == MODE3 || sysparam.MODE == MODE4)
    {
    	motionTick = 0;
        if (sysinfo.sleep && isModulePowerOff() && sysinfo.kernalRun)
        {
            tmos_set_event(sysinfo.taskId, APP_TASK_STOP_EVENT);
        }
    }
    else if (sysparam.MODE == MODE23 || sysparam.MODE == MODE21 || sysparam.MODE == MODE2)
    {
		/*检测gsensor是否有中断进来*/
		//LogPrintf(DEBUG_ALL, "motioncnt:%d, motionTick:%d ", motionCheckOut(sysparam.gsdettime), motionTick);
		if (motionCheckOut(sysparam.gsdettime) <= 1)
		{
			if (sysinfo.sleep && sysinfo.kernalRun && isModulePowerOff())
			{
				tmos_set_event(sysinfo.taskId, APP_TASK_STOP_EVENT);
				motionTick = 0;
			}

		}
    }
}

/**************************************************
@bref		当前是否为运行模式
@param
@return
	1		是
	0		否
@note
**************************************************/

uint8_t isModeRun(void)
{
    if (sysinfo.runFsm == MODE_RUNING || sysinfo.runFsm == MODE_START)
        return 1;
    return 0;
}
/**************************************************
@bref		当前是否为done模式
@param
@return
	1		是
	0		否
@note
**************************************************/

uint8_t isModeDone(void)
{
    if (sysinfo.runFsm == MODE_DONE || sysinfo.runFsm == MODE_STOP)
        return 1;
    return 0;
}


/**************************************************
@bref		系统到时自动唤醒
@param
@return
@note
**************************************************/

static void sysAutoReq(void)
{
    uint16_t year;
    uint8_t month, date, hour, minute, second;

    if (sysparam.MODE == MODE1 || sysparam.MODE == MODE21)
    {
        portGetRtcDateTime(&year, &month, &date, &hour, &minute, &second);
        if (date == sysinfo.alarmDate && hour == sysinfo.alarmHour && minute == sysinfo.alarmMinute)
        {
            LogPrintf(DEBUG_ALL, "sysAutoReq==>%02d/%02d/%02d %02d:%02d:%02d", year, month, date, hour, minute, second);
            gpsRequestSet(GPS_REQUEST_UPLOAD_ONE);
            if (sysinfo.kernalRun == 0)
            {
            	volCheckRequestSet();
            	LogPrintf(DEBUG_ALL, "kernal start==>%s, %d", __FUNCTION__, __LINE__);
                tmos_set_event(sysinfo.taskId, APP_TASK_RUN_EVENT);
            }
        }
    }
    else if (sysparam.MODE == MODE4)
    {
    	if (sysparam.mode4GapMin != 0)
        {
    	    sysinfo.mode4SysMin++;
            if (sysinfo.mode4SysMin >= sysparam.mode4GapMin)
            {
                sysinfo.mode4SysMin = 0;
                lbsRequestSet(DEV_EXTEND_OF_MY);
   	 			gpsRequestSet(GPS_REQUEST_UPLOAD_ONE);
                netRequestSet(NET_REQUEST_CONNECT_ONE);
                LogPrintf(DEBUG_ALL, "mode 4 gps upload");
                if (sysinfo.kernalRun == 0)
                {
                    volCheckRequestSet();
                    LogPrintf(DEBUG_ALL, "kernal start==>%s, %d", __FUNCTION__, __LINE__);
                    tmos_set_event(sysinfo.taskId, APP_TASK_RUN_EVENT);
                    changeModeFsm(MODE_START);
                }
            }
        }
        
		if (sysinfo.noNetFlag)
		{
			/*mode4 没网的工作逻辑*/
			sysinfo.mode4NoNetTick++;
			LogPrintf(DEBUG_ALL, "mode4NoNetTick:%d", sysinfo.mode4NoNetTick);
			if (sysinfo.mode4NoNetTick >= 60)
			{
				sysinfo.mode4NoNetTick = 0;
				sysinfo.noNetFlag = 0;
                LogMessage(DEBUG_ALL, "mode 4 restoration network");
                if (sysinfo.kernalRun == 0)
                {
                	volCheckRequestSet();
                	LogPrintf(DEBUG_ALL, "kernal start==>%s, %d", __FUNCTION__, __LINE__);
                    tmos_set_event(sysinfo.taskId, APP_TASK_RUN_EVENT);
                    changeModeFsm(MODE_START);
                }
			}
		}
		/*mode4 正常搜围栏逻辑*/
		else
		{
			sysinfo.mode4NoNetTick = 0;
			/*没出WIFI围栏且离开蓝牙围栏*/
			if (sysinfo.outWifiFenceFlag == 0 && sysinfo.outBleFenceFlag)
			{
				sysinfo.outWifiTick = 0;
				if (++sysinfo.inWifiTick >= sysparam.wifiCheckGapMin_in)
				{
					sysinfo.inWifiTick = 0;
					LogMessage(DEBUG_ALL, "[IN]Wifi period");
					wifiRequestSet(DEV_EXTEND_OF_FENCE);
					/* 刷新一下搜WIFI步数 */
					 sysinfo.alreadystep = portUpdateStep();
					 sysinfo.runningstep = portUpdateStep();
				}
			}
			/*已经WIFI出围栏，上线，不受蓝牙围栏限制*/
			if (sysinfo.outWifiFenceFlag)
			{
				sysinfo.inWifiTick = 0;
				if (++sysinfo.outWifiTick >= sysparam.wifiCheckGapMin_out)
				{
					sysinfo.outWifiTick = 0;
					LogMessage(DEBUG_ALL, "[OUT]Wifi out period");
					wifiRequestSet(DEV_EXTEND_OF_FENCE);
					/* 刷新一下WIFI步数 */
					sysinfo.alreadystep = portUpdateStep();
					sysinfo.runningstep = portUpdateStep();
				}
			}
		}
    }
    else
    {
        if (sysparam.gapMinutes != 0)
        {
            sysinfo.sysMinutes++;
            if (sysinfo.sysMinutes % sysparam.gapMinutes == 0)
            {
            	sysinfo.sysMinutes = 0;
				gpsRequestSet(GPS_REQUEST_UPLOAD_ONE);
                LogMessage(DEBUG_ALL, "upload period");
                if (sysinfo.kernalRun == 0)
                {
                	volCheckRequestSet();
                	LogPrintf(DEBUG_ALL, "kernal start==>%s, %d", __FUNCTION__, __LINE__);
                    tmos_set_event(sysinfo.taskId, APP_TASK_RUN_EVENT);
                }
            }
        }
    }
}

/**************************************************
@bref		电池低电关机检测
@param
@return
@note	0：正在检测				1：检测完成
**************************************************/

uint8_t SysBatDetection(void)
{
	static uint8_t waitTick = 0;
	/*开机检测电压*/
	if (sysinfo.volCheckReq == 0)
    {
        if (sysinfo.canRunFlag)
        {
			waitTick = 0;
			volCheckRequestClear();
			LogPrintf(DEBUG_ALL, "电池电压正常，正常开机:%f", sysinfo.outsidevoltage);
        }
        else
        {
			if (waitTick++ >= 6)
			{
				waitTick = 0;
				changeModeFsm(MODE_DONE);
				volCheckRequestClear();
				gpsRequestClear(GPS_REQUEST_ALL);
				netRequestClear(NET_REQUEST_ALL);
				LogPrintf(DEBUG_ALL, "电池电压偏低，关机:%f", sysinfo.outsidevoltage);
			}
        }
        return 0;
    }
	/*工作时检测电压*/
	/*不能工作*/
	if (sysinfo.canRunFlag == 0)
	{
		/*如果正在工作*/
		if (sysinfo.runFsm == MODE_RUNING)
		{
			modeTryToStop();
			if (sysparam.MODE == MODE2 || sysparam.MODE == MODE21 || sysparam.MODE == MODE23)
			{
				if (sysinfo.gsensorOnoff == 1)
				{
					portGsensorCtl(0);
				}
			}
		}
		else if (sysinfo.runFsm == MODE_START || sysinfo.runFsm == MODE_CHOOSE)
		{
			modeTryToDone();
		}
	}
	/*可以工作*/
	else
	{
		if (sysinfo.canRunFlag == 1)
		{
			if (sysparam.MODE == MODE2 || sysparam.MODE == MODE21 || sysparam.MODE == MODE23)
			{
				if (sysinfo.gsensorOnoff == 0)
				{
					portGsensorCtl(1);
				}
			}
		}
	}
	return 1;
}

/**************************************************
@bref		电池检查请求设置
@param
@return
@note
**************************************************/

void volCheckRequestSet(void)
{
	sysinfo.volCheckReq = 0;
	sysinfo.canRunFlag = 0;
	LogMessage(DEBUG_ALL, "volCheckRequestSet==>OK");
}

/**************************************************
@bref		电池检查请求清除
@param
@return
@note
**************************************************/

void volCheckRequestClear(void)
{
	sysinfo.volCheckReq = 1;
	LogMessage(DEBUG_ALL, "volCheckRequestClear==>OK");
}

/**************************************************
@bref		模式运行任务
@param
@return
@note
**************************************************/

static void sysModeRunTask(void)
{
	if (SysBatDetection() != 1)
	{
		return;
	}
    switch (sysinfo.runFsm)
    {
        case MODE_CHOOSE:
            modeChoose();
            break;
        case MODE_START:
            modeStart();
            break;
        case MODE_RUNING:
            modeRun();
            break;
        case MODE_STOP:
            modeStop();
            break;
        case MODE_DONE:
            modeDone();
            break;
    }
}

/**************************************************
@bref		基站上送请求
@param
@return
@note
**************************************************/

void lbsRequestSet(uint8_t ext)
{
    sysinfo.lbsRequest = 1;
    sysinfo.lbsExtendEvt |= ext;
}

/**************************************************
@bref		清除基站上送请求
@param
@return
@note
**************************************************/

void lbsRequestClear(void)
{
	sysinfo.lbsRequest = 0;
    sysinfo.lbsExtendEvt = 0;
}

static void sendLbs(void)
{
    if (sysinfo.lbsExtendEvt & DEV_EXTEND_OF_MY)
    {
    	sysinfo.jt808Lbs = 1;
        protocolSend(NORMAL_LINK, PROTOCOL_19, NULL);
        jt808SendToServer(TERMINAL_POSITION, getCurrentGPSInfo());

    }
    if (sysinfo.lbsExtendEvt & DEV_EXTEND_OF_BLE)
    {
        protocolSend(BLE_LINK, PROTOCOL_19, NULL);
    }
    sysinfo.lbsExtendEvt = 0;
}
/**************************************************
@bref		基站上送任务
@param
@return
@note
**************************************************/

static void lbsRequestTask(void)
{
    if (sysinfo.lbsRequest == 0)
    {
        return;
    }
    if (primaryServerIsReady() == 0)
        return;
    sysinfo.lbsRequest = 0;
    moduleGetLbs();
    startTimer(30, sendLbs, 0);
    
}

static int8_t wifiTimeOutId = -1;
/**************************************************
@bref		wifi超时处理
@param
@return
@note
**************************************************/

void wifiTimeout(void)
{
	LogMessage(DEBUG_ALL, "wifiTimeout");
	sysinfo.wifiExtendEvt = 0;
	wifiTimeOutId = -1;
	netRequestClear(NET_REQUEST_WIFI_CTL);
	if (sysinfo.wifiScanCnt > 0)
	{
		wifiRequestSet(DEV_EXTEND_OF_FENCE);
		sysinfo.wifiScanCnt--;
		LogPrintf(DEBUG_BLE, "Wifi timeout, try again:%d", sysinfo.wifiScanCnt);
	}
	if (sysinfo.runFsm == MODE_RUNING)
		moduleReset();
}

/**************************************************
@bref		wifi应答成功
@param
@return
@note
**************************************************/

void wifiRspSuccess(void)
{
	LogMessage(DEBUG_ALL, "wifiRspSuccess");
	if (wifiTimeOutId != -1)
	{
		stopTimer(wifiTimeOutId);
		wifiTimeOutId = -1;
	}
	netRequestClear(NET_REQUEST_WIFI_CTL);
}

/**************************************************
@bref		设置WIFI上送请求
@param
@return
@note
**************************************************/

void wifiRequestSet(uint8_t ext)
{
	/* 初次wifi请求 */
	if ((sysinfo.wifiExtendEvt & DEV_EXTEND_OF_FENCE) == 0 && (ext & DEV_EXTEND_OF_FENCE))
	{
		sysinfo.wifiScanCnt = 1;
		netRequestSet(NET_REQUEST_WIFI_CTL);
	}
	if (ext & DEV_EXTEND_OF_FENCE)
    {
		netRequestSet(NET_REQUEST_WIFI_CTL);
    }
    sysinfo.wifiRequest = 1;
    sysinfo.wifiExtendEvt |= ext;
    LogPrintf(DEBUG_ALL, "wifiRequestSet==>0x%04x", ext);
}

/**************************************************
@bref		清除WIFI上送请求
@param
@return
@note
**************************************************/

void wifiRequestClear(uint8_t ext)
{
	sysinfo.wifiExtendEvt &= ~ext;
	if (sysinfo.wifiExtendEvt == 0)
	{
		sysinfo.wifiRequest = 0;
	}
}

/**************************************************
@bref		清除WIFI上送获取
@param
@return
@note
**************************************************/

uint8_t wifiRequestGet(uint8_t ext)
{
	return (sysinfo.wifiExtendEvt & ext);
}



/**************************************************
@bref		WIFI上送任务
@param
@return
@note
**************************************************/

static void wifiRequestTask(void)
{
	if (isModulePowerOff())
	{
		return;
	}
    if (sysinfo.wifiRequest == 0)
    {
        return;
    }
    if (primaryServerIsReady() == 0 && wifiRequestGet(DEV_EXTEND_OF_FENCE) == 0)
    {
        return;
    }
    /* 搜wifi围栏的时候, 需要模组驻网成功 */
    if (wifiRequestGet(DEV_EXTEND_OF_FENCE) && isModuleOfflineStatus() == 0 && isModuleRunNormal() == 0)
    {
    	return;
   	}
   	if (sysinfo.agpsRequest)
   	{
		return;
   	}
    sysinfo.wifiRequest = 0;
    startTimer(30, moduleGetWifiScan, 0);
    wifiTimeOutId = startTimer(620, wifiTimeout, 0);
}

/**************************************************
@bref		唤醒设备
@param
@return
@note
**************************************************/
void wakeUpByInt(uint8_t      type, uint8_t sec)
{
    switch (type)
    {
        case 0:
            sysinfo.ringWakeUpTick = sec;
            break;
        case 1:
        	if (sec >= sysinfo.cmdTick)
            	sysinfo.cmdTick = sec;
            break;
        case 2:
        	sysinfo.irqTick = sec;
        	break;
    }

    portSleepDn();
}

/**************************************************
@bref		查询是否需要休眠
@param
@return
@note
**************************************************/

static uint8_t getWakeUpState(void)
{
    //打印串口信息时，不休眠
    if (sysinfo.logLevel == DEBUG_FACTORY)
    {
        return 1;
    }
    //未联网，不休眠
    if (primaryServerIsReady() == 0 && isModeRun() && sysparam.MODE != MODE4)
    {
        return 2;
    }
    //开gps时，不休眠
    if (sysinfo.gpsRequest != 0)
    {
        return 3;
    }
    if (sysinfo.ringWakeUpTick != 0)
    {
        return 4;
    }
    if (sysinfo.cmdTick != 0)
    {
        return 5;
    }
    if (sysinfo.irqTick != 0)
    {
		return 6;
    }
    //扫描wifi时,不休眠
    if (sysinfo.wifiExtendEvt != 0)
    {
		return 7;
    }
    //播放音频时,不休眠
    if (netRequestGet(NET_REQUEST_TTS_CTL))
    {
		return 8;
    }
//    if (sysinfo.petBellOnoff != 0 && sysinfo.petbellPlaying != 0)
//    {
//		return 9;
//    }

    if (rspTimeOut != -1)
    {
		return 11;
    }
    /*模式4不在offline或不在normal模式，不休眠*/
    if (sysparam.MODE == MODE4 && isModeRun() && 
    	(isModulePowerOff()== 0 && (isModuleOfflineStatus() == 0 && isModuleRunNormal() == 0)))//模组开着且在联网状态
    {
    	return 12;
    }
    //非0 时强制不休眠
    return 0;
}

/**************************************************
@bref		自动休眠
@param
@return
@note
**************************************************/

void autoSleepTask(void)
{
    static uint8_t flag = 0;
    if (sysinfo.ringWakeUpTick != 0)
    {
        sysinfo.ringWakeUpTick--;
    }
    if (sysinfo.cmdTick != 0)
    {
        sysinfo.cmdTick--;
    }
    if (sysinfo.irqTick != 0)
    {
		sysinfo.irqTick--;
    }
    //LogPrintf(DEBUG_ALL, "getWakeUpState:%d", getWakeUpState());
    if (getWakeUpState())
    {
        portSleepDn();
        if (flag != 0)
        {
            flag = 0;
            portFsclkChange(0);
            LogMessage(DEBUG_ALL, "disable sleep");
            tmos_start_reload_task(sysinfo.taskId, APP_TASK_POLLUART_EVENT, MS1_TO_SYSTEM_TIME(50));
            sysinfo.sleep = 0;
            //portDebugUartCfg(1);
        }
    }
    else
    {
        portSleepEn();
        if (flag != 1)
        {
            flag = 1;
            portFsclkChange(1);
            LogMessage(DEBUG_ALL, "enable sleep");
            tmos_stop_task(sysinfo.taskId, APP_TASK_POLLUART_EVENT);
            sysinfo.sleep = 1;
            //portDebugUartCfg(0);
        }
    }
}

/**************************************************
@bref		每天重启
@param
@note
**************************************************/

static void rebootEveryDay(void)
{
    
    //    if (sysinfo.sysTick < 86400)
    //        return ;
    //    if (sysinfo.gpsRequest != 0)
    //        return ;
    //    portSysReset();
}



/**************************************************
@bref		计算设备正常状态时间
@param
@return
@note	
如果设备处于暗状态或者处于平放状态，每一分钟计数+1
**************************************************/

void calculateNormalTime(void)
{



}

/**************************************************
@bref		感光任务
@param
@return
@note
**************************************************/
static void lightDetectionTask(void)
{
	if (sysparam.ldrEn == 0)
		return;
	//LogPrintf(DEBUG_ALL, "ldr:%d", LDR_READ);
	if (sysinfo.ldrIrqFlag)
	{
		//亮
		if (sysinfo.ldrDarkCnt >= 2)
		{
			LogMessage(DEBUG_ALL, "Light alarm");
			alarmRequestSet(ALARM_LIGHT_REQUEST);
			gpsRequestSet(GPS_REQUEST_UPLOAD_ONE);
			jt808UpdateAlarm(JT808_LIGHT_ALARM, 1);
			lbsRequestSet(DEV_EXTEND_OF_MY);
			wifiRequestSet(DEV_EXTEND_OF_MY);		
		}
		else
		{
			LogMessage(DEBUG_ALL, "Light alarm cancel");
		}
		sysinfo.ldrIrqFlag = 0;
		sysinfo.ldrDarkCnt = 0;
	}
	else
	{
		//暗
		
	}
}

/**************************************************
@bref		123上报任务
@param
@return
@note
**************************************************/
static void mode123UploadTask(void)
{
	gpsinfo_s *gpsinfo;
	static uint8_t uploadTick = 0, nofixTick = 0;
	if (gpsRequestGet(GPS_REQUEST_123_CTL) == 0)
	{
		uploadTick = 0;
		sysinfo.mode123RunTick = 0;
		nofixTick = 0;
		return;
	}
	wifiRequestClear(DEV_EXTEND_OF_FENCE);
	if (sysinfo.mode123Min == 0)
	{
		uploadTick = 0;
		sysinfo.mode123RunTick = 0;
		nofixTick = 0;
		return;
	}
	if (sysinfo.gpsOnoff == 0)
	{
		uploadTick = 0;
		sysinfo.mode123RunTick = 0;
		nofixTick = 0;
		return;
	}
	if (sysinfo.mode123RunTick >= sysinfo.mode123Min * 60)
	{
		/* 先关GPS */
		
		if (dbUpload() == 0)
        {
            if (gpsRestoreUpload() == 0)
            {
				LogPrintf(DEBUG_ALL, "补传完成");
				gpsRequestClear(GPS_REQUEST_123_CTL);
				if (netRequestGet(NET_REQUEST_CONNECT_ONE) && gpsRequestGet(GPS_REQUEST_123_CTL) == 0)
	            {
					netRequestClear(NET_REQUEST_CONNECT_ONE);
	            }
				//resetSafeArea();
            }
        }
        if (sysinfo.mode123RunTick + 300 >= sysinfo.mode123Min * 60) 
        {
			//resetSafeArea();
			gpsRequestClear(GPS_REQUEST_123_CTL);
			if (netRequestGet(NET_REQUEST_CONNECT_ONE) && gpsRequestGet(GPS_REQUEST_123_CTL) == 0)
            {
				netRequestClear(NET_REQUEST_CONNECT_ONE);
            }
        }
		return;
	}

	nofixTick++;
	gpsinfo = getCurrentGPSInfo();
	if (gpsinfo->fixstatus == 0)
	{
		uploadTick = 0;
		sysinfo.mode123RunTick = 0;
		if ((nofixTick % 10) == 0)
		{
			lbsRequestSet(DEV_EXTEND_OF_MY);
		}
		if (nofixTick >= 180)
		{
			sysinfo.mode123RunTick = sysinfo.mode123Min * 60;
			startTimer(50, resetSafeArea, 0);
			protocolSend(NORMAL_LINK, PROTOCOL_12, getLastFixedGPSInfo());
	        jt808SendToServer(TERMINAL_POSITION, getLastFixedGPSInfo());
		}
		return;
	}
	lbsRequestClear();
	uploadTick++;
	sysinfo.mode123RunTick++;
	if (uploadTick >= sysinfo.mode123GpsFre)
	{
//	    if (calculateDistanceOfPoint() == 1)
//        {
			protocolSend(NORMAL_LINK, PROTOCOL_12, getCurrentGPSInfo());
	        jt808SendToServer(TERMINAL_POSITION, getCurrentGPSInfo());
	        initLastPoint(getCurrentGPSInfo());
	        uploadTick = 0;
//        }
	}


}

/**************************************************
@bref       gsensor检查任务
@param
@note
**************************************************/
static void gsensorRepair(void)
{
    portGsensorCtl(1);
    LogMessage(DEBUG_ALL, "repair gsensor");
}

static void gsCheckTask(void)
{
    static uint8_t tick = 0;
    static uint8_t errorcount = 0;
    if (sysinfo.gsensorOnoff == 0)
    {
        tick = 0;
        return;
    }

    tick++;
    if (tick % 60 == 0)
    {
        tick = 0;
        if (readInterruptConfig() != 0)
        {
            LogMessage(DEBUG_ALL, "gsensor error");
            portGsensorCtl(0);
            startTimer(20, gsensorRepair, 0);

        }
        else
        {
            errorcount = 0;
        }
    }
}


/**************************************************
@bref		关机任务
@param
@return
@note
**************************************************/
void systemCloseTask(void)
{
	static uint8_t tick = 0;
	if (sysinfo.logLevel != 0)
	{
		tick = 0;
		return;
	}
	if (tick++ >= 20)
	{
		portSyspwkOffGpioCfg();
		tick = 0;
	}
}

/**************************************************
@bref		寻宠语音播报
@param
@return
@note
**************************************************/

void lookForPetSpeakerTask(void)
{
	static uint8_t tick = 0;
	if (sysinfo.petSpkCnt == 0)
	{
		tick = 0;
		return;
	}
	if (sysinfo.petBellOnoff && sysinfo.petbellPlaying)
	{
		tick = 0;
		LogPrintf(DEBUG_ALL, "Pet bell is playing, waiting..");
		return;
	}
	if (strlen(sysinfo.ttsContent) == 0)
	{
		tick = 0;
		return;
	}
	if (getModuleStatus() < CPIN_STATUS)
	{
		return;
	}
	if (tick++ % sysinfo.petSpkGap == 0)
	{
		addTTS(sysinfo.ttsContent, sysinfo.ttsContentLen);
		if (sysinfo.petSpkCnt > 0)
			sysinfo.petSpkCnt--;
	}
}

/**************************************************
@bref		回归安全围栏
@param
@return
@note
**************************************************/

void resetSafeArea(void)
{
	lbsRequestClear();
	netRequestClear(NET_REQUEST_KEEPNET_CTL | NET_REQUEST_OFFLINE | NET_REQUEST_CONNECT_ONE);
	wifiRequestClear(DEV_EXTEND_OF_FENCE);
	gpsRequestClear(GPS_REQUEST_ALL);
	sysinfo.mode123Min = 0;		// 退出123模式
	sysinfo.flag123 = 0;
	sysinfo.outBleFenceFlag = 0;
	sysinfo.outWifiFenceFlag = 0;
	LogMessage(DEBUG_ALL, "resetSafeArea==>OK");

	/* 重新计时 */
	sysinfo.alreadystep = portUpdateStep();
	sysinfo.runningstep = portUpdateStep();
	sysinfo.outWifiTick = 0;
	sysinfo.inWifiTick  = 0;
}

///**************************************************
//@bref		模组管理
//@param
//@return
//@note
//
//**************************************************/
//
//void netRequestTask(void)
//{
//	static uint16_t delaytick = 0;
//	static uint16_t offlinetick = 0;
//
//	switch (sysinfo.moduleFsm)
//	{
//		case MODULE_STATUS_CLOSE:
//			if (sysinfo.netRequest != 0)
//			{
//				portAdcCfg(1);
//				modulePowerOn();
//				portLedGpioCfg(1);
//				sysinfo.moduleFsm = MODULE_STATUS_OPEN;
//			}
//			delaytick = 0;
//			break;
//		case MODULE_STATUS_OPEN:
//			/* 如果回到蓝牙围栏则关闭模组 */
//		    if (sysinfo.gpsRequest == 0 && sysinfo.alarmRequest    == 0 \
//										&& sysinfo.wifiRequest     == 0 \
//										&& sysinfo.lbsRequest      == 0 \
//										&& sysinfo.netRequest      == 0 \
//										&& sysinfo.outBleFenceFlag == 0)
//			{
//				delaytick++;
//				if (delaytick >= 10)
//				{
//					//LogMessage(DEBUG_ALL, "mode4CloseModuleQuickly==>ok");
//					delaytick = 0;
//					modulePowerOff();
//					portAdcCfg(0);
//					portLedGpioCfg(0);
//					portSpkGpioCfg(0);
//					sysinfo.moduleFsm = MODULE_STATUS_CLOSE;
//				}
//			}
//			/* 如果回到wifi围栏则切换到在网模式 */
//			if (isModuleRunNormal())
//			{
//				if (sysinfo.netRequest == NET_REQUEST_OFFLINE) 
//				{
//					offlinetick++;
//					if (offlinetick >= 10) {
//						changeMode4Callback();
//						offlinetick = 0;
//					}
//				}
//			}
//			break;
//		default:
//			sysinfo.moduleFsm = MODULE_STATUS_CLOSE;
//			break;
//	}
//}
/**************************************************
@bref		网络请求设置
@param
@return
@note
**************************************************/

void netRequestSet(uint32_t req)
{
	sysinfo.netRequest |= req;
	//LogPrintf(DEBUG_ALL, "netRequestSet==>0x%04x", req);
//	if (sysinfo.kernalRun == 0)
//	{
//		volCheckRequestSet();
//		LogPrintf(DEBUG_ALL, "kernal start==>%s, %d", __FUNCTION__, __LINE__);
//		tmos_set_event(sysinfo.taskId, APP_TASK_RUN_EVENT);
//		changeModeFsm(MODE_START);
//	}
}

/**************************************************
@bref		网络请求清除
@param
@return
@note
**************************************************/

void netRequestClear(uint32_t req)
{
	sysinfo.netRequest &= ~req;
	//LogPrintf(DEBUG_ALL, "netRequestClear==>0x%04x", req);
}

/**************************************************
@bref		网络请求获取
@param
@return
@note
**************************************************/

uint8_t netRequestGet(uint32_t req)
{
	return (sysinfo.netRequest & req);
}

/**************************************************
@bref		获取除了req以外的开模组请求
@param
@return
@note
**************************************************/

uint8_t netRequestOtherGet(uint32_t req)
{
	uint32_t request;
	request = sysinfo.netRequest;
	request &= ~req;
	LogPrintf(DEBUG_ALL, "netRequestOtherGet==>0x%04x", request & NET_REQUEST_ALL);
	return (request & NET_REQUEST_ALL);
}

/**************************************************
@bref		更新模组工作状态
@param
@return
@note
sysinfo.outBleFenceFlag优先级高于sysinfo.outWifiFenceFlag

**************************************************/

void updateModuleStatus(void)
{
	/* 出蓝牙 */
	if (sysinfo.outBleFenceFlag)
	{	
		//等待扫描结果
		if (wifiRequestGet(DEV_EXTEND_OF_FENCE) == 0)
		{
			/* 且出WIFI */
			if (sysinfo.outWifiFenceFlag)
			{
				if (sysinfo.safeAreaFlag == SAFE_AREA_IN)
				{
					LogPrintf(DEBUG_ALL, "离开安全围栏");
					alarmRequestSet(ALARM_LEAVESAFEAREA_REQUEST);
				}
				sysinfo.safeAreaFlag = SAFE_AREA_OUT;
				netRequestClear(NET_REQUEST_OFFLINE);
				netRequestSet(NET_REQUEST_KEEPNET_CTL);
			}
			/* 没出WIFI */
			else
			{
				if (sysinfo.safeAreaFlag == SAFE_AREA_OUT)
				{
					alarmRequestSet(ALARM_ENTERSAFEAREA_REQUEST);
				}
				sysinfo.safeAreaFlag = SAFE_AREA_IN;
				netRequestClear(NET_REQUEST_KEEPNET_CTL);
				netRequestSet(NET_REQUEST_OFFLINE);
			}
		}
	}
	/* 进蓝牙 */
	else
	{
		if (sysinfo.safeAreaFlag == SAFE_AREA_OUT)
		{
			LogPrintf(DEBUG_ALL, "进入安全围栏");
			alarmRequestSet(ALARM_ENTERSAFEAREA_REQUEST);
		}
		sysinfo.safeAreaFlag = SAFE_AREA_IN;
		netRequestClear(NET_REQUEST_KEEPNET_CTL);
		netRequestClear(NET_REQUEST_OFFLINE);
		netRequestClear(NET_REQUEST_WIFI_CTL);
	}

	/* 如果模组不开启,顺便关闭GPS */
	if (netRequestGet(NET_REQUEST_CONNECT_ONE) == 0 && 
		netRequestGet(NET_REQUEST_KEEPNET_CTL) == 0 &&
		netRequestGet(NET_REQUEST_ALARM_ONE) == 0)
	{
		if (gpsRequestGet(GPS_REQUEST_ALL))
		{
			LogPrintf(DEBUG_ALL, "No net request,close gps");
			gpsRequestClear(GPS_REQUEST_ALL);
		}
	}
	//没网的话这里会让设备反复起来
	if ((netRequestGet(NET_REQUEST_OFFLINE) || netRequestGet(NET_REQUEST_KEEPNET_CTL)) && sysinfo.noNetFlag == 0)
	{
		if (sysinfo.kernalRun == 0)
		{
			volCheckRequestSet();
			changeModeFsm(MODE_START);
			if (netRequestGet(NET_REQUEST_KEEPNET_CTL))
			{
				lbsRequestSet(DEV_EXTEND_OF_MY);
	   	 		gpsRequestSet(GPS_REQUEST_UPLOAD_ONE);
   	 		}
            tmos_set_event(sysinfo.taskId, APP_TASK_RUN_EVENT);
		}
	}
}

/**************************************************
@bref		模组管理
@param
@return
@note
 开模组再开灯和ADC，关模组就关灯ADC，因为开灯功耗比较大
**************************************************/

void netRequestTask(void)
{
	static uint8_t runtick = 0;
	switch (sysinfo.moduleFsm)
	{
		case MODULE_STATUS_CLOSE:
		    if (sysinfo.canRunFlag != 1)
            {
            	break;
            }
			if (sysinfo.netRequest != 0)
			{
				portAdcCfg(1);
				if (isModulePowerOff()) modulePowerOn();
				sysinfo.moduleFsm = MODULE_STATUS_OPEN;
				runtick = 0;
			}

			break;
		case MODULE_STATUS_OPEN:
			/* 如果回到蓝牙围栏则关闭模组 */
		    if (sysinfo.netRequest == 0)
			{
				/* 这里要等于0，如果写大于0 */
				if (runtick++ >= 2)
				{
					
					socketDelAll();
					portAdcCfg(0);
					sysinfo.moduleFsm = MODULE_STATUS_WAIT;
					runtick = 0;
					moduleInit();
				}
			}
			break;
		case MODULE_STATUS_WAIT:
			if (runtick++ >= 3)
			{
				modulePowerOff();
				sysinfo.moduleFsm = MODULE_STATUS_CLOSE;
			}
			break;
		default:
			sysinfo.moduleFsm = MODULE_STATUS_CLOSE;
			break;
	}
}


void petBellTask(void)
{
	static uint8_t fsm = 0;
	
	if (sysinfo.petBellOnoff == 0)
	{
		fsm = 0;
		if (sysinfo.petbellPlaying)
		{
			sysinfo.petbellPlaying = 0;
			stopAudio();
		}
		return;
	}
	if (sysinfo.petSpkCnt > 0 || sysinfo.ttsPlayNow)
	{
		LogPrintf(DEBUG_ALL, "TTS is playing, waiting..");
		return;
	}
	if (netRequestGet(NET_REQUEST_TTS_CTL) == 0)
    {
		return;
    }
    if (isModulePowerOff())
    {
		return;
    }
    /* 设备开机正常 */
    if (getModuleStatus() < CPIN_STATUS)
    {
		return;
    }

	//开机查询音频
    if (sysinfo.petbellPlaying == 0)
    {
		plalAudio(sysparam.musicNum);
		sysinfo.petbellPlaying = 1;
    }
    
}


/**************************************************
@bref		1秒任务
@param
@return
@note
**************************************************/

void taskRunInSecond(void)
{
	sysinfo.sysTick++;
	movingStatusCheck();
	wifiCheckByStep();
	BleFenceCheck();
	updateModuleStatus();
    rebootEveryDay();
    netConnectTask();
    netRequestTask();
    gpsRequestTask();
    voltageCheckTask();
    alarmRequestTask();
    gpsUplodOnePointTask();
    mode123UploadTask();
    lbsRequestTask();
    wifiRequestTask();
    lookForPetSpeakerTask();
    petBellTask();
    autoSleepTask();
    sysModeRunTask();
    serverManageTask();
    outputTTs();
    //systemCloseTask();
}


/**************************************************
@bref		串口调试接收
@param
@return
@note
**************************************************/
void doDebugRecvPoll(uint8_t *msg, uint16_t len)
{
    static uint8_t gpsRestore[DEBUG_BUFF_SIZE + 1];
    static uint16_t size = 0;
    uint16_t i, begin;
    if (len + size > DEBUG_BUFF_SIZE)
    {
        size = 0;
    }
    memcpy(gpsRestore + size, msg, len);
    size += len;
    begin = 0;
    for (i = 0; i < size; i++)
    {
        if (gpsRestore[i] == '\n')
        {
            atCmdParserFunction(gpsRestore + begin, i - begin + 1);
            begin = i + 1;
        }
    }
    if (begin != 0)
    {
        memmove(gpsRestore, gpsRestore + begin, size - begin);
        size -= begin;
    }
}


/*调试专用任务*/
void debugtask(void)
{
    static uint8_t fsm;
    static uint8_t tick;
    switch(fsm)
    {
    case 0:
        modulePowerOn();
        //portGsensorCtl(1);
        fsm = 1;
        break;

    case 1:
        tick++;
        if (tick>=5)
        {
            tick = 0;
        	fsm = 3;
        	modulePowerOff();
//        	moduleSleepCtl(1);
        }
//        else
//        {
//			netConnectTask();
//			serverManageTask();
//        }
        break;
    case 2:
    	LogMessage(DEBUG_ALL, "SHUT DOWN");
        tmos_stop_task(sysinfo.taskId, APP_TASK_POLLUART_EVENT);
        tmos_set_event(sysinfo.taskId, APP_TASK_STOP_EVENT);
		portDebugUartCfg(0);
        portSleepEn();
        fsm = 3;
        break;
    case 3:
        tick++;
        if (tick>=5)
        {
            modulePowerOff();
            tick = 0;
            fsm = 4;
            tmos_stop_task(sysinfo.taskId, APP_TASK_POLLUART_EVENT);
            tmos_set_event(sysinfo.taskId, APP_TASK_STOP_EVENT);
            portSleepEn();
        }
        break;
    }
}

void openuart2(void)
{
	portUartCfg(APPUSART2, 1, 115200, doDebugRecvPoll);
	SYS_POWER_ON;
}

/**************************************************
@bref		系统启动时配置
@param
@return
@note
**************************************************/

void myTaskPreInit(void)
{
	portGpioSetDefCfg();
	
    tmos_memset(&sysinfo, 0, sizeof(sysinfo));
    paramInit();
    sysinfo.logLevel = 9;
    SetSysClock(CLK_SOURCE_PLL_60MHz);

    //portModuleGpioCfg(1);
    portGpsGpioCfg(1);
    portLedGpioCfg(1);
    portAdcCfg(1);
    portWdtCfg();
    portDebugUartCfg(1);
    socketListInit();
    portSleepDn();
	ledStatusUpdate(SYSTEM_LEN_IDLE, 1);
	portSyspwkOffGpioCfg();

    volCheckRequestSet();
    createSystemTask(ledTask, 1);
    createSystemTask(outputNode, 2);
    createSystemTask(keyTask, 1);
    startTimer(10, openuart2, 0);
    portSpkGpioCfg(0);

    sysinfo.sysTaskId = createSystemTask(taskRunInSecond, 10);
	LogMessage(DEBUG_ALL, ">>>>>>>>>>>>>>>>>>>>>>");
    LogPrintf(DEBUG_ALL, "SYS_GetLastResetSta:%x", SYS_GetLastResetSta());
   	addCmdTTS(TTS_STARTUP);
    /* 开机保持长连接，3分钟内有连着蓝牙就断网，没连着蓝牙就继续长连接 */
	netRequestSet(NET_REQUEST_KEEPNET_CTL);
	/* 开机搜一次WIFI */
	wifiRequestSet(DEV_EXTEND_OF_FENCE);
	/* 开机定位一次 */
	gpsRequestSet(GPS_REQUEST_UPLOAD_ONE);
	/* 初始化状态 */
	sysinfo.outBleFenceFlag  = 1;
	sysinfo.outWifiFenceFlag = 1;
	sysinfo.safeAreaFlag     = SAFE_AREA_IN;
}

/**************************************************
@bref		tmos 任务回调
@param
@return
@note
**************************************************/

static tmosEvents myTaskEventProcess(tmosTaskID taskID, tmosEvents events)
{
	uint8_t ret;
    if (events & SYS_EVENT_MSG)
    {
        uint8 *pMsg;
        if ((pMsg = tmos_msg_receive(sysinfo.taskId)) != NULL)
        {
            tmos_msg_deallocate(pMsg);
        }
        return (events ^ SYS_EVENT_MSG);
    }
    if (events & APP_TASK_CLOSE_MODULE_EVENT)
    {
		modulePowerOff();
        return events ^ APP_TASK_CLOSE_MODULE_EVENT;
    }
    if (events & APP_TASK_RESET_EVENT)
    {
    	portSysReset();
        return events ^ APP_TASK_RESET_EVENT;
    }

    if (events & APP_TASK_KERNAL_EVENT)
    {
        kernalRun();
        portWdtFeed();
        return events ^ APP_TASK_KERNAL_EVENT;
    }

    if (events & APP_TASK_POLLUART_EVENT)
    {
        pollUartData();
        portWdtFeed();
        return events ^ APP_TASK_POLLUART_EVENT;
    }

    if (events & APP_TASK_RUN_EVENT)
    {
    	portDebugUartCfg(1);
        LogMessage(DEBUG_ALL, "Task kernal start");
        sysinfo.kernalRun = 1;
        /*重新配置IO*/
		portModuleGpioCfg(1);
		portGpsGpioCfg(1);
		portAdcCfg(1);
		portWdtCfg();
        tmos_start_reload_task(sysinfo.taskId, APP_TASK_KERNAL_EVENT, MS1_TO_SYSTEM_TIME(100));
        return events ^ APP_TASK_RUN_EVENT;
    }
    if (events & APP_TASK_STOP_EVENT)
    {
    	portDebugUartCfg(1);
        LogMessage(DEBUG_ALL, "Task kernal stop");
        sysinfo.kernalRun = 0;
        motionClear();
        /*关闭所有IO*/
		portAdcCfg(0);
		portModuleGpioCfg(0);
		portGpsGpioCfg(0);
		//portSpkGpioCfg(0);
		//portWdtCancel();
       	//tmos_stop_task(sysinfo.taskId, APP_TASK_KERNAL_EVENT);
        portDebugUartCfg(0);
        return events ^ APP_TASK_STOP_EVENT;
    }
    if (events & APP_TASK_ONEMINUTE_EVENT)
    {
   	 	portDebugUartCfg(1);
    	sysAutoReq();
    	calculateNormalTime();
        LogMessage(DEBUG_ALL, "*****************************Task one minutes****************************");
        connectionInfoStruct *devInfo;
		devInfo = getBeaconInfoAll();
		LogPrintf(DEBUG_BLE, "dev[0]sockFlag:%d dev[1]sockFlag:%d ", devInfo[0].sockFlag, devInfo[1].sockFlag);
		LogPrintf(DEBUG_BLE, "Master sn:%s", sysinfo.masterSn);
		LogPrintf(DEBUG_BLE, "dev[0]upTick:%d dev[1]Uptick:%d ", sysinfo.sysTick-devInfo[0].updateTick, sysinfo.sysTick - devInfo[1].updateTick);
		LogPrintf(DEBUG_ALL, "fsm:%d gps:%x alm:%x wifi:%x lbs:%x net:%x outble:%x outwifi:%x", 
				sysinfo.runFsm, 		 sysinfo.gpsRequest, 
				sysinfo.alarmRequest,    sysinfo.wifiExtendEvt,
				sysinfo.lbsRequest, 	 sysinfo.netRequest, 
				sysinfo.outBleFenceFlag, sysinfo.outWifiFenceFlag);
//        LogPrintf(DEBUG_ALL,  "*Mode: %d, mode4run: %d outWifiTick: %d inWifiTick: %d alreadystep:%d runstep:%d modulestate:%d*", sysparam.MODE, sysinfo.mode4SysMin, sysinfo.outWifiTick,sysinfo.inWifiTick,sysinfo.alreadystep,sysinfo.runningstep, getModuleStatus());
        LogMessage(DEBUG_ALL, "*************************************************************************");
        portDebugUartCfg(0);
        return events ^ APP_TASK_ONEMINUTE_EVENT;
    }

    return 0;
}

/**************************************************
@bref		任务初始化
@param
@return
@note
**************************************************/

void myTaskInit(void)
{
    sysinfo.taskId = TMOS_ProcessEventRegister(myTaskEventProcess);
    LogPrintf(DEBUG_ALL, "kernal start==>%s, %d", __FUNCTION__, __LINE__);
    tmos_set_event(sysinfo.taskId, APP_TASK_RUN_EVENT);
    tmos_start_reload_task(sysinfo.taskId, APP_TASK_POLLUART_EVENT, MS1_TO_SYSTEM_TIME(50));
    tmos_start_reload_task(sysinfo.taskId, APP_TASK_ONEMINUTE_EVENT, MS1_TO_SYSTEM_TIME(60000));
}

