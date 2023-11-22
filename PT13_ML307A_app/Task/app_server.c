/*
 * app_server.c
 *
 *  Created on: Jul 14, 2022
 *      Author: idea
 */
#include "app_server.h"
#include "app_kernal.h"
#include "app_net.h"
#include "app_protocol.h"
#include "app_param.h"
#include "app_sys.h"
#include "app_socket.h"
#include "app_gps.h"
#include "app_db.h"
#include "app_jt808.h"
#include "app_task.h"
#include "app_central.h"

static netConnectInfo_s privateServConn, hiddenServConn;
static jt808_Connect_s jt808ServConn;
static agps_connect_s agpsServConn;
static bleInfo_s *bleHead = NULL;
int8_t timeOutId = -1;
static int8_t hbtTimeOutId = -1;

/**************************************************
@bref		模组回复，停止定时器，防止模组死机用
@param
@return
@note
**************************************************/

void moduleRspSuccess(void)
{
    if (timeOutId != -1)
    {
        stopTimer(timeOutId);
        timeOutId = -1;
    }
}
/**************************************************
@bref		心跳回复，停止定时器，防止模组死机用
@param
@return
@note
**************************************************/

void hbtRspSuccess(void)
{
    if (hbtTimeOutId != -1)
    {
        stopTimer(hbtTimeOutId);
        hbtTimeOutId = -1;
    }
}


/**************************************************
@bref		执行复位模组
@param
@return
@note
**************************************************/

void moduleRspTimeout(void)
{
    timeOutId = -1;
	if (sysinfo.runFsm == MODE_RUNING)
	{
   	  moduleReset();
   	  LogMessage(DEBUG_ALL, "moduleRspTimeout");
   	}
}

static void hbtRspTimeOut(void)
{
    LogMessage(DEBUG_ALL, "heartbeat timeout");
    hbtTimeOutId = -1;
	if (sysinfo.runFsm == MODE_RUNING)
		moduleReset();
}


/**************************************************
@bref		联网状态机切换
@param
@return
@note
**************************************************/

static void privateServerChangeFsm(NetWorkFsmState state)
{
    privateServConn.fsmstate = state;
}


/**************************************************
@bref		停止定时器，防止模组死机用
@param
@return
@note
**************************************************/

void privateServerReconnect(void)
{
    LogMessage(DEBUG_ALL, "private reconnect server");
    socketDel(NORMAL_LINK);
    socketSetConnState(0, SOCKET_CONN_ERR);
    moduleSleepCtl(0);
}






/**************************************************
@bref		服务器断开
@param
@return
@note
**************************************************/

void privateServerDisconnect(void)
{
    privateServerChangeFsm(SERV_LOGIN);
}
/**************************************************
@bref		主服务器登录正常
@param
@return
@note
**************************************************/

void privateServerLoginSuccess(void)
{
    privateServConn.loginCount = 0;
    privateServConn.heartbeattick = 0;
    moduleSleepCtl(1);
    ledStatusUpdate(SYSTEM_LED_NETOK, 1);
    privateServerChangeFsm(SERV_READY);
	//scanListUpload();
}
/**************************************************
@bref		socket数据接收
@param
@return
@note
**************************************************/

static void privateServerSocketRecv(char *data, uint16_t len)
{
    uint16_t i, beginindex, contentlen, lastindex;
    //遍历，寻找协议头
    for (i = 0; i < len; i++)
    {
        beginindex = i;
        if (data[i] == 0x78)
        {
            if (i + 1 >= len)
            {
                continue ;
            }
            if (data[i + 1] != 0x78)
            {
                continue ;
            }
            if (i + 2 >= len)
            {
                continue ;
            }
            contentlen = data[i + 2];
            if ((i + 5 + contentlen) > len)
            {
                continue ;
            }
            if (data[i + 3 + contentlen] == 0x0D && data[i + 4 + contentlen] == 0x0A)
            {
                i += (4 + contentlen);
                lastindex = i + 1;
                //LogPrintf(DEBUG_ALL, "Fint it ====>Begin:7878[%d,%d]", beginindex, lastindex - beginindex);
                protocolRxParser(NORMAL_LINK, (char *)data + beginindex, lastindex - beginindex);
            }
        }
        else if (data[i] == 0x79)
        {
            if (i + 1 >= len)
            {
                continue ;
            }
            if (data[i + 1] != 0x79)
            {
                continue ;
            }
            if (i + 3 >= len)
            {
                continue ;
            }
            contentlen = data[i + 2] << 8 | data[i + 3];
            if ((i + 6 + contentlen) > len)
            {
                continue ;
            }
            if (data[i + 4 + contentlen] == 0x0D && data[i + 5 + contentlen] == 0x0A)
            {
                i += (5 + contentlen);
                lastindex = i + 1;
                //LogPrintf(DEBUG_ALL, "Fint it ====>Begin:7979[%d,%d]", beginindex, lastindex - beginindex);
                protocolRxParser(NORMAL_LINK, (char *)data + beginindex, lastindex - beginindex);
            }
        }
    }
}
/**************************************************
@bref		主服务器连接任务
@param
@return
@note
**************************************************/

void privateServerConnTask(void)
{
    static uint16_t unLoginTick = 0;
	static uint8 uploadflag = 0;
    if (isModuleRunNormal() == 0)
    {
    	uploadflag = 0;
        ledStatusUpdate(SYSTEM_LED_NETOK, 0);
        if (socketGetUsedFlag(NORMAL_LINK) == 1)
        {
            socketDel(NORMAL_LINK);
        }
        privateServerChangeFsm(SERV_LOGIN);
        return ;
    }
    if (socketGetUsedFlag(NORMAL_LINK) != 1)
    {
        ledStatusUpdate(SYSTEM_LED_NETOK, 0);
        privateServerChangeFsm(SERV_LOGIN);
        socketAdd(NORMAL_LINK, sysparam.Server, sysparam.ServerPort, privateServerSocketRecv);
        return;
    }
    if (socketGetConnStatus(NORMAL_LINK) != SOCKET_CONN_SUCCESS)
    {
    	uploadflag = 0;
        ledStatusUpdate(SYSTEM_LED_NETOK, 0);
        privateServerChangeFsm(SERV_LOGIN);
        if (unLoginTick++ >= 900)
        {
            unLoginTick = 0;
            moduleReset();
        }
        return;
    }
    switch (privateServConn.fsmstate)
    {
        case SERV_LOGIN:
            unLoginTick = 0;
            if (strncmp(dynamicParam.SN, "888888887777777", 15) == 0)
            {
                LogMessage(DEBUG_ALL, "no Sn");
                return;
            }

            LogMessage(DEBUG_ALL, "Login to server...");
            protocolSnRegister(dynamicParam.SN);
            protocolSend(NORMAL_LINK, PROTOCOL_01, NULL);
            protocolSend(NORMAL_LINK, PROTOCOL_F1, NULL);
            protocolSend(NORMAL_LINK, PROTOCOL_8A, NULL);
            privateServerChangeFsm(SERV_LOGIN_WAIT);
            privateServConn.logintick = 0;
            break;
        case SERV_LOGIN_WAIT:
            privateServConn.logintick++;
            if (privateServConn.logintick >= 60)
            {
                privateServerChangeFsm(SERV_LOGIN);
                privateServConn.loginCount++;
                privateServerReconnect();
                if (privateServConn.loginCount >= 3)
                {
                    privateServConn.loginCount = 0;
                    moduleReset();
                }
            }
            break;
        case SERV_READY:
            if (privateServConn.heartbeattick % (sysparam.heartbeatgap - 2) == 0)
            {
                queryBatVoltage();
                moduleGetCsq();
            }
            if (privateServConn.heartbeattick % sysparam.heartbeatgap == 0)
            {
                privateServConn.heartbeattick = 0;
                if (hbtTimeOutId == -1)
                {
                    hbtTimeOutId = startTimer(1800, hbtRspTimeOut, 0);
                }
                protocolInfoResiter(getBatteryLevel(), sysinfo.outsidevoltage > 5.0 ? sysinfo.outsidevoltage : sysinfo.insidevoltage,
                                    dynamicParam.startUpCnt, dynamicParam.runTime);
                protocolSend(NORMAL_LINK, PROTOCOL_13, NULL);
                uploadflag = 1;
                sysinfo.hbtFlag = 1;
            }
            privateServConn.heartbeattick++;
            if (getTcpNack())
            {
                querySendData(NORMAL_LINK);
            }

            if (sysinfo.mode123GpsFre == 0)
            {
            	if (privateServConn.heartbeattick % 2 == 0)
	            {
	                //传完ram再传文件系统
	                if (getTcpNack() == 0)
	                {
	                    if (dbUpload() == 0)
	                    {
	                        gpsRestoreUpload();
	                    }
	                }
	            }
            }
            else 
            {
				if (uploadflag)
				{
					if (privateServConn.heartbeattick % 2 == 0)
					{
						LogMessage(DEBUG_ALL, "111");
						if (dbUpload() == 0)
	                    {
	                    	LogMessage(DEBUG_ALL, "222");
	                        if (gpsRestoreUpload() == 0)
	                        {
								uploadflag = 0;
								LogPrintf(DEBUG_ALL, "打包上传完成");
	                        }
	                    }
	                }
				}
            }
           
            break;
        default:
            privateServConn.fsmstate = SERV_LOGIN;
            privateServConn.heartbeattick = 0;
            break;
    }
}


static uint8_t hiddenServCloseChecking(void)
{
    if (sysparam.hiddenServOnoff == 0)
    {
        return 1;
    }
    if (sysparam.protocol == ZT_PROTOCOL_TYPE)
    {
        if (sysparam.ServerPort == sysparam.hiddenPort)
        {
            if (strcmp(sysparam.Server, sysparam.hiddenServer) == 0)
            {
                //if use the same server and port ,abort use hidden server.
                return	1;
            }
        }
    }
    if (sysinfo.hiddenServCloseReq)
    {
        //it is the system request of close hidden server,maybe the socket error.
        return 1;
    }
    return 0;
}

static void hiddenServerChangeFsm(NetWorkFsmState state)
{
    hiddenServConn.fsmstate = state;
}

/**************************************************
@bref		socket数据接收
@param
@return
@note
**************************************************/

static void hiddenServerSocketRecv(char *data, uint16_t len)
{
    protocolRxParser(HIDDEN_LINK, data, len);
}

/**************************************************
@bref		隐藏服务器登录回复
@param
	none
@return
	none
@note
**************************************************/

void hiddenServerLoginSuccess(void)
{
    hiddenServConn.loginCount = 0;
    hiddenServConn.heartbeattick = 0;
    hiddenServerChangeFsm(SERV_READY);
}

/**************************************************
@bref		请求关闭隐藏链路
@param
@return
@note
**************************************************/

void hiddenServerCloseRequest(void)
{
    sysinfo.hiddenServCloseReq = 1;
    LogMessage(DEBUG_ALL, "hidden serv close request");
}


/**************************************************
@bref		清除关闭隐藏链路的请求
@param
@return
@note
**************************************************/

void hiddenServerCloseClear(void)
{
    sysinfo.hiddenServCloseReq = 0;
    LogMessage(DEBUG_ALL, "hidden serv close clear");
}

/**************************************************
@bref		隐藏服务器连接任务
@param
@return
@note
**************************************************/

static void hiddenServerConnTask(void)
{
    if (isModuleRunNormal() == 0)
    {
    	if (socketGetUsedFlag(HIDDEN_LINK) == 1)
        {
            LogMessage(DEBUG_ALL, "hidden server abort");
            socketDel(HIDDEN_LINK);
        }
        hiddenServerChangeFsm(SERV_LOGIN);
        return ;
    }

    if (hiddenServCloseChecking())
    {
        if (socketGetUsedFlag(HIDDEN_LINK) == 1)
        {
            LogMessage(DEBUG_ALL, "hidden server abort");
            socketDel(HIDDEN_LINK);
        }
        hiddenServerChangeFsm(SERV_LOGIN);
        return;
    }
    if (socketGetUsedFlag(HIDDEN_LINK) != 1)
    {
        hiddenServerChangeFsm(SERV_LOGIN);
        socketAdd(HIDDEN_LINK, sysparam.hiddenServer, sysparam.hiddenPort, hiddenServerSocketRecv);
        return;
    }
    if (socketGetConnStatus(HIDDEN_LINK) != SOCKET_CONN_SUCCESS)
    {
        hiddenServerChangeFsm(SERV_LOGIN);
        return;
    }
    switch (hiddenServConn.fsmstate)
    {
        case SERV_LOGIN:
            LogMessage(DEBUG_ALL, "Login to server...");
            protocolSnRegister(dynamicParam.SN);
            protocolSend(HIDDEN_LINK, PROTOCOL_01, NULL);
            hiddenServerChangeFsm(SERV_LOGIN_WAIT);
            hiddenServConn.logintick = 0;
            break;
        case SERV_LOGIN_WAIT:
            hiddenServConn.logintick++;
            if (hiddenServConn.logintick >= 60)
            {
                hiddenServerChangeFsm(SERV_LOGIN);
                hiddenServConn.loginCount++;
                privateServerReconnect();
                if (hiddenServConn.loginCount >= 3)
                {
                    hiddenServConn.loginCount = 0;
                    hiddenServerCloseRequest();
                }
            }
            break;
        case SERV_READY:
            if (hiddenServConn.heartbeattick % (sysparam.heartbeatgap - 2) == 0)
            {
                queryBatVoltage();
                moduleGetCsq();
            }
            if (hiddenServConn.heartbeattick % sysparam.heartbeatgap == 0)
            {
                hiddenServConn.heartbeattick = 0;
//                if (timeOutId == -1)
//                {
//                    timeOutId = startTimer(80, moduleRspTimeout, 0);
//                }
                protocolInfoResiter(getBatteryLevel(), sysinfo.outsidevoltage > 5.0 ? sysinfo.outsidevoltage : sysinfo.insidevoltage,
                                    dynamicParam.startUpCnt, dynamicParam.runTime);
                protocolSend(HIDDEN_LINK, PROTOCOL_13, NULL);
            }
            hiddenServConn.heartbeattick++;

            break;
        default:
            hiddenServConn.fsmstate = SERV_LOGIN;
            hiddenServConn.heartbeattick = 0;
            break;
    }
}



/**************************************************
@bref		jt808状态机切换状态
@param
	nfsm	新状态
@return
	none
@note
**************************************************/

static void jt808ServerChangeFsm(jt808_connfsm_s nfsm)
{
    jt808ServConn.connectFsm = nfsm;
    jt808ServConn.runTick = 0;
}

/**************************************************
@bref		jt808数据接收回调
@param
	none
@return
	none
@note
**************************************************/

static void jt808ServerSocketRecv(char *rxbuf, uint16_t len)
{
    jt808ReceiveParser((uint8_t *)rxbuf, len);
}

/**************************************************
@bref		数据发送接口
@param
	none
@return
	1		发送成功
	!=1		发送失败
@note
**************************************************/

static int jt808ServerSocketSend(uint8_t link, uint8_t *data, uint16_t len)
{
    int ret;
    ret = socketSendData(link, data, len);
    return 1;
}

/**************************************************
@bref		jt808联网状态机
@param
@return
@note
**************************************************/

void jt808ServerReconnect(void)
{
    LogMessage(DEBUG_ALL, "jt808 reconnect server");
    socketDel(JT808_LINK);
    socketSetConnState(2, SOCKET_CONN_ERR);
    moduleSleepCtl(0);
}

/**************************************************
@bref		jt808鉴权成功回复
@param
@return
@note
**************************************************/

void jt808ServerAuthSuccess(void)
{
    jt808ServConn.authCnt = 0;
    moduleSleepCtl(1);
    jt808ServerChangeFsm(JT808_NORMAL);
    ledStatusUpdate(SYSTEM_LED_NETOK, 1);
}

/**************************************************
@bref		jt808服务器连接任务
@param
@return
@note
**************************************************/

void jt808ServerConnTask(void)
{
    static uint8_t ret = 1;
    static uint16_t unLoginTick = 0;
    static uint8 uploadflag = 0;
    if (isModuleRunNormal() == 0)
    {
        ledStatusUpdate(SYSTEM_LED_NETOK, 0);
        if (socketGetUsedFlag(JT808_LINK) == 1)
        {
            socketDel(JT808_LINK);
        }
        jt808ServerChangeFsm(JT808_REGISTER);
        return;
    }
    if (socketGetUsedFlag(JT808_LINK) != 1)
    {
        ledStatusUpdate(SYSTEM_LED_NETOK, 0);
        jt808ServerChangeFsm(JT808_REGISTER);
        jt808RegisterTcpSend(jt808ServerSocketSend);
        jt808RegisterManufactureId((uint8_t *)"ZT");
        jt808RegisterTerminalType((uint8_t *)"06");
        jt808RegisterTerminalId((uint8_t *)"01");
        socketAdd(JT808_LINK, sysparam.jt808Server, sysparam.jt808Port, jt808ServerSocketRecv);
        return;
    }
    if (socketGetConnStatus(JT808_LINK) != SOCKET_CONN_SUCCESS)
    {
        ledStatusUpdate(SYSTEM_LED_NETOK, 0);
        jt808ServerChangeFsm(JT808_REGISTER);
        if (unLoginTick++ >= 900)
        {
            unLoginTick = 0;
            moduleReset();
        }
        return;
    }


    switch (jt808ServConn.connectFsm)
    {
        case JT808_REGISTER:

            if (strcmp((char *)dynamicParam.jt808sn, "888777") == 0)
            {
                LogMessage(DEBUG_ALL, "no JT808SN");
                return;
            }

            if (dynamicParam.jt808isRegister)
            {
                //已注册过的设备不用重复注册
                jt808ServerChangeFsm(JT808_AUTHENTICATION);
                jt808ServConn.regCnt = 0;
            }
            else
            {
                //注册设备
                if (jt808ServConn.runTick % 60 == 0)
                {
                    if (jt808ServConn.regCnt++ > 3)
                    {
                        LogMessage(DEBUG_ALL, "Terminal register timeout");
                        jt808ServConn.regCnt = 0;
                        jt808ServerReconnect();
                    }
                    else
                    {
                        LogMessage(DEBUG_ALL, "Terminal register");
                        jt808RegisterLoginInfo(dynamicParam.jt808sn, dynamicParam.jt808isRegister, dynamicParam.jt808AuthCode, dynamicParam.jt808AuthLen);
                        jt808SendToServer(TERMINAL_REGISTER, NULL);
                    }
                }
                break;
            }
        case JT808_AUTHENTICATION:

            if (jt808ServConn.runTick % 60 == 0)
            {
                ret = 1;
                if (jt808ServConn.authCnt++ > 3)
                {
                    jt808ServConn.authCnt = 0;
                    dynamicParam.jt808isRegister = 0;
                    dynamicParamSaveAll();
                    jt808ServerReconnect();
                    LogMessage(DEBUG_ALL, "Authentication timeout");
                }
                else
                {
                    LogMessage(DEBUG_ALL, "Terminal authentication");
                    jt808ServConn.hbtTick = sysparam.heartbeatgap;
                    jt808RegisterLoginInfo(dynamicParam.jt808sn, dynamicParam.jt808isRegister, dynamicParam.jt808AuthCode, dynamicParam.jt808AuthLen);
                    jt808SendToServer(TERMINAL_AUTH, NULL);
                }
            }
            break;
        case JT808_NORMAL:
            if (++jt808ServConn.hbtTick >= sysparam.heartbeatgap)
            {
                jt808ServConn.hbtTick = 0;
                queryBatVoltage();
                LogMessage(DEBUG_ALL, "Terminal heartbeat");
                jt808SendToServer(TERMINAL_HEARTBEAT, NULL);
                if (hbtTimeOutId == -1)
                {
                    hbtTimeOutId = startTimer(1800, hbtRspTimeOut, 0);
                }
                uploadflag = 1;
            }
            if (getTcpNack())
            {
                querySendData(JT808_LINK);
            }
            if (jt808ServConn.runTick % 3 == 0)
            {
                //传完ram再传文件系统
                if (getTcpNack() == 0)
                {
                    if (dbUpload() == 0)
                    {
                        gpsRestoreUpload();
                    }
                }
            }

            if (sysinfo.mode123GpsFre == 0)
            {
            	if (jt808ServConn.runTick % 2 == 0)
	            {
	                //传完ram再传文件系统
	                if (getTcpNack() == 0)
	                {
	                    if (dbUpload() == 0)
	                    {
	                        gpsRestoreUpload();
	                    }
	                }
	            }
            }
            else 
            {
				if (uploadflag)
				{
					if (jt808ServConn.runTick % 2 == 0)
					{
						LogMessage(DEBUG_ALL, "111");
						if (dbUpload() == 0)
	                    {
	                    	LogMessage(DEBUG_ALL, "222");
	                        if (gpsRestoreUpload() == 0)
	                        {
								uploadflag = 0;
								LogPrintf(DEBUG_ALL, "打包上传完成");
	                        }
	                    }
	                }
				}
            }
            break;
        default:
            jt808ServerChangeFsm(JT808_AUTHENTICATION);
            break;
    }
    jt808ServConn.runTick++;
}

/**************************************************
@bref		agps请求
@param
	none
@return
	none
@note
**************************************************/

void agpsRequestSet(void)
{
    sysinfo.agpsRequest = 1;
    LogMessage(DEBUG_ALL, "agpsRequestSet==>OK");
}

void agpsRequestClear(void)
{
    sysinfo.agpsRequest = 0;
    
}
/**************************************************
@bref		socket数据接收
@param
@return
@note
**************************************************/

static void agpsSocketRecv(char *data, uint16_t len)
{
    LogPrintf(DEBUG_ALL, "Agps Reject %d Bytes", len);
    //LogMessageWL(DEBUG_ALL, data, len);
    portUartSend(&usart3_ctl, data, len);
}

static void agpsServerConnTask(void)
{
	static uint8_t agpsFsm = 0;
	static uint8_t runTick = 0;
	char agpsBuff[150] = {0};
	uint16_t agpsLen;
	int ret;
	gpsinfo_s *gpsinfo;

	if (sysparam.agpsen == 0)
	{
		sysinfo.agpsRequest = 0;
		agpsFsm = 0;
		if (socketGetUsedFlag(AGPS_LINK))
		{
			socketDel(AGPS_LINK);
		}
		return;
	}
	if (sysinfo.agpsRequest == 0)
	{
		agpsFsm = 0;
		if (socketGetUsedFlag(AGPS_LINK))
		{
			socketDel(AGPS_LINK);
		}
		return;
	}

	gpsinfo = getCurrentGPSInfo();

	if (isModuleRunNormal() == 0)
	{
		return ;
	}
	if (gpsinfo->fixstatus || sysinfo.gpsOnoff == 0)
	{
		socketDel(AGPS_LINK);
		agpsRequestClear();
		return;
	}
	if (socketGetUsedFlag(AGPS_LINK) != 1)
	{
		agpsFsm = 0;
		ret = socketAdd(AGPS_LINK, sysparam.agpsServer, sysparam.agpsPort, agpsSocketRecv);
		if (ret != 1)
		{
			LogPrintf(DEBUG_ALL, "agps add socket err[%d]", ret);
			agpsRequestClear();
		}
		return;
	}
	if (socketGetConnStatus(AGPS_LINK) != SOCKET_CONN_SUCCESS)
	{
		agpsFsm = 0;
		LogMessage(DEBUG_ALL, "wait agps server ready");
		return;
	}
	switch (agpsFsm)
	{
		case 0:
//			  if (gpsinfo->fixstatus == 0)
//			  {
//				  sprintf(agpsBuff, "user=%s;pwd=%s;cmd=full;", sysparam.agpsUser, sysparam.agpsPswd);
//				  socketSendData(AGPS_LINK, (uint8_t *) agpsBuff, strlen(agpsBuff));
//			  }
			createProtocolA0(agpsBuff, &agpsLen);
			socketSendData(AGPS_LINK, (uint8_t *) agpsBuff, agpsLen);
			agpsFsm = 1;
			runTick = 0;
			break;
		case 1:
			if (++runTick >= 30)
			{
				if (isAgpsDataRecvComplete() == 0)
				{
					agpsFsm = 0;
					runTick = 0;
					socketDel(AGPS_LINK);
					agpsRequestClear();
				}
			}
			break;
		default:
			agpsFsm = 0;
			break;
	}

}


/**************************************************
@bref		服务器链接管理任务
@param
@return
@note
**************************************************/

void serverManageTask(void)
{
    if (sysparam.protocol == JT808_PROTOCOL_TYPE)
    {
        jt808ServerConnTask();
    }
    else
    {
        privateServerConnTask();
    }

    hiddenServerConnTask();
    agpsServerConnTask();
}

/**************************************************
@bref		判断主要服务器是否登录正常
@param
@return
@note
**************************************************/

uint8_t primaryServerIsReady(void)
{
    if (isModuleRunNormal() == 0)
        return 0;
    if (sysparam.protocol == JT808_PROTOCOL_TYPE)
    {
        if (socketGetConnStatus(JT808_LINK) != SOCKET_CONN_SUCCESS)
            return 0;
        if (jt808ServConn.connectFsm != JT808_NORMAL)
            return 0;
    }
    else
    {
        if (socketGetConnStatus(NORMAL_LINK) != SOCKET_CONN_SUCCESS)
            return 0;
        if (privateServConn.fsmstate != SERV_READY)
            return 0;
    }
    return 1;
}

/**************************************************
@bref		判断隐藏服务器是否登录正常
@param
@return
@note
**************************************************/

uint8_t hiddenServerIsReady(void)
{
    if (isModuleRunNormal() == 0)
        return 0;
    if (socketGetConnStatus(HIDDEN_LINK) != SOCKET_CONN_SUCCESS)
        return 0;
    if (hiddenServConn.fsmstate != SERV_READY)
        return 0;
    return 1;
}

