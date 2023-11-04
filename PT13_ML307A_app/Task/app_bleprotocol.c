#include "app_bleprotocol.h"
#include "app_peripheral.h"
#include "app_param.h"
#include "app_net.h"
#include "app_port.h"
#include "app_task.h"


/*
 * 0C 03 01 01 05 0D
 * 0C 02 01 03 0D
 *
 * 0C 01 01 02 0D
 * 0C 02 01 02 05 0D
 * 0C 03 01 02 03 0B 0D
 */

uint8_t appPackProtocol(uint8_t *dest, uint8_t cmd, uint8_t *data, uint8_t len)
{
    uint8_t p;
    uint8_t i;
    uint8_t LRC;
    p = 0;
    dest[p++] = 0x0C; //Header
    dest[p++] = len + 1;
    dest[p++] = cmd;
    if (data != NULL && len > 0)
    {
        for (i = 0; i < len; i++)
        {
            dest[p++] = data[i];
        }
    }

    LRC = 0;
    for (i = 1; i < p; i++)
    {
        LRC += dest[i];
    }
    //LRC
    dest[p++] = LRC;
    dest[p++] = 0x0D;
    dest[p] = 0;
    return p;
}

static void appRespon(uint16_t connhandle, uint8 cmd, uint8 result)
{
    uint8 dest[20], data[2], destlen;
    data[0] = cmd;
    data[1] = result;
    destlen = appPackProtocol(dest, CMD_GENERAL_RESPON, data, 2);
    appSendNotifyData(connhandle, dest, destlen);
}


/*
 * 0C 03 01 02 03 0B 0D
 * 0C 01 01 02 0D
 */

void bleProtocolParser(uint16_t connhandle, uint8_t *data, uint16_t len)
{
	uint8_t readInd, size, crc, i;
	uint8_t password[6] = { 0 };
	int8_t ind = -1;
	connectionInfoStruct *devinfo;
	ind = getBeaconIdByHandle(connhandle);
	if (ind < 0)
	{
		LogPrintf(DEBUG_ALL, "bleProtocolParser==>can not find this dev, conhandle:%d", connhandle);
		return;
	}
	devinfo = getBeaconInfoByIndex(ind);
	
	for (readInd = 0; readInd < len; readInd++)
	{
		if (data[readInd] != 0x0C)
		{
			continue;
		}
		if (readInd + 4 >= len)
		{
			//内容超长了
			break;
		}
		size = data[readInd + 1];
		if (readInd + 3 + size >= len)
		{
			continue;
		}
		if (data[readInd + 3 + size] != 0x0D)
		{
			continue;
		}
		crc = 0;
		for (i = 0; i < (size + 1); i++)
		{
			crc += data[readInd + 1 + i];
		}
		if (crc != data[readInd + size + 2])
		{
			continue;
		}
		//LogPrintf(DEBUG_ALL, "CMD[0x%02X]", data[readInd + 3]);
		/*状态更新*/
		devinfo->updateTick = sysinfo.sysTick;
		switch (data[readInd + 2])
        {
        	case CMD_OTA:
//            appRespon(CMD_OTA, 1);
//            tmos_start_task(myAppTaskId, MY_APP_OTA_EVENT, MS1_TO_SYSTEM_TIME(100));
           		break;
       		//鉴权
        	case CMD_AUTH_CMD:
				byteToHexString(data + readInd + 3, password, 6);
				password[6] = 0;
				if (tmos_memcmp(password, sysparam.blePsw, 6) == TRUE)
				{
					appBeaconAuthSuccess(connhandle);
					appRespon(connhandle, data[readInd + 2], 1);
					LogMessage(DEBUG_ALL, "Authentication success");
				}
				else
				{
					appRespon(connhandle, data[readInd + 2], 0);
					LogPrintf(DEBUG_ALL, "Authentication fail:%s  %s", password, sysparam.blePsw);
				}
        		break;
        	case CMD_ONLINE_CTL_CMD:
				if (data[readInd + 3] == 0x01)
				{
					portGsensorCtl(1);
					sysinfo.mode123Min = data[readInd + 4];
					lbsRequestSet(DEV_EXTEND_OF_MY);
					gpsRequestSet(GPS_REQUEST_123_CTL | GPS_REQUEST_UPLOAD_ONE);
					netRequestSet(NET_REQUEST_KEEPNET_CTL);
					sysinfo.flag123 = 1;
				    save123InstructionId();
					LogPrintf(DEBUG_ALL, "GPS forced work %d min, and reporting every 10 seconds", sysinfo.mode123Min);
				}
				else
				{
					lbsRequestClear();
					netRequestClear(NET_REQUEST_KEEPNET_CTL | NET_REQUEST_WIFI_CTL);
					gpsRequestClear(NET_REQUEST_ALL);
					sysinfo.mode123Min = 0;
					sysinfo.flag123 = 0;
					LogPrintf(DEBUG_ALL, "GPS forced work cancel");
				}
				appRespon(connhandle, data[readInd + 2], 1);
        		break;
        		//0C 02 B1 04 B7 0D  MODE4
        		//0C 02 B1 02 B4 0D  MODE2
        	case CMD_MODE_CMD:
				switch (data[readInd + 3])
				{
					case MODE1:
						gpsRequestSet(GPS_REQUEST_UPLOAD_ONE);
						sysparam.MODE = MODE1;
						break;
					case MODE2:
						gpsRequestSet(GPS_REQUEST_UPLOAD_ONE);
						sysparam.MODE = MODE2;
						break;
					case MODE3:
						gpsRequestSet(GPS_REQUEST_UPLOAD_ONE);
						sysparam.MODE = MODE3;
						break;
					case MODE4:
						sysparam.MODE = MODE4;
						break;
					case MODE21:
						gpsRequestSet(GPS_REQUEST_UPLOAD_ONE);
						sysparam.MODE = MODE21;
						break;
					case MODE23:
						gpsRequestSet(GPS_REQUEST_UPLOAD_ONE);
						sysparam.MODE = MODE23;
						break;
					default:
						gpsRequestSet(GPS_REQUEST_UPLOAD_ONE);
						sysparam.MODE = MODE2;
						break;
				}
				LogPrintf(DEBUG_ALL, "Change to mode%d", sysparam.MODE);
				paramSaveAll();
				appRespon(connhandle, data[readInd + 2], 1);
        		break;
        	case CMD_DEV_LOGIN_INFO:
				bleSendLoginInfo(connhandle);
        		break;
        	case CMD_DEV_HEARTBEAT:
				bleSendHbtInfo(connhandle);
        		break;
        	case CMD_DEV_MASTER_INFO:
				appRespon(connhandle, CMD_DEV_MASTER_INFO, 0);
				LogPrintf(DEBUG_ALL, "Master info");
        		break;
        }

	    readInd += size + 3;
    }

}


void appSendShieldAlarm(void)
{
//    uint8 dest[20], data[2], destlen;
//    data[0] = CMD_SEND_SHIELD_ALARM;
//    destlen = appPackProtocol(dest, CMD_GENERAL_RESPON, data, 1);
//    appSendNotifyData(peripheralConInfo.connHandle, dest, destlen);
//    LogMessage(DEBUG_ALL, "send shield alarm");
}

void bleGeneratePsdProtocol(uint8_t *psd)
{
	uint8_t data[3] = {0}, dest[20] = {0}, destlen;
	char debug[50];
	changeHexStringToByteArray(data, psd, 6);
	destlen = appPackProtocol(dest, CMD_AUTH_CMD, data, 3);
	byteToHexString(dest, debug, destlen);
	debug[destlen * 2] = 0;
	LogPrintf(DEBUG_ALL, "password[%d]:%s", destlen, debug);
}

void bleSendLoginInfo(uint16_t connhandle)
{
	uint8_t data[60] = { 0 }, dest[60] = { 0 };
	uint8_t destlen = 0, i;
	char debug[120] = { 0 };
	connectionInfoStruct *devinfo = NULL;
	devinfo = getBeaconInfoByHandle(connhandle);
	if (devinfo == NULL)
	{
		LogPrintf(DEBUG_BLE, "Can not find ble info, connhandle:%d", connhandle);
		return;
	}
	data[destlen++] = CMD_DEV_LOGIN_INFO;
	/* 设备号 */
	for (i = 0; i < 15; i++)
	{
		data[destlen++] = dynamicParam.SN[i];
	}
	/* sock id */
	data[destlen++] = devinfo->login;
	destlen = appPackProtocol(dest, CMD_GENERAL_RESPON, data, destlen);
	appSendNotifyData(connhandle, dest, destlen);
	byteToHexString(dest, debug, destlen);
	debug[destlen * 2] = 0;
	LogPrintf(DEBUG_ALL, "Login Info[%d]:%s", destlen, debug);
}

void bleSendHbtInfo(uint16_t connhandle)
{
	uint8_t destlen = 0, i;
	uint8_t data[60] = { 0 }, dest[60] = { 0 };
	uint16_t value;
	char debug[120] = { 0 };
	connectionInfoStruct *devinfo = NULL;
	devinfo = getBeaconInfoByHandle(connhandle);
	if (devinfo == NULL)
	{
		LogPrintf(DEBUG_BLE, "Can not find ble info, connhandle:%d", connhandle);
		return;
	}
	data[destlen++] = CMD_DEV_HEARTBEAT;
	/* 外电 */
	value = (uint16_t)(sysinfo.outsidevoltage * 10);
	data[destlen++] = (value >> 8) & 0xFF;
	data[destlen++] = value & 0xFF;
	/* 电量 */
	value = getBatteryLevel();
	data[destlen++] = value & 0xFF;
	/* 步数 */
	value = dynamicParam.step;
	data[destlen++] = (value >> 8) & 0xFF;
	data[destlen++] = value & 0xFF;
	destlen = appPackProtocol(dest, CMD_GENERAL_RESPON, data, destlen);
	appSendNotifyData(connhandle, dest, destlen);
//	if (appSendNotifyData(connhandle, dest, destlen) == SUCCESS)
//	{
//		appBeaconSockflagSet(index);
//	}
	byteToHexString(dest, debug, destlen);
	debug[destlen * 2] = 0;
	LogPrintf(DEBUG_ALL, "SendHbtInfo[%d]:%s", destlen, debug);
}

