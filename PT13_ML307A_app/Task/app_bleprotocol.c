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
		LogPrintf(DEBUG_BLE, "bleProtocolParser==>can not find this dev, conhandle:%d", connhandle);
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
					LogMessage(DEBUG_BLE, "Authentication success");
				}
				else
				{
					appRespon(connhandle, data[readInd + 2], 0);
					LogPrintf(DEBUG_BLE, "Authentication fail:%s  %s", password, sysparam.blePsw);
				}
        		break;
        	case CMD_DEV_LOGIN_INFO:
				bleSendLoginInfo(connhandle);
        		break;
        	case CMD_DEV_HEARTBEAT:
				bleSendHbtInfo(connhandle);
        		break;
        	case CMD_DEV_MASTER_INFO:
        		bleMasterInfo(connhandle, data + readInd + 4, ind, data[readInd + 3]);
        		
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
	LogPrintf(DEBUG_BLE, "password[%d]:%s", destlen, debug);
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
	/* 本机设备号 */
	for (i = 0; i < 15; i++)
	{
		data[destlen++] = dynamicParam.SN[i];
	}
	/* sock success flag */
	data[destlen++] = sysinfo.sockSuccess;
	/* 主机设备号 */
	if (sysinfo.sockSuccess)
	{
		for (i = 0; i < 15; i++)
		{
			data[destlen++] = sysinfo.masterSn[i];
		}
	}
	destlen = appPackProtocol(dest, CMD_GENERAL_RESPON, data, destlen);
	appSendNotifyData(connhandle, dest, destlen);
	byteToHexString(dest, debug, destlen);
	debug[destlen * 2] = 0;
	LogPrintf(DEBUG_BLE, "Send Login info[%d]:%s", connhandle, debug);
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
	LogPrintf(DEBUG_ALL, "Send HbtInfo[%d]:%s", destlen, debug);
}

void bleMasterInfo(uint16_t connhandle, char *sn, uint8_t ind, uint8_t socksuccess)
{
	uint8_t destlen = 0, i, ret = 0;
	uint8_t data[60] = { 0 }, dest[60] = { 0 };
	char debug[121] = { 0 };
	connectionInfoStruct *devinfo = NULL;
	devinfo = getBeaconInfoByHandle(connhandle);
	if (devinfo == NULL)
	{
		LogPrintf(DEBUG_BLE, "Can not find ble info, connhandle:%d", connhandle);
		return;
	}
	data[destlen++] = CMD_DEV_MASTER_INFO;

	/* 录入主机信息 */
//	if (socksuccess)
//	{
//		if (sysinfo.sockSuccess == 0)
//		{
//			for (uint8_t j = 0; j < 15; j++)
//			{
//				sysinfo.masterSn[j++] = sn[j++];
//			}
//			sysinfo.masterSn[15] = 0;
//			sysinfo.sockSuccess = 1;
//			ret = 1;
//			LogPrintf(DEBUG_BLE, "MasterInfo==>master[%s] rigister ok", sysinfo.masterSn);
//		}
//		else
//		{
//			if (strncmp(sn, sysinfo.masterSn, 15) == 0)
//			{
//				ret = 1;
//				sysinfo.sockSuccess = 1;
//				LogPrintf(DEBUG_BLE, "MasterInfo==>Same mastersn[%s], rigister ok", sysinfo.masterSn);
//			}
//			else
//			{
//				ret = 0;
//				LogPrintf(DEBUG_BLE, "MasterInfo==>Other link,master[%s]rigister fail", sysinfo.masterSn);
//			}
//		}
//	}
//	/*  */
//	else
//	{
//		ret = 0;
//		LogPrintf(DEBUG_BLE, "MasterInfo==>master[%s]rigister fail", sysinfo.masterSn);
//	}

	/* 已注册的主机拥有修改socksuccess权 */
	if (strncmp(sn, sysinfo.masterSn, 15) == 0)
	{
		ret = 1;
		sysinfo.sockSuccess = socksuccess;
		LogPrintf(DEBUG_BLE, "MasterInfo==>Same mastersn[%s %s], rigister ok", sysinfo.masterSn);
		appBeaconSockflagSet(socksuccess, ind);
	}
	/* 非自己主机只能注册，不能注销socksuccess */
	else
	{
		if (sysinfo.sockSuccess == 0)
		{
			if (socksuccess)
			{
				sysinfo.sockSuccess = 1;
				for (uint8_t j = 0; j < 15; j++)
				{
					sysinfo.masterSn[j] = sn[j];
				}
				sysinfo.masterSn[15] = 0;
				LogPrintf(DEBUG_BLE, "MasterInfo==>master[%s] rigister ok", sysinfo.masterSn);
				ret = 1;
				appBeaconSockflagSet(socksuccess, ind);
			}
			else
			{
				LogPrintf(DEBUG_BLE, "MasterInfo==>master[%s]can not link,rigister fail", sysinfo.masterSn);
				ret = 0;
				appBeaconSockflagSet(0, ind);
			}
		}
		else if (sysinfo.sockSuccess)
		{
			LogPrintf(DEBUG_BLE, "MasterInfo==>master[%s]other link,rigister fail", sysinfo.masterSn);
			ret = 0;
			appBeaconSockflagSet(0, ind);
		}
	}

	
	if (sysinfo.sockSuccess)
	{
		/* 回归安全围栏,如果还有别的类型的请求,gps wifi等请求,这里可能会影响 */
		resetSafeArea();
	}
	data[destlen++] = ret;
	destlen = appPackProtocol(dest, CMD_GENERAL_RESPON, data, destlen);
	appSendNotifyData(connhandle, dest, destlen);
	byteToHexString(dest, debug, destlen);
	debug[destlen * 2] = 0;
	LogPrintf(DEBUG_BLE, "Send Master Info[%d]:%s", connhandle, debug);
}


