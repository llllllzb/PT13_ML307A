/*
 * app_peripheral.c
 *
 *  Created on: Feb 25, 2022
 *      Author: idea
 */
#include "app_peripheral.h"
#include "aes.h"
#include "app_instructioncmd.h"
#include "app_port.h"
#include "app_sys.h"
#include "app_server.h"
#include "app_protocol.h"
#include "app_param.h"
#include "aes.h"
#include "app_bleprotocol.h"
#include "app_net.h"
#include "app_task.h"
/*
 * ȫ�ֱ�������
 */
tmosTaskID appPeripheralTaskId = INVALID_TASK_ID;
gapBondCBs_t appPeripheralGapBondCallBack;
gapRolesCBs_t appPeripheralGapRolesCallBack;
connectionInfoStruct beaconInfoList[PERIPHERAL_MAX_CONNECTION];
/*
 * ��������
 */
static tmosEvents appPeripheralEventProcess(tmosTaskID taskID, tmosEvents events);
static void appPeripheralGapRolesRssiRead(uint16_t connHandle, int8_t newRSSI);
static void appPeripheralGapRolesStateNotify(gapRole_States_t newState, gapRoleEvent_t *pEvent);
static void appPeripheralGapRolesParamUpdate(uint16_t connHandle, uint16_t connInterval, uint16_t connSlaveLatency,
        uint16_t connTimeout);
static void appPeripheralPasscodeCB(uint8_t *deviceAddr, uint16_t connectionHandle,
                             uint8_t uiInputs, uint8_t uiOutputs);
static void appPeripheralPairStateCB(uint16_t connHandle, uint8_t state, uint8_t status);
static void appBeaconInit(void);

static int appBeaconInfoAdd(gapRoleEvent_t *pEvent);


/********************************************************
 * *����UUID
 ********************************************************/

#define APP_SERVICE_UUID            0xFFE0
#define APP_CHARACTERISTIC1_UUID    0xFFE1

/********************************************************
 * *UUID����С��
 ********************************************************/

static uint8 ServiceUUID[ATT_BT_UUID_SIZE] = { LO_UINT16(APP_SERVICE_UUID), HI_UINT16(APP_SERVICE_UUID)};
static uint8 Char1UUID[ATT_BT_UUID_SIZE] = { LO_UINT16(APP_CHARACTERISTIC1_UUID), HI_UINT16(APP_CHARACTERISTIC1_UUID)};

/********************************************************
 * *����UUID������Ϣ
 ********************************************************/
static gattAttrType_t ServiceProfile =
{
    ATT_BT_UUID_SIZE, ServiceUUID
};

/********************************************************
 * *����������ֵ
 ********************************************************/
static uint8 char1_Properties = GATT_PROP_READ | GATT_PROP_WRITE
                                | GATT_PROP_NOTIFY;

/********************************************************
 * *�����洢��
 ********************************************************/
static uint8 char1ValueStore[4];
static gattCharCfg_t char1ClientConfig[4];

/********************************************************
 * *��������
 ********************************************************/

static uint8 char1Description[] = "appchar1";

/********************************************************
 * *����������
 ********************************************************/
static gattAttribute_t appAttributeTable[] =
{
    //Service
    {   { ATT_BT_UUID_SIZE, primaryServiceUUID }, //type
        GATT_PERMIT_READ, 0,
        (uint8 *)& ServiceProfile
    },
    //��������
    {   { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ, 0, &char1_Properties
    },
    //��������ֵ
    {   { ATT_BT_UUID_SIZE, Char1UUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 0, char1ValueStore
    },
    //�ͻ�����������NOTIFY
    {   { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 0, (uint8 *) char1ClientConfig
    },
    //�����������û�����
    {   { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 0, char1Description
    }
};

/********************************************************
 * *��ӷ���ص�
 ********************************************************/
static bStatus_t appReadAttrCB(uint16 connHandle, gattAttribute_t *pAttr,
                               uint8 *pValue, uint16 *pLen, uint16 offset, uint16 maxLen, uint8 method)
{
    bStatus_t ret = SUCCESS;
    uint16 uuid;
    if (gattPermitAuthorRead(pAttr->permissions))
    {
        return ATT_ERR_INSUFFICIENT_AUTHOR;
    }
    if (offset > 0)
    {
        return ATT_ERR_ATTR_NOT_LONG;
    }
    if (pAttr->type.len == ATT_BT_UUID_SIZE)
    {
        uuid = BUILD_UINT16(pAttr->type.uuid[0], pAttr->type.uuid[1]);
        LogPrintf(DEBUG_ALL, "UUID[0x%X]==>Read Request", uuid);
        switch (uuid)
        {
            case APP_CHARACTERISTIC1_UUID:
                *pLen = 4;
                tmos_memcpy(pValue, pAttr->pValue, 4);
                break;
        }
    }
    else
    {
        *pLen = 0;
        ret = ATT_ERR_INVALID_HANDLE;
    }

    return ret;
}



static bStatus_t appWriteAttrCB(uint16 connHandle, gattAttribute_t *pAttr,
                                uint8 *pValue, uint16 len, uint16 offset, uint8 method)
{
    bStatus_t ret = SUCCESS;
    uint16 uuid;
    uint8_t debugStr[101], debugLen;
    insParam_s insParam;
    if (gattPermitAuthorWrite(pAttr->permissions))
    {
        return ATT_ERR_INSUFFICIENT_AUTHOR;
    }
    if (pAttr->type.len == ATT_BT_UUID_SIZE)
    {
        uuid = BUILD_UINT16(pAttr->type.uuid[0], pAttr->type.uuid[1]);
        //LogPrintf(DEBUG_ALL, "UUID[0x%X]==>Write Request,size:%d", uuid, len);
        debugLen = len > 50 ? 50 : len;
        byteToHexString(pValue, debugStr, debugLen);
        debugStr[debugLen * 2] = 0;
        LogPrintf(DEBUG_ALL, "Dev rec(%d)[%d]:%s", getBeaconIdByHandle(connHandle), connHandle, debugStr);
        switch (uuid)
        {
            case APP_CHARACTERISTIC1_UUID:
            	insParam.bleConhandle = connHandle;
            	if (pValue[0] != 0x0C)
            	{
                	instructionParser(pValue, len, BLE_MODE, &insParam);
                }
                else
                {
					bleProtocolParser(connHandle, pValue, len);
                }
                break;
            case GATT_CLIENT_CHAR_CFG_UUID:
                ret = GATTServApp_ProcessCCCWriteReq(connHandle, pAttr, pValue, len,
                                                     offset, GATT_CLIENT_CFG_NOTIFY);
                break;
        }
    }
    else
    {
        ret = ATT_ERR_INVALID_HANDLE;
    }
    return ret;
}
static gattServiceCBs_t gattServiceCallBack = { appReadAttrCB, appWriteAttrCB,
                                                NULL,
                                              };

static void appHandleConnStatusCB(uint16 connHandle, uint8 changeType)
{
    if (connHandle != LOOPBACK_CONNHANDLE)
    {
        if ((changeType == LINKDB_STATUS_UPDATE_REMOVED)
                || ((changeType == LINKDB_STATUS_UPDATE_STATEFLAGS)
                    && (!linkDB_Up(connHandle))))
        {
            GATTServApp_InitCharCfg(connHandle, char1ClientConfig);
        }
    }
}

static void appAddServer(void)
{
    GATTServApp_InitCharCfg(INVALID_CONNHANDLE, char1ClientConfig);
    linkDB_Register(appHandleConnStatusCB);
    GATTServApp_RegisterService(appAttributeTable,
                                GATT_NUM_ATTRS(appAttributeTable), GATT_MAX_ENCRYPT_KEY_SIZE,
                                &gattServiceCallBack);
}

/*
 * ���ù㲥����Ϣ
 */
void appPeripheralBroadcastInfoCfg(uint8 *broadcastnmae)
{
    uint8 len, advLen;
    uint8 advertData[31];
    len = tmos_strlen(broadcastnmae);

    advLen = 0;
    advertData[advLen++] = len + 1;
    advertData[advLen++] = GAP_ADTYPE_LOCAL_NAME_COMPLETE;
    tmos_memcpy(advertData + advLen, broadcastnmae, len);
    advLen += len;
    GAPRole_SetParameter(GAPROLE_ADVERT_DATA, advLen, advertData);
}
/*
 * ��������ʼ��
 */
void appPeripheralInit(void)
{
    char broadCastNmae[30] = { 0 };
    uint8 u8Value;
    uint16 u16Value;
    
    //�����ʼ��
    GAPRole_PeripheralInit();
    //ע������
    appPeripheralTaskId = TMOS_ProcessEventRegister(appPeripheralEventProcess);

    appPeripheralGapBondCallBack.pairStateCB = appPeripheralPairStateCB;
    appPeripheralGapBondCallBack.passcodeCB = appPeripheralPasscodeCB;
    appPeripheralGapRolesCallBack.pfnParamUpdate = appPeripheralGapRolesParamUpdate;
    appPeripheralGapRolesCallBack.pfnRssiRead = appPeripheralGapRolesRssiRead;
    appPeripheralGapRolesCallBack.pfnStateChange = appPeripheralGapRolesStateNotify;

    //��������
    //���ù㲥��Ϣ
    sprintf(broadCastNmae, "PT13-%s", dynamicParam.SN + 9);
    appPeripheralBroadcastInfoCfg(broadCastNmae);
    //�����㲥
    u8Value = TRUE;
    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8), &u8Value);
    //���ð����
    appBondPsdCfg(atoi(sysparam.blePsw));
    //����������Ӽ��
    u16Value = 0x0006;    //6*1.25=7.5ms
    GAPRole_SetParameter(GAPROLE_MIN_CONN_INTERVAL, sizeof(uint16), &u16Value);
    //��������Ӽ��
    u16Value = 0x0c80;    //3200*1.25=4000ms
    GAPRole_SetParameter(GAPROLE_MIN_CONN_INTERVAL, sizeof(uint16), &u16Value);
    //������̹㲥���
    //unit:0.625ms*1600=1000ms
    GAP_SetParamValue(TGAP_DISC_ADV_INT_MIN, 1600);
    //������㲥���
    GAP_SetParamValue(TGAP_DISC_ADV_INT_MAX, 1600);
    //ʹ��ɨ��Ӧ��֪ͨ
    GAP_SetParamValue(TGAP_ADV_SCAN_REQ_NOTIFY, FALSE);
    //��ӷ���
    appAddServer();
    //GATT_InitClient();
    appBeaconInit();
    tmos_set_event(appPeripheralTaskId, APP_PERIPHERAL_START_EVENT);
	/* ����������һ������ */
    tmos_start_reload_task(appPeripheralTaskId, APP_PERIPHERAL_ONEMINUTE_EVENT, MS1_TO_SYSTEM_TIME(1000));
}

/*
 * *notify data
 */
static bStatus_t appNotify(uint16 connHandle, attHandleValueNoti_t *pNoti)
{
    uint16 value = GATTServApp_ReadCharCfg(connHandle, char1ClientConfig);
    if (value & GATT_CLIENT_CFG_NOTIFY)
    {
        pNoti->handle = appAttributeTable[2].handle;
        return GATT_Notification(connHandle, pNoti, FALSE);
    }
    return bleIncorrectMode;
}

bStatus_t appSendNotifyData(uint16 connHandle, uint8 *data, uint16 len)
{
    bStatus_t ret;
    attHandleValueNoti_t notify;
    notify.len = len;
    notify.pValue = GATT_bm_alloc(connHandle, ATT_HANDLE_VALUE_NOTI, notify.len, NULL, 0);
    if (notify.pValue == NULL)
    {
        LogPrintf(DEBUG_ALL, "appSendNotifyData==>alloc memory fail");
        return 0x14;
    }
    tmos_memcpy(notify.pValue, data, notify.len);
    ret = appNotify(connHandle, &notify);
    if (ret != SUCCESS)
    {
        GATT_bm_free((gattMsg_t *)&notify, ATT_HANDLE_VALUE_NOTI);
        LogPrintf(DEBUG_ALL, "Notify fail[0x%x]", ret);
        return ret;
    }
    else
    {
        LogPrintf(DEBUG_ALL, "Notify success[0x%x]", ret);
        return ret;
    }
}

static void appGapMsgProcess(gapRoleEvent_t *pMsg)
{
    int8 debug[20];
    LogPrintf(DEBUG_ALL, "appGapMsgProcess==>OptionCode:0x%X", pMsg->gap.opcode);
    switch (pMsg->gap.opcode)
    {
        case GAP_SCAN_REQUEST_EVENT:
            byteToHexString(pMsg->scanReqEvt.scannerAddr, debug, B_ADDR_LEN);
            debug[B_ADDR_LEN * 2] = 0;
            LogPrintf(DEBUG_ALL, "ScannerMac:%s", debug);
            break;
        case GAP_PHY_UPDATE_EVENT:
            LogPrintf(DEBUG_ALL, "-------------------------------------");
            LogPrintf(DEBUG_ALL, "*****LinkPhyUpdate*****");
            LogPrintf(DEBUG_ALL, "connHandle:%d", pMsg->linkPhyUpdate.connectionHandle);
            LogPrintf(DEBUG_ALL, "connRxPHYS:%d", pMsg->linkPhyUpdate.connRxPHYS);
            LogPrintf(DEBUG_ALL, "connTxPHYS:%d", pMsg->linkPhyUpdate.connTxPHYS);
            LogPrintf(DEBUG_ALL, "-------------------------------------");
            break;
    }
}

static void appGattMsgProcess(gattMsgEvent_t *pMsg)
{
//    LogPrintf(DEBUG_ALL, "pMsg->connHandle==>%#X", pMsg->connHandle);
//    LogPrintf(DEBUG_ALL, "pMsg->method==>%#X", pMsg->method);
    switch (pMsg->method)

    {
        case ATT_EXCHANGE_MTU_RSP:
            LogPrintf(DEBUG_ALL, "pMsg->msg.exchangeMTURsp.serverRxMTU==>%d", pMsg->msg.exchangeMTURsp.serverRxMTU);
            break;
        case ATT_MTU_UPDATED_EVENT:
            LogPrintf(DEBUG_ALL, "pMsg->msg.mtuEvt.MTU==>%d", pMsg->msg.mtuEvt.MTU);
            break;
    }
}
static void appPerihperalSysEventMsg(tmos_event_hdr_t *msg)
{
    LogPrintf(DEBUG_ALL, "SysMsgEvent=0x%X,Status=0x%X", msg->event, msg->status);
    switch (msg->event)
    {
        case GAP_MSG_EVENT:
            appGapMsgProcess((gapRoleEvent_t *) msg);
            break;
        case GATT_MSG_EVENT:
            appGattMsgProcess((gattMsgEvent_t *)msg);
            break;
    }
}

void appLocalNotifyAutoCtl(uint16_t connhandle, uint8 onoff)
{
    uint16 cccd = 0;
    uint8 ret;
    if (onoff)
    {
        cccd |= GATT_CLIENT_CFG_NOTIFY;
    }
    else
    {
        cccd &= ~GATT_CLIENT_CFG_NOTIFY;
    }
    ret = GATTServApp_WriteCharCfg(connhandle, char1ClientConfig, cccd);
    LogPrintf(DEBUG_ALL, "Notify ret:%d", ret);
}



/*
 * �����¼�����
 */
static tmosEvents appPeripheralEventProcess(tmosTaskID taskID, tmosEvents events)
{
    if (events & SYS_EVENT_MSG)
    {
        uint8 *pMsg;
        if ((pMsg = tmos_msg_receive(appPeripheralTaskId)) != NULL)
        {
            appPerihperalSysEventMsg((tmos_event_hdr_t *) pMsg);
            tmos_msg_deallocate(pMsg);
        }
        return (events ^ SYS_EVENT_MSG);
    }
    if (events & APP_PERIPHERAL_START_EVENT)
    {
        GAPRole_PeripheralStartDevice(appPeripheralTaskId,
                                      &appPeripheralGapBondCallBack, &appPeripheralGapRolesCallBack);
        return events ^ APP_PERIPHERAL_START_EVENT;
    }

	if (events & APP_PERIPHERAL_TIMEOUT_TERMINATE_EVENT)
	{
		if (getBeaconIdByTaskId(taskID) >= 0)
		{
			LogPrintf(DEBUG_ALL, "Auth Time out==>taskid:%d conhandle:%d", taskID, beaconInfoList[getBeaconIdByTaskId(taskID)].connectionHandle);
			GAPRole_TerminateLink(beaconInfoList[getBeaconIdByTaskId(taskID)].connectionHandle);
		}
		else
		{
			tmos_stop_task(taskID, APP_PERIPHERAL_TIMEOUT_TERMINATE_EVENT);
		}
		return events ^ APP_PERIPHERAL_TIMEOUT_TERMINATE_EVENT;
	}

	if (events & APP_PERIPHERAL_AUTH_EVENT)
	{
		
		return events ^ APP_PERIPHERAL_AUTH_EVENT;
	}

	if (events & APP_PERIPHERAL_TEST_EVENT)
	{
		uint8_t param[2]={0x10, 0x88};
		uint8_t test_handle = getBeaconHandleByTaskId(taskID);
		appSendNotifyData(test_handle, param, 2);
		return events ^ APP_PERIPHERAL_TEST_EVENT;
	}
	if (events & APP_PERIPHERAL_NOTIFYEN_EVENT)
	{
		
		return events ^ APP_PERIPHERAL_NOTIFYEN_EVENT;
	}

	if (events & APP_PERIPHERAL_ONEMINUTE_EVENT)
	{
		/* �����������ֹͣ������systick������ */
		return events ^ APP_PERIPHERAL_ONEMINUTE_EVENT;
	}
	LogPrintf(DEBUG_ALL, "unknow event taskid:%d events:%d", taskID, events);

    return 0;
}



/**************************************************
@bref       ���ӳɹ�ʱ����
@param
    connHandle  ����ľ��
@return
@note
**************************************************/
static void appBeaconConnectCB(gapRoleEvent_t *pEvent)
{
	uint8_t index;
	for (index = 0; index < PERIPHERAL_MAX_CONNECTION; index++)
	{
		if (beaconInfoList[index].useflag == 0 && beaconInfoList[index].connectionHandle == GAP_CONNHANDLE_INIT)
			break;
	}
	if (index == PERIPHERAL_MAX_CONNECTION)
	{
		GAPRole_TerminateLink(pEvent->linkCmpl.connectionHandle);
		LogPrintf(DEBUG_ALL, "Beacon count maximum limit has been reached");
	}
	else
	{
		if (appBeaconInfoAdd(pEvent) < 0)
    	{
			LogPrintf(DEBUG_ALL, "Beacon list is full!!!");
    	}
    	//��ѯ�Ƿ��п���λ��
    	for (index = 0; index < PERIPHERAL_MAX_CONNECTION; index++)
        {
            if(beaconInfoList[index].connectionHandle == GAP_CONNHANDLE_INIT)
            {
                break;
            }
        }
        if (index < PERIPHERAL_MAX_CONNECTION)
        {
            // Restart advertising
            {
                uint8_t advertising_enable = TRUE;
                GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &advertising_enable);
            }
        }
	}
}

/**************************************************
@bref       ���ӱ��Ͽ�ʱ����
@param
    connHandle  ����ľ��
@return
@note
**************************************************/
static void appBeaconTerminateCB(gapRoleEvent_t *pEvent)
{
	uint8_t index;
	for (index = 0; index < PERIPHERAL_MAX_CONNECTION; index++)
	{
		if (beaconInfoList[index].useflag && pEvent->linkTerminate.connectionHandle == beaconInfoList[index].connectionHandle)
		{
			break;
		}
	}
	if (index == PERIPHERAL_MAX_CONNECTION)
	{
		LogPrintf(DEBUG_ALL, "appBeaconTerminate==>Search beacon info error[%d]", index);
		uint8_t advertising_enable = TRUE;
        GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &advertising_enable);
	}
	else
	{
		if (appBeaconInfoDel(pEvent) < 0)
		{
			LogPrintf(DEBUG_ALL, "appBeaconTerminateCB==>Delete beacon info fail");
		}
	    // Restart advertising
        uint8_t advertising_enable = TRUE;
        GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &advertising_enable);
	}
}

/**************************************************
@bref       �豸�㲥
@param
@return
@note
**************************************************/

static void appGaproleAdvertising(gapRoleEvent_t *pEvent)
{
	if(pEvent->gap.opcode == GAP_LINK_TERMINATED_EVENT)
	{
		appBeaconTerminateCB(pEvent);
	}
	else if(pEvent->gap.opcode == GAP_MAKE_DISCOVERABLE_DONE_EVENT)
	{
		LogPrintf(DEBUG_ALL, "Advertising..");
	}

}


/**************************************************
@bref       �豸����
@param
@return
@note
**************************************************/

static void appGaproleConnect(gapRoleEvent_t *pEvent)
{
    if (pEvent->gap.opcode == GAP_LINK_ESTABLISHED_EVENT)
    {
		appBeaconConnectCB(pEvent);
    }
}
/**************************************************
@bref       �豸����
@param
@return
@note
**************************************************/

static void appGaproleWaitting(gapRoleEvent_t *pEvent)
{
    uint8 u8Value;
    if (pEvent->gap.opcode == GAP_LINK_TERMINATED_EVENT)
    {
    	appBeaconTerminateCB(pEvent);
    }
    else if (pEvent->gap.opcode == GAP_LINK_ESTABLISHED_EVENT)
    {
		if(pEvent->gap.hdr.status != SUCCESS)
        {
            LogPrintf(DEBUG_ALL, "Waiting for advertising..");
        }
        else
        {
            LogPrintf(DEBUG_ALL, "Error..");
        }
    }
    else
    {
        LogPrintf(DEBUG_ALL, "Error..%x", pEvent->gap.opcode);
        appBeaconTerminateCB(pEvent);
    }
}

/*
*	GapBond״̬�ص�����
*/
static void appPeripheralPasscodeCB(uint8_t *deviceAddr, uint16_t connectionHandle,
                             uint8_t uiInputs, uint8_t uiOutputs)
{
    LogPrintf(DEBUG_ALL, "uiInput:%d, uiOutput:%d\n", uiInputs, uiOutputs);
    uint32_t passkey;
    GAPBondMgr_GetParameter(GAPBOND_PERI_DEFAULT_PASSCODE, &passkey);
    LogPrintf(DEBUG_ALL, "passkey:%d", passkey);
//    GAPBondMgr_PasscodeRsp(connectionHandle, SUCCESS, passkey);
}

static void appPeripheralPairStateCB(uint16_t connHandle, uint8_t state, uint8_t status)
{
    LogPrintf(DEBUG_ALL, "appHidPairStateCB==>0x%02x", state);
	switch(state)
	{
		case GAPBOND_PAIRING_STATE_STARTED:
			LogMessage(DEBUG_ALL, "Pairing Start");
			break;
		//��ʾ��ԵĹ�������ˣ�����һ���ǳɹ���
		case GAPBOND_PAIRING_STATE_COMPLETE:
		    if(status == SUCCESS)
		    {
		        LogMessage(DEBUG_ALL, "Pairing Success");
	           	
	            //sysinfo.bleConnStatus = 1;
		    }
		    else
		    {
                LogMessage(DEBUG_ALL, "Pairing Fail");
            }
			break;
		case GAPBOND_PAIRING_STATE_BOND_SAVED:
			LogMessage(DEBUG_ALL, "Pairing Bond Save");
            //��ȡ�Ѱ󶨵��豸
            uint8_t u8Value;
            GAPBondMgr_GetParameter( GAPBOND_BOND_COUNT, &u8Value );
            LogPrintf(DEBUG_ALL, "Bonded device count :%d", u8Value);
			break;
		//�Ѿ�����Թ����豸��ֱ�ӽ������״̬�����ϵ�״̬����
		case GAPBOND_PAIRING_STATE_BONDED:
			LogMessage(DEBUG_ALL, "Paring Bonded");	
			break;
		default :
			LogMessage(DEBUG_ALL, "Unknow Pairing State");
			break;
			
	}
}

/*
 * ��������
 */
static void appPeripheralGapRolesParamUpdate(uint16_t connHandle,
        uint16_t connInterval, uint16_t connSlaveLatency, uint16_t connTimeout)
{
    LogPrintf(DEBUG_ALL, "-------------------------------------");
    LogPrintf(DEBUG_ALL, "*****ParamUpdate*****");
    LogPrintf(DEBUG_ALL, "connectionHandle:%d", connHandle);
    LogPrintf(DEBUG_ALL, "connInterval:%d", connInterval);
    LogPrintf(DEBUG_ALL, "connLatency:%d", connSlaveLatency);
    LogPrintf(DEBUG_ALL, "connTimeout:%d", connTimeout);
    LogPrintf(DEBUG_ALL, "-------------------------------------");
}
/*
 * �ź�ֵ
 */
static void appPeripheralGapRolesRssiRead(uint16_t connHandle, int8_t newRSSI)
{
    LogPrintf(DEBUG_ALL, "ConnHandle %d, RSSI :%d", connHandle, newRSSI);
}
/*
 * ״̬�¼�
 */
static void appPeripheralGapRolesStateNotify(gapRole_States_t newState,        gapRoleEvent_t *pEvent)
{
    switch (newState & GAPROLE_STATE_ADV_MASK)
    {
        case GAPROLE_STARTED:
            LogPrintf(DEBUG_ALL, "GapRolesStateNotify==>GAPRole started");
            break;
        case GAPROLE_ADVERTISING:
            LogPrintf(DEBUG_ALL, "GapRolesStateNotify==>GAPRole advertising");
            appGaproleAdvertising(pEvent);
            break;
        case GAPROLE_WAITING:
            LogPrintf(DEBUG_ALL, "GapRolesStateNotify==>GAPRole waitting");
            appGaproleWaitting(pEvent);
            break;
        case GAPROLE_CONNECTED:
            LogPrintf(DEBUG_ALL, "GapRolesStateNotify==>GAPRole connected");
            appGaproleConnect(pEvent);
            break;
        case GAPROLE_CONNECTED_ADV:
            LogPrintf(DEBUG_ALL, "GapRolesStateNotify==>GAPRole connected adv");
            break;
        case GAPROLE_ERROR:
            LogPrintf(DEBUG_ALL, "GapRolesStateNotify==>GAPRole error");
            break;
        default:
            break;
    }
}

/**************************************************
@bref       ���ð�����
@param
@return
@note
**************************************************/

void appBondPsdCfg(uint32_t psd)
{
//	uint32_t passkey  = psd;
//	uint8_t  pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;//GAPBOND_PAIRING_MODE_INITIATE��ʾ�������������������Ҳ�������á� GAPBOND_PAIRING_MODE_WAIT_FOR_REQ��ʾ�ȴ��Է��������
//	uint8_t  mitm 	  = TRUE;
//	uint8_t  ioCap    = GAPBOND_IO_CAP_KEYBOARD_DISPLAY;
//	uint8_t  bonding  = TRUE;
//	GAPBondMgr_SetParameter(GAPBOND_PERI_DEFAULT_PASSCODE, sizeof(uint32_t), &passkey);
//	GAPBondMgr_SetParameter(GAPBOND_PERI_PAIRING_MODE, 	   sizeof(uint8_t),  &pairMode);
//	GAPBondMgr_SetParameter(GAPBOND_PERI_MITM_PROTECTION,  sizeof(uint8_t),  &mitm);
//	GAPBondMgr_SetParameter(GAPBOND_PERI_IO_CAPABILITIES,  sizeof(uint8_t),  &ioCap);
//	GAPBondMgr_SetParameter(GAPBOND_PERI_BONDING_ENABLED,  sizeof(uint8_t),  &bonding);
}

/**************************************************
@bref      	����SN��������Կ
@param
@return
@note
**************************************************/

void appCreatePasswordBySn(char *sn)
{
	strncpy(sysparam.blePsw, sn, 7);
	LogPrintf(DEBUG_BLE, "CreatePassword:%s", sysparam.blePsw);
	paramSaveAll();
}

/**************************************************
@bref       ��ѯ���豸
@param
@return
@note
**************************************************/

void appShowBondDecv(void)
{
	uint8_t bondcount;
    uint8_t addr[6] = {0};
    uint8_t buf[7] = {0};
    uint8_t debug[50];
    uint8_t i;
    GAPBondMgr_GetParameter(GAPBOND_BOND_COUNT, &bondcount);
    if (bondcount <= 0)
    {
        LogMessage(DEBUG_BLE, "No bond");
        return;
    }
    LogPrintf(DEBUG_BLE, "Ble cnt:%d", bondcount);
    for (i = 0; i < bondcount; i++)
    {
	    tmos_snv_read(mainRecordNvID(i), 6, addr);
	    byteToHexString(addr, debug, 6);
	    debug[12] = 0;
	    LogPrintf(DEBUG_BLE, "Ble Mac[%d]:%s", i, debug);
    }
}

/**************************************************
@bref       ��ѯ����id
@param
@return
@note
**************************************************/
void appBeaconShowTaskId(void)
{
	uint8_t i;
	for (i = 0; i < PERIPHERAL_MAX_CONNECTION; i++)
	{
		LogPrintf(DEBUG_BLE, "appBeaconInit==>Dev(%d)taskid:%d", i, beaconInfoList[i].taskID);
	}
}

/**************************************************
@bref       �����ű��б��ʼ��
@param
@return
@note
**************************************************/

static void appBeaconInit(void)
{
	uint8_t i;
	tmos_memset(beaconInfoList, 0, sizeof(beaconInfoList));
	for (i = 0; i < PERIPHERAL_MAX_CONNECTION; i++)
	{
		beaconInfoList[i].connectionHandle = GAP_CONNHANDLE_INIT;
		beaconInfoList[i].taskID		   = TMOS_ProcessEventRegister(appPeripheralEventProcess);
		LogPrintf(DEBUG_BLE, "appBeaconInit==>Dev(%d)taskid:%d", i, beaconInfoList[i].taskID);
	}
}

/**************************************************
@bref       ����socketλ��1
@param
@return
@note	
	����socketflag�ǻ���ģ�ֻ����1����1
	����1�ɹ�����1�����඼��0
**************************************************/

//uint8_t appBeaconSockflagSet(uint8_t set, char *sn)
//{
//	uint8_t i, cnt = 0;
//	uint8_t ind;
//	if (set)
//	{
//		for (i = 0; i < PERIPHERAL_MAX_CONNECTION; i++)
//		{
//			if (beaconInfoList[i].sockFlag == 0) 
//			{
//				cnt++;
//			}
//			else
//			{
//				ind = i;
//			}
//		}
//		/* �б�û������sock */
//		if (cnt >= PERIPHERAL_MAX_CONNECTION)
//		{
//			LogPrintf(DEBUG_BLE, "111");
//			beaconInfoList[ind].sockFlag = 1;
//			//
//			for (i = 0; i < 15; i++)
//			{
//				beaconInfoList[ind].mSn[i] = sn[i];
//			}
//			beaconInfoList[ind].mSn[15] = 0;
//			return 1;
//		}
//		/* ˵����һ���Ѿ����ϣ��ж������Ƿ�ƥ�� */
//		if (cnt != 0 && cnt < PERIPHERAL_MAX_CONNECTION &&
//			strncmp(sn, beaconInfoList[ind].mSn, 15) == 0 &&
//			beaconInfoList[ind].socksuccess)
//		{
//			
//			LogPrintf(DEBUG_BLE, "222");
//			return 1;
//		}
//		LogPrintf(DEBUG_BLE, "333");
//		
//		return 0;
//	}
//	else
//	{
//		LogPrintf(DEBUG_BLE, "444");
//		beaconInfoList[ind].socksuccess = 0;
//		tmos_memset(beaconInfoList[ind].mSn, 0, sizeof(beaconInfoList[ind].mSn));
//		return 0;
//	}
//}

uint8_t appBeaconSockflagSet(uint8_t set, uint8_t index)
{
	beaconInfoList[index].sockFlag = set;
	LogPrintf(DEBUG_BLE, "appBeaconSockflagSet==>%s", set ? "set" : "clear");
}


/**************************************************
@bref       ��ѯ����socksuccess״̬
@param
@return
@note	
**************************************************/

//int8_t appBeaconGetSockSuccess(void)
//{
//	for (uint8_t i  = 0; i < PERIPHERAL_MAX_CONNECTION; i++)
//	{
//		if (beaconInfoList[i].socksuccess)
//		{
//			return i;
//		}
//	}
//	return -1;
//}

/**************************************************
@bref       �����ű���Ϣ¼��
@param
@return
@note	1: ��Ϣ¼��ɹ�
		0����Ϣ¼��ʧ��
**************************************************/

static int appBeaconInfoAdd(gapRoleEvent_t *pEvent)
{
	int i;
	char debug[20] = { 0 };
	for (i = 0; i < PERIPHERAL_MAX_CONNECTION; i++)
	{
		if (beaconInfoList[i].useflag == 0)
		{
			tmos_memcpy(beaconInfoList[i].addr,  pEvent->linkCmpl.devAddr, B_ADDR_LEN);
			beaconInfoList[i].addrType		   = pEvent->linkCmpl.devAddrType;
			beaconInfoList[i].connectionHandle = pEvent->linkCmpl.connectionHandle;
			beaconInfoList[i].connRole 		   = pEvent->linkCmpl.connRole;
			beaconInfoList[i].connInterval     = pEvent->linkCmpl.connInterval;
			beaconInfoList[i].connLatency      = pEvent->linkCmpl.connLatency;
			beaconInfoList[i].connTimeout      = pEvent->linkCmpl.connTimeout;
			//beaconInfoList[i].updateTick	   = sysinfo.sysTick;	//ע�͵���Ϊ������������ɻ�ȡ�豸Э��������
			beaconInfoList[i].useflag 		   = 1;
			byteToHexString(pEvent->linkCmpl.devAddr, debug, B_ADDR_LEN);
        	debug[B_ADDR_LEN * 2] = 0;
        	LogPrintf(DEBUG_BLE, "-------------------------------------");
        	LogPrintf(DEBUG_BLE, "*****Device Connect*****");
			LogPrintf(DEBUG_BLE, "DeviceMac:%s", 		debug);
		    LogPrintf(DEBUG_BLE, "DeviceType:%d", 		beaconInfoList[i].addrType);
		    LogPrintf(DEBUG_BLE, "connectionHandle:%d", beaconInfoList[i].connectionHandle);
		    LogPrintf(DEBUG_BLE, "connInterval:%d", 	beaconInfoList[i].connInterval);
		    LogPrintf(DEBUG_BLE, "connLatency:%d", 		beaconInfoList[i].connLatency);
		    LogPrintf(DEBUG_BLE, "connRole:%d", 		beaconInfoList[i].connRole);
		    LogPrintf(DEBUG_BLE, "connTimeout:%d", 		beaconInfoList[i].connTimeout);
		    LogPrintf(DEBUG_BLE, "taskId:%d", 			beaconInfoList[i].taskID);
		    LogPrintf(DEBUG_BLE, "-------------------------------------");
		    appLocalNotifyAutoCtl(beaconInfoList[i].connectionHandle, 1);
		    //DEBUG �����Ȩ����֮��Ҫɾ����������
		    beaconInfoList[i].connSuccess = 1;
			LogPrintf(DEBUG_BLE, "Connhandle[%d]==>AuthSuccess", beaconInfoList[i].connectionHandle);
		    //tmos_start_task(beaconInfoList[i].taskID, APP_PERIPHERAL_TIMEOUT_TERMINATE_EVENT, MS1_TO_SYSTEM_TIME(10000));
			return 1;
		}
	}
	return -1;
}

/**************************************************
@bref       �����ű���Ϣɾ��
@param
@return
@note	 1: ɾ���ɹ�
		-1��ɾ��ʧ��
**************************************************/

static int appBeaconInfoDel(gapRoleEvent_t *pEvent)
{
	int i = -1;
	char debug[20];
	i = getBeaconIdByHandle(pEvent->linkTerminate.connectionHandle);
	//LogPrintf(DEBUG_ALL, "i:%d useflag:%d conhandle:%d %d", i, beaconInfoList[i].useflag, beaconInfoList[i].connectionHandle, pEvent->linkTerminate.connectionHandle);
	if (i >= 0)
	{
		if (beaconInfoList[i].useflag && beaconInfoList[i].connectionHandle == pEvent->linkTerminate.connectionHandle)
		{
			byteToHexString(beaconInfoList[i].addr, debug, B_ADDR_LEN);
        	debug[B_ADDR_LEN * 2] = 0;
        	LogPrintf(DEBUG_BLE, "-------------------------------------");
        	LogPrintf(DEBUG_BLE, "*****Device Terminate*****");
			LogPrintf(DEBUG_BLE, "DeviceMac:%s", 		debug);
		    LogPrintf(DEBUG_BLE, "DeviceType:%d", 		beaconInfoList[i].addrType);
			LogPrintf(DEBUG_BLE, "TerminateHandle:%d", beaconInfoList[i].connectionHandle);
			LogPrintf(DEBUG_BLE, "TerminateRole:%d", pEvent->linkTerminate.connRole);
			LogPrintf(DEBUG_BLE, "TerminateReason:0x%X", pEvent->linkTerminate.reason);
			LogPrintf(DEBUG_BLE, "-------------------------------------");
			/*����memsetֱ��ȫ���������ΪtaskidΨһ���޷�ֹͣ*/
			beaconInfoList[i].useflag		   = 0;
			beaconInfoList[i].connectionHandle = GAP_CONNHANDLE_INIT;
			beaconInfoList[i].connSuccess      = 0;

			/*tmos����Ҳ�����*/
			tmos_stop_task(beaconInfoList[i].taskID, APP_PERIPHERAL_TIMEOUT_TERMINATE_EVENT);
		}
		else
		{
			i = -1;
		}
	}
	return i;
}

/**************************************************
@bref       ɾ��ȫ�������ű���Ϣ
@param
@return
@note	
**************************************************/

void appBeaconInfoDelAll(void)
{
	int i;
	for (i = 0; i < PERIPHERAL_MAX_CONNECTION; i++)
	{
		if (beaconInfoList[i].connectionHandle != GAP_CONNHANDLE_INIT && beaconInfoList[i].useflag)
		{
			GAPRole_TerminateLink(beaconInfoList[i].connectionHandle);
		}
		//tmos_memset(&beaconInfoList[i], 0, sizeof(connectionInfoStruct));
		beaconInfoList[i].useflag		   = 0;
		beaconInfoList[i].connectionHandle = GAP_CONNHANDLE_INIT;
	}
}

/**************************************************
@bref       ����������Ȩ�ɹ�
@param
@return
@note	
**************************************************/

void appBeaconAuthSuccess(uint16_t connhandle)
{
	if (getBeaconIdByHandle(connhandle) >= 0)
	{
		beaconInfoList[getBeaconIdByHandle(connhandle)].connSuccess = 1;
		LogPrintf(DEBUG_BLE, "Connhandle[%d]==>AuthSuccess", connhandle);
	}
	if (getBeaconTaskidByHandle(connhandle) >= 0)
	{
		tmos_stop_task(getBeaconTaskidByHandle(connhandle), APP_PERIPHERAL_TIMEOUT_TERMINATE_EVENT);
	}
}


/**************************************************
@bref       ���ݾ���ҵ��б��еı��
@param
@return
@note	
**************************************************/
int getBeaconIdByHandle(uint16_t connhandle)
{
	int i;
	for (i = 0; i < PERIPHERAL_MAX_CONNECTION; i++)
	{
		if (beaconInfoList[i].connectionHandle == connhandle && beaconInfoList[i].useflag)
		{
			return i;
		}
	}
	return -1;
}

/**************************************************
@bref       ��������id�ҵ��豸���
@param
@return
@note	
**************************************************/
int getBeaconIdByTaskId(uint8_t task_id)
{
	int i;
	for (i = 0; i < PERIPHERAL_MAX_CONNECTION; i++)
	{
		if (task_id == beaconInfoList[i].taskID && beaconInfoList[i].useflag)
			return i;
	}
	return -1;
}


/**************************************************
@bref       ��������id�ҵ��豸���
@param
@return
@note	
**************************************************/
int getBeaconHandleByTaskId(uint8_t task_id)
{
	int i;
	for (i = 0; i < PERIPHERAL_MAX_CONNECTION; i++)
	{
		if (task_id == beaconInfoList[i].taskID && beaconInfoList[i].useflag)
			return beaconInfoList[i].connectionHandle;
	}
	return -1;
}

/**************************************************
@bref       �����豸����ҵ�����id
@param
@return
@note	
**************************************************/
int getBeaconTaskidByHandle(uint16_t handle)
{
	int i;
	for (i = 0; i < PERIPHERAL_MAX_CONNECTION; i++)
	{
		if (handle == beaconInfoList[i].connectionHandle && beaconInfoList[i].useflag)
		{
			return beaconInfoList[i].taskID;
		}
	}
	return -1;
}

/**************************************************
@bref       �����豸ָ����������Ϣ
@param
@return
@note	
	��ȡ����Ϣ�����Ƿ����豸���ӽ���
**************************************************/
connectionInfoStruct *getBeaconInfoByIndex(uint8_t index)
{
	return &beaconInfoList[index];
}

/**************************************************
@bref       �����豸ָ����������Ϣ
@param
@return
@note	
**************************************************/
connectionInfoStruct *getBeaconInfoByHandle(uint16_t handle)
{
	uint8_t i;
	for (i = 0; i < PERIPHERAL_MAX_CONNECTION; i++)
	{
		if (beaconInfoList[i].useflag && beaconInfoList[i].connectionHandle == handle) {
			return &beaconInfoList[i];
		}
	}
	return NULL;
}

connectionInfoStruct *getBeaconInfoAll(void)
{
	return beaconInfoList;
}

/**************************************************
@bref       �Ƿ�����������
@param
@return
@note	1: ��
		0����
**************************************************/

uint8_t isInsideBleFence(void)
{
	for (uint8_t i = 0; i < PERIPHERAL_MAX_CONNECTION; i++)
	{
		if (beaconInfoList[i].connSuccess)
		{
			return 1;
		}
	}
	return 0;
}

/**************************************************
@bref       �������ض�����ʱ
@param
@return
@note	1: ��
		0����
�ж��Ƿ������������ص�������
	1.sock��־Ϊ1
	2.3��������Э��ͨѶ
**************************************************/

void bleGatewayDisconnDetect(void)
{
	uint8_t i;
	for (i = 0; i < PERIPHERAL_MAX_CONNECTION; i++)
	{
		if ((sysinfo.sysTick - beaconInfoList[i].updateTick) >= CONNTECT_TIMEOUT_MAXMIN &&
			beaconInfoList[i].sockFlag)
		{
			LogPrintf(DEBUG_BLE, "bleGateway disconnect timeout:%d", sysinfo.sysTick - beaconInfoList[i].updateTick);
			appBeaconSockflagSet(0, i);
			sysinfo.sockSuccess = 0;
			tmos_memset(sysinfo.masterSn, 0, sizeof(sysinfo.masterSn));
		}		
	}

}

/**
 * ��һ�ο������ϱ���ʱ���ڼ�⵽�����ͽ���������û��⵽�����ͼ�������
 * 
*/

/**************************************************
@bref       ����Χ����ѯ����
@param
@return
@note
�����������ڣ��͹���״̬
����ͨ�����ڣ�����������
��wifi�ڣ�ÿ��һ��ʱ����wifi
�뿪wifi����������
**************************************************/

void BleFenceCheck(void)
{
	static uint16_t Contick = 0;
	static uint16_t disConTick = 0;
	bleGatewayDisconnDetect();
	/* ������������ */
	if (sysinfo.sockSuccess)
	{
		Contick    = 0;
		disConTick = 0;
		sysinfo.outBleFenceFlag = 0;
		//LogPrintf(DEBUG_ALL, "In ble gateway");
	}
	/* ���ֻ������� */
	else if (isInsideBleFence())
	{
		disConTick = 0;
		if (Contick++ >= 10)
		{
			if (sysinfo.outBleFenceFlag)
			{
				sysinfo.outBleFenceFlag = 0;
				LogPrintf(DEBUG_BLE, "Dev enter ble fence");
			}
		}
		//LogPrintf(DEBUG_ALL, "In ble fence, Contick:%d", Contick);
	}
//	if (sysinfo.outBleFenceFlag)
//	{
//		disConTick = 0;
//		return;
//	}
	/* outBleFenceFlag��־��1��0�����¿�ʼ��ʱ */
	/* ������ */
	else 
	{
		Contick = 0;
		//LogPrintf(DEBUG_ALL, "Out ble fence, disConTick:%d", disConTick);
		if (disConTick++ >= 10)
		{
			if (sysinfo.outBleFenceFlag == 0)
			{
				sysinfo.outBleFenceFlag = 1;
				LogPrintf(DEBUG_BLE, "Dev leave ble fence");
				LogPrintf(DEBUG_BLE, "wifireq line:%d", __LINE__);
				if (sysparam.wifiCnt != 0)
					wifiRequestSet(DEV_EXTEND_OF_FENCE);
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
@bref       �����ر�
@param
@return
@note
**************************************************/
void bleBeaconClt(uint8_t onoff)
{
	uint8_t u8Value;
	if (onoff)
	{
		u8Value = TRUE;
    	GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8), &u8Value);
	}
	else
	{
		u8Value = FALSE;
		GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8), &u8Value);
	}
}


