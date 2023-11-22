/*
 * app_peripheral.h
 *
 *  Created on: Feb 25, 2022
 *      Author: idea
 */

#ifndef APP_INCLUDE_APP_PERIPHERAL_H_
#define APP_INCLUDE_APP_PERIPHERAL_H_

#include "config.h"

#define CONNTECT_TIMEOUT_MAXMIN					180

#define APP_PERIPHERAL_START_EVENT              0x0001
#define APP_PERIPHERAL_PARAM_UPDATE_EVENT       0x0002
#define APP_PERIPHERAL_TIMEOUT_TERMINATE_EVENT	0x0004
#define APP_PERIPHERAL_AUTH_EVENT				0x0008
#define APP_PERIPHERAL_TEST_EVENT				0x0010
#define APP_PERIPHERAL_ONEMINUTE_EVENT			0x0020
#define APP_PERIPHERAL_NOTIFYEN_EVENT			0x0040

typedef struct{
	uint8_t  useflag;
	uint8_t  taskID;
	uint8_t  connSuccess;		 //!
	uint8_t  addr[6];
	uint8_t  addrType;
    uint16_t connectionHandle;   //!< Connection Handle from controller used to ref the device
    uint8_t  connRole;            //!< Connection formed as Master or Slave
    uint16_t connInterval;       //!< Connection Interval
    uint16_t connLatency;        //!< Connection Latency
    uint16_t connTimeout;        //!< Connection Timeout
    uint16_t updateTick;
	uint8_t  sockFlag;			 //!< 这个标志意味着已经被PT02连接，且两个连接标志位只能有一个置1，若是被手机或其他主机连接这个表示也不会置1
}connectionInfoStruct;


void appPeripheralInit(void);
bStatus_t appSendNotifyData(uint16 connHandle, uint8 *data, uint16 len);
void isInsideBleGateway(void);
uint8_t isInsideBleFence(void);

int getBeaconIdByHandle(uint16_t connhandle);
int getBeaconIdByTaskId(uint8_t task_id);
int getBeaconHandleByTaskId(uint8_t task_id);
void appBeaconAuthSuccess(uint16_t connhandle);
int getBeaconTaskidByHandle(uint16_t handle);
static int appBeaconInfoDel(gapRoleEvent_t *pEvent);
void appBeaconInfoDelAll(void);
void appShowBondDecv(void);
void appBeaconShowTaskId(void);
void appCreatePasswordBySn(char *sn);

connectionInfoStruct *getBeaconInfoByIndex(uint8_t index);
connectionInfoStruct *getBeaconInfoByHandle(uint16_t handle);
uint8_t appBeaconSockflagSet(uint8_t set, uint8_t index);
int8_t appBeaconGetSockSuccess(void);

connectionInfoStruct *getBeaconInfoAll(void);

void appBondPsdCfg(uint32_t psd);
void BleFenceCheck(void);
void bleBeaconClt(uint8_t onoff);


#endif /* APP_INCLUDE_APP_PERIPHERAL_H_ */
