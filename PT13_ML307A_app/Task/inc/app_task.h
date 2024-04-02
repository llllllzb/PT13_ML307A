#ifndef APP_TASK
#define APP_TASK
#include <stdint.h>
#include "app_central.h"
#define SYSTEM_LEN_IDLE					0X01	//待机状态，关闭模组
#define SYSTEM_LED_RUN					0X02	//开启模组
#define SYSTEM_LED_NETOK				0X04
#define SYSTEM_LED_GPSOK				0X08	//普通GPS
#define SYSTEM_LED_BLE					0X10


#define GPSLED1							0
#define PETLED1							1

#define GPS_REQUEST_UPLOAD_ONE			0X00000001
#define GPS_REQUEST_ACC_CTL				0X00000002
#define GPS_REQUEST_GPSKEEPOPEN_CTL		0X00000004
#define GPS_REQUEST_BLE					0X00000008
#define GPS_REQUEST_DEBUG				0X00000010
#define GPS_REQUEST_123_CTL				0X00000020

#define GPS_REQUEST_ALL					0xFFFFFFFF

#define NET_REQUEST_CONNECT_ONE			0X00000001	//主动上报一次位置
#define NET_REQUEST_WIFI_CTL			0X00000002	//开启模组去扫描WIFI，不一定要联网
#define NET_REQUEST_KEEPNET_CTL			0X00000004	//*保持链路常在(该状态与NET_REQUEST_OFFLINE互斥)
#define NET_REQUEST_OFFLINE				0X00000008	//*该标志表示模组开启但不联网(该状态与NET_REQUEST_KEEPNET_CTL互斥)
#define NET_REQUEST_TTS_CTL				0X00000010	//发送TTS语音
#define NET_REQUEST_ALARM_ONE			0X00000040  //开机上报一次报警

#define NET_REQUEST_ALL					0xFFFFFFFF

/* 模组工作状态 */
#define NET_STATUS_CLOSE				0x0001		//该状态下
#define NET_STATUS_OPEN					0x0002		//

/* 安全区域状态 */
#define SAFE_AREA_OUT					0
#define SAFE_AREA_IN					1
#define SAFE_AREA_UNKNOW				2



#define ALARM_LIGHT_REQUEST				0x0001 //感光
#define ALARM_LOSTV_REQUEST				0x0002 //断电
#define ALARM_LOWV_REQUEST				0x0004 //低电
#define ALARM_SHUTTLE_REQUEST			0x0008 //震动报警
#define ALARM_ACCLERATE_REQUEST			0X0010
#define ALARM_DECELERATE_REQUEST		0X0020
#define ALARM_RAPIDRIGHT_REQUEST		0X0040
#define ALARM_RAPIDLEFT_REQUEST			0X0080
#define ALARM_SOS_REQUEST				0X0100
#define ALARM_ENTERSAFEAREA_REQUEST		0//0x0200//暂时先不用
#define ALARM_LEAVESAFEAREA_REQUEST		0x0400//暂时先不用
#define ALARM_BLEALARM_REQUEST			0x0800



#define ACCURACY_INIT_STATUS			0
#define ACCURACY_INITWAIT_STATUS		1
#define ACCURACY_RUNNING_STATUS			2

#define ACCURACY_INIT_OK				0
#define ACCURACY_INIT_SDK_ERROR			1
#define ACCURACT_INIT_NET_ERROR			2
#define ACCURACT_INIT_NONE				3

#define APP_TASK_KERNAL_EVENT		    0x0001
#define APP_TASK_POLLUART_EVENT			0x0002
#define APP_TASK_RUN_EVENT				0x0004
#define APP_TASK_STOP_EVENT				0x0008
#define APP_TASK_ONEMINUTE_EVENT		0x0010
#define APP_TASK_CLOSE_MODULE_EVENT		0x0020
#define APP_TASK_RESET_EVENT			0x0040

#define UART_RECV_BUFF_SIZE 			768
#define DEBUG_BUFF_SIZE					256

#define MODE_CHOOSE						0
#define MODE_START						1
#define MODE_RUNING						2
#define MODE_STOP						3
#define MODE_DONE						4

//GPS_UPLOAD_GAP_MAX 以下，gps常开，以上(包含GPS_UPLOAD_GAP_MAX),周期开启
#define GPS_UPLOAD_GAP_MAX				60

#define ACC_READ		0
#define ACC_STATE_ON	1
#define ACC_STATE_OFF	0


#define DEV_EXTEND_OF_MY	0x01
#define DEV_EXTEND_OF_BLE	0x02
#define DEV_EXTEND_OF_FENCE	0x04


typedef enum 
{
	MODULE_STATUS_CLOSE,
	MODULE_STATUS_OPEN,
	MODULE_STATUS_WAIT,
}moduleFsm_e;

typedef struct
{
    uint32_t sys_tick;		//记录系统运行时间
  	uint8_t sysLedState;
    
    uint8_t	sys_led1_onoff;
	uint8_t sys_led2_onoff;
    uint8_t sys_led1_on_time;
    uint8_t sys_led1_off_time;
    uint8_t sys_led2_on_time;
    uint8_t sys_led2_off_time;	
	
} SystemLEDInfo;

typedef enum
{
    TERMINAL_WARNNING_NORMAL = 0, /*    0 		*/
    TERMINAL_WARNNING_SHUTTLE,    /*    1：震动报警       */
    TERMINAL_WARNNING_LOSTV,      /*    2：断电报警       */
    TERMINAL_WARNNING_LOWV,       /*    3：低电报警       */
    TERMINAL_WARNNING_SOS,        /*    4：SOS求救      */
    TERMINAL_WARNNING_CARDOOR,        /*    5：车门求救      */
    TERMINAL_WARNNING_SWITCH,        /*    6：开关      */
    TERMINAL_WARNNING_LIGHT,      /*    7：感光报警       */
} TERMINAL_WARNNING_TYPE;

typedef enum{
	GPSCLOSESTATUS,
	GPSWATISTATUS,
	GPSOPENSTATUS,

}GPSREQUESTFSMSTATUS;

typedef struct
{
	uint8_t initok;
	uint8_t fsmStep;
	uint8_t tick;
}AccuracyStruct;

typedef enum
{
    ACC_SRC,
    VOLTAGE_SRC,
    GSENSOR_SRC,
    SYS_SRC,
} motion_src_e;
typedef enum
{
    MOTION_STATIC = 0,
    MOTION_MOVING,
} motionState_e;


typedef struct
{
    uint8_t ind;
    motionState_e motionState;
    uint8_t tapInterrupt;
    uint8_t tapCnt[50];
} motionInfo_s;


typedef enum
{
    BLE_IDLE,
    BLE_SCAN,
    BLE_SCAN_WAIT,
    BLE_CONN,
    BLE_CONN_WAIT,
    BLE_READY,
    BLE_DONE,
} modeChoose_e;

typedef struct
{
    uint8_t runFsm ;
    uint8_t runTick;
    uint8_t scanCnt;
    uint8_t connCnt;
    uint8_t mac[6];
    uint8_t addrType;
} bleScanTry_s;

extern motionInfo_s motionInfo;

void terminalDefense(void);
void terminalDisarm(void);
uint8_t getTerminalAccState(void);
void terminalAccon(void);
void terminalAccoff(void);
void terminalCharge(void);
void terminalunCharge(void);
uint8_t getTerminalChargeState(void);
void terminalGPSFixed(void);
void terminalGPSUnFixed(void);

void hdGpsGsvCtl(uint8_t onoff);
void ledSetPeriod(uint8_t ledtype, uint8_t on_time, uint8_t off_time);

void ledStatusUpdate(uint8_t status, uint8_t onoff);
void lbsRequestClear(void);

void saveGpsHistory(void);
void resetSafeArea(void);
void updateModuleStatus(void);

void gpsRequestSet(uint32_t flag);
void gpsRequestClear(uint32_t flag);
uint32_t gpsRequestGet(uint32_t flag);
void gpsTcpSendRequest(void);
uint8_t netRequestOtherGet(uint32_t req);
void movingStatusCheck(void);
void wifiCheckByStep(void);
void motionClear(void);

uint8_t gpsInWait(void);

void wakeUpByInt(uint8_t     type,uint8_t sec);

void volCheckRequestSet(void);
void volCheckRequestClear(void);

void alarmRequestSet(uint16_t request);
void alarmRequestClear(uint16_t request);

void wifiRspSuccess(void);
void wifiTimeout(void);

void lbsRequestSet(uint8_t ext);
void wifiRequestSet(uint8_t ext);
void wifiRequestClear(uint8_t ext);
void netRequestSet(uint32_t req);
void netRequestClear(uint32_t req);
uint8_t netRequestGet(uint32_t req);

void netRequestTask(void);
void motionOccur(void);
void motionInit(void);
void modeTryToStop(void);
void systemShutDown(void);
uint8_t isModeDone(void);
void bleScanCallBack(deviceScanList_s *list);
void bleConnCallBack(void);
void sosRequestSet(void);

void doDebugRecvPoll(uint8_t *msg, uint16_t len);
void gpsUartRead(uint8_t *msg, uint16_t len);
void myTaskPreInit(void);
void myTaskInit(void);
#endif
