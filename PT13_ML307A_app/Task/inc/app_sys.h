#ifndef APP_SYS
#define APP_SYS

#include "config.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define DEBUG_NONE				0
#define DEBUG_LOW				1
#define DEBUG_NET				2
#define DEBUG_GPS				3
#define DEBUG_FACTORY			4
#define DEBUG_BLE				5
#define DEBUG_ALL				9

#define MODE1									1
#define MODE2									2
#define MODE3									3
#define MODE4									4
#define MODE21									5  //ģʽ2����չ������ģʽ1
#define MODE23									6  //ģʽ2����չ������ģʽ3

#define ITEMCNTMAX	8
#define ITEMSIZEMAX	60

#define ACCDETMODE0			0		//��ʹ��acc��
#define ACCDETMODE1			1		//acc�����ȣ���ε�ѹ
#define ACCDETMODE2			2		//acc�����ȣ�����gsensor
#define ACCDETMODE3         3       //acc�����ȣ���ε�ѹ+gsensor



#define NORMAL_LINK				0
#define BLE_LINK				1
#define JT808_LINK				2
#define HIDDEN_LINK				3
#define AGPS_LINK				4

typedef struct
{
    unsigned char item_cnt;
    char item_data[ITEMCNTMAX][ITEMSIZEMAX];
} ITEM;

typedef struct
{
    uint8_t nmeaOutPutCtl   		: 1;
    uint8_t lbsRequest				: 1;
    uint8_t wifiRequest				: 1;
    uint8_t rtcUpdate				: 1;
    uint8_t flag123					: 1;
    uint8_t gsensorOnoff			: 1;
    uint8_t gsensorErr				: 1;
    uint8_t hiddenServCloseReq		: 1;
    uint8_t agpsRequest				: 1;
    uint8_t doSosFlag				: 1;
    uint8_t doMotionFlag			: 1;
    uint8_t kernalRun				: 1;
    uint8_t sleep					: 1;
    uint8_t first					: 1;
    uint8_t volCheckReq				: 1;
    uint8_t adcOnoff				: 1;
	uint8_t moduleRstFlag			: 1;
	uint8_t mode4NoNetFlag			: 1;
	uint8_t mode4First				: 1;
	uint8_t jt808Lbs				: 1;
    uint8_t jt808Wifi				: 1;
    uint8_t ephemerisFlag			: 1;
    uint8_t outBleFenceFlag			: 1;
    uint8_t outWifiFenceFlag		: 1;
    uint8_t ttsPlayNow				: 1;
    uint8_t inWifiFlag				: 1;
    uint8_t petledOnoff				: 1;
    uint8_t petspkOnoff				: 1;
    uint8_t uploadflag				: 1;
    uint8_t hbtFlag					: 1;
    uint8_t closeTTs				: 1;
    uint8_t lbsExtendEvt;
    uint8_t wifiExtendEvt;
    uint8_t ringWakeUpTick;
    uint8_t cmdTick;
    uint8_t runFsm;
    uint8_t gpsFsm;
    uint8_t gpsOnoff;
    uint8_t gsensorTapCnt;
    uint8_t terminalStatus;
    uint8_t sysLedState;
    uint8_t sysTaskId;
    uint8_t tapRecord[15];
    uint8_t logLevel;
    uint8_t noNmeaCnt;

    uint8_t alarmDate;
    uint8_t alarmHour;
    uint8_t alarmMinute;

    uint8_t taskId;

    uint16_t gpsuploadonepositiontime;
    uint16_t alarmRequest;
	uint32_t sysMinutes;

    uint32_t gpsRequest;	  /*GPS ��������*/
    uint32_t netRequest;	  /*��������*/
    uint32_t sysTick;    /*ϵͳ����*/
    uint32_t runStartTick;  /*��������*/
    uint32_t gpsUpdatetick;
    uint32_t runningstep;	/*���߲���*/
    uint32_t alreadystep;	/*��һ�δ��Ŀ��ʱ�Ĳ���*/
    uint32_t lastsetp;		/*�ϴβ���*/
    uint32_t runningtime;	/*�˶�ʱ��*/
    uint16_t mode4SysMin;

    float outsidevoltage;
    float insidevoltage;
    float lowvoltage;

    uint8_t canRunFlag;	/*1:��ʾ��ѹ���ʣ���������      2����ʾ��ѹ���ͣ��ػ�*/

    /*�жϴ���*/
	uint8_t ldrIrqFlag				: 1;	/*LDR�����жϱ�־λ*/
	uint8_t ldrDarkCnt;	
	
	uint8_t safeAreaFlag			: 1;
	uint8_t noNetFlag				: 1;
	uint8_t irqTick;
	uint16_t mode4NoNetTick;
	uint16_t outWifiTick;
	uint16_t inWifiTick;
    uint16_t nonetTick;
    uint8_t mode123Min;
    uint16_t mode123RunTick;
    uint8_t ttstick;
    uint8_t ttsContent[60];
	uint8_t mode123GpsFre;
	uint8_t wifiscanCnt;
	uint8_t moduleFsm;
	uint8_t outputLockTick;		//ATָ���������ʱ
	uint16_t moduleWorkStatus;
	uint8_t sockSuccess;
	uint8_t masterSn[16];
} SystemInfoTypedef;

extern SystemInfoTypedef sysinfo;

uint16_t GetCrc16(const char *pData, int nLength);

void LogMessage(uint8_t level, char *debug);
void LogMessageWL(uint8_t level, char *buf, uint16_t len);
void LogPrintf(uint8_t level, const char *debug, ...);
void Log(uint8_t level, const char *debug, ...);
void LogWL(uint8_t level, uint8 *buf, uint16_t len);

uint8_t mycmdPatch(uint8_t *cmd1, uint8_t *cmd2);
int getCharIndex(uint8_t *src, int src_len, char ch);
int my_strpach(char *str1, const char *str2);
int my_getstrindex(char *str1, const char *str2, int len);
int my_strstr(char *str1, const char *str2, int len);
int distinguishOK(char *buf);
int16_t getCharIndexWithNum(uint8_t *src, uint16_t src_len, uint8_t ch, uint8_t num);
void byteToHexString(uint8_t *src, uint8_t *dest, uint16_t srclen);
int16_t changeHexStringToByteArray(uint8_t *dest, uint8_t *src, uint16_t size);
int16_t changeHexStringToByteArray_10in(uint8_t *dest, uint8_t *src, uint16_t size);
void stringToItem(ITEM *item, uint8_t *str, uint16_t len);
void strToUppper(char *data, uint16_t len);

void updateRTCtimeRequest(void);
void byteArrayInvert(uint8 *data, uint8 dataLen);
void stringToLowwer(char *str, uint16_t strlen);
void showByteData(uint8_t *mark, uint8_t *buf, uint16_t len);

char *url_encode(char const *s, int len, int *new_length);
uint8_t encodeUtf8(uint8_t* buf, int value);
int enc_unicode_to_utf8_one(unsigned long unic, unsigned char *pOutput, int outSize);


#endif
