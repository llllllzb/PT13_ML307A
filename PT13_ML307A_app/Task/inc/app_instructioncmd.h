#ifndef APP_INSTRUCTION_H
#define APP_INSTRUCTION_H

#include <stdint.h>


typedef enum
{
    DEBUG_MODE,
    SMS_MODE,
    NET_MODE,
    BLE_MODE,
    JT808_MODE,
} insMode_e;

typedef enum
{
    PARAM_INS,
    STATUS_INS,
    VERSION_INS,
    SERVER_INS,
    MODE_INS,
    HBT_INS,
    POSITION_INS,
    APN_INS,
    UPS_INS,
    LOWW_INS,
    LED_INS,
    POITYPE_INS,
    RESET_INS,
    UTC_INS,
    DEBUG_INS,
    ACCCTLGNSS_INS,
    ACCDETMODE_INS,
    FENCE_INS,
    FACTORY_INS,
    ICCID_INS,
    SETAGPS_INS,
    JT808SN_INS,
    HIDESERVER_INS,
    BLESERVER_INS,
    TIMER_INS,
    SOS_INS,
    CENTER_INS,
    SOSALM_INS,
    BLELINKFAIL_INS,
    ALARMMODE_INS,
    AGPSEN_INS,
    SETWIFIMAC_INS,
    SETBLEPSW_INS,
    NONWIFIPARAM_INS,
    WITHWIFIPARAM_INS,
    RSWIFIFENCE_INS,
    PETLED_INS,
    PETSPK_INS,
    UART_INS,
    PETDEBUG_INS,
    STEPPARAM_INS,
    SYSTEMSHUTDOWN_INS,
    VOLUME_INS,
    SPORTS_INS,
    PETBELL_INS,
    DOWNLOADFILE_INS,
    MUSICLIST_INS,
} INSTRUCTIONID;



typedef struct
{
    uint16_t cmdid;
    char    *cmdstr;
} instruction_s;

typedef struct
{
    char *telNum;
    char *data;
    uint16_t len;
    uint8_t link;
    uint8_t bleConhandle;
} insParam_s;
extern insParam_s lastparam;
extern int rspTimeOut;
void instructionParser(uint8_t *str, uint16_t len, insMode_e mode, void *param);
void dorequestSend123(void);
void instructionRespone(char *message);

#endif
