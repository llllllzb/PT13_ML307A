#ifndef APP_BLEPROTOCOL
#define APP_BLEPROTOCOL
#include "CH58x_common.h"

#define APP_TX_POWEER_MINUS_20_DBM                 1
#define APP_TX_POWEER_MINUS_14_DBM                 2
#define APP_TX_POWEER_MINUS_8_DBM                  3
#define APP_TX_POWEER_MINUS_4_DBM                  4
#define APP_TX_POWEER_0_DBM                        5
#define APP_TX_POWEER_1_DBM                        6
#define APP_TX_POWEER_2_DBM                        7
#define APP_TX_POWEER_3_DBM                        8
#define APP_TX_POWEER_4_DBM                        9
#define APP_TX_POWEER_5_DBM                        10

#define CMD_GET_SHIELD_CNT               	0x00  //获取屏蔽次数
#define CMD_CLEAR_SHILED_CNT             	0x01  //清除计数器
#define CMD_RELAY_ON                  		0x02  //主动接通继电器
#define CMD_RELAY_OFF                 		0x03  //主动断开继电器
#define CMD_SET_SHIELD_VOL             		0x04  //设置屏蔽电压阈值
#define CMD_GET_SHIELD_VOL             		0x05  //读取屏蔽电压阈值
#define CMD_GET_CUR_SHILEDVOL               0x06  //读取屏蔽电压
#define CMD_SET_OUTSIDE_VOL          		0x07  //设置外部电压阈值
#define CMD_GET_OUTSIDE_VOL          		0x08  //读取外部电压阈值
#define CMD_GET_CUR_OUTSIDEVOL              0x09  //读取外部电压值
#define CMD_GET_RELAY_STATE               	0x0A  //读取继电器状态
#define CMD_SEND_SHIELD_ALARM               0x0B  //报警发送
#define CMD_SET_SPEED_FLAG              	0x0C  //速度标志
#define CMD_SET_DISCONNECT_PARAM            0x0D  //设置倒计时参数
#define CMD_CLEAR_SHIELD_ALARM             	0x0E  //清除报警
#define CMD_SEND_PRE_ALARM                	0x0F  //预警发送
#define CMD_CLEAR_PRE_ALARM          		0x10  //清除预警
#define CMD_OTA                             0x11  //进入OTA
#define CMD_VER                             0x12  //读version
#define CMD_SET_PRE_ALARM_PARAM             0x13  //设置屏蔽预警参数
#define CMD_GET_PRE_ALARM_PARAM             0x14  //读取屏蔽预警参数
#define CMD_GET_DISCONNECT_PARAM            0x15  //读取倒计时参数
#define CMD_GET_SOCKET_STATUS				0x18  //获取主机网络状态
#define CMD_SEND_SHIELD_LOCK_ALARM			0x19  //发送屏蔽锁车报警
#define CMD_RES_SHIELD_LOCK_ALRAM			0x20  //屏蔽锁车应答
#define CMD_CLEAR_SHIELD_LOCK_ALRAM			0x21  //清除屏蔽锁车事件

/*暂时不用*/
#define CMD_SET_SHIEL_ALARM_HOLD_PARAM		0x16  //设置屏蔽电压保持参数
#define CMD_GET_SHIEL_ALARM_HOLD_PARAM		0x17  //读取屏蔽电压保持参数

/**
 * 设备连上后，主机通过CMD_DEV_LOGIN_INFO询问从机的SN号
 * 从机发送自身的SN号和socksuccess状态，如果socksuccess为1，会顺便把主机SN号信息（也就是替PT13上报的主机设备号）也发送给主机
 * 主机一旦确认PT13处于为socksuccess为0且自身有空余链路，就会发CMD_DEV_MASTER_INFO信息告知PT13主机的SN号且将socksuccess置1
 * login状态在持续3分钟无协议通讯才会被清除或者主机发送CMD_DEV_MASTER_INFO将socksuccess清除
 * login有更新，立即在广播包上体现，以免其他主机重复连接
 */
#define CMD_DEV_LOGIN_INFO                  0x30  //蓝牙上报信息
#define CMD_DEV_HEARTBEAT					0x31
#define CMD_DEV_MASTER_INFO					0x32  //主机状态


#define CMD_SET_BROADCAST_NAME          	0xA0//修改广播名称
#define CMD_SET_MODIFY_ADVINTER         	0xA1//修改广播间隔
#define CMD_SET_CONINTER         			0xA2//修改连接间隔
#define CMD_HEARTBEAT						0xA3//通讯心跳
#define CMD_SET_TXPOWER             		0xA7//发射功率控制
#define CMD_SET_RTC              			0xA8//更新RTC时间

#define CMD_DECVICE_CMD						0xA9//指令透传
#define CMD_AUTH_CMD						0xAA//鉴权
#define CMD_ONLINE_CTL_CMD					0xB0//强制开启
#define CMD_MODE_CMD						0xB1//修改模式


#define CMD_ADCCAL                          0xE0//ADC校准

#define CMD_GENERAL_RESPON           		0x80//设备回复
#define CMD_DECVICE_CMD_RESPON				0x81//指令回复



uint8_t appPackProtocol(uint8_t *dest,uint8_t cmd,uint8_t *data,uint8_t len);
void bleProtocolParser(uint16_t connhandle, uint8_t *data, uint16_t len);

void bleGeneratePsdProtocol(uint8_t *psd);
void bleSendHbtInfo(uint16_t connhandle);
void bleSendLoginInfo(uint16_t connhandle);
void bleMasterInfo(uint16_t connhandle, char *sn, uint8_t ind, uint8_t socksuccess);



#endif
