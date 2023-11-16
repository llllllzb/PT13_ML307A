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

#define CMD_GET_SHIELD_CNT               	0x00  //��ȡ���δ���
#define CMD_CLEAR_SHILED_CNT             	0x01  //���������
#define CMD_RELAY_ON                  		0x02  //������ͨ�̵���
#define CMD_RELAY_OFF                 		0x03  //�����Ͽ��̵���
#define CMD_SET_SHIELD_VOL             		0x04  //�������ε�ѹ��ֵ
#define CMD_GET_SHIELD_VOL             		0x05  //��ȡ���ε�ѹ��ֵ
#define CMD_GET_CUR_SHILEDVOL               0x06  //��ȡ���ε�ѹ
#define CMD_SET_OUTSIDE_VOL          		0x07  //�����ⲿ��ѹ��ֵ
#define CMD_GET_OUTSIDE_VOL          		0x08  //��ȡ�ⲿ��ѹ��ֵ
#define CMD_GET_CUR_OUTSIDEVOL              0x09  //��ȡ�ⲿ��ѹֵ
#define CMD_GET_RELAY_STATE               	0x0A  //��ȡ�̵���״̬
#define CMD_SEND_SHIELD_ALARM               0x0B  //��������
#define CMD_SET_SPEED_FLAG              	0x0C  //�ٶȱ�־
#define CMD_SET_DISCONNECT_PARAM            0x0D  //���õ���ʱ����
#define CMD_CLEAR_SHIELD_ALARM             	0x0E  //�������
#define CMD_SEND_PRE_ALARM                	0x0F  //Ԥ������
#define CMD_CLEAR_PRE_ALARM          		0x10  //���Ԥ��
#define CMD_OTA                             0x11  //����OTA
#define CMD_VER                             0x12  //��version
#define CMD_SET_PRE_ALARM_PARAM             0x13  //��������Ԥ������
#define CMD_GET_PRE_ALARM_PARAM             0x14  //��ȡ����Ԥ������
#define CMD_GET_DISCONNECT_PARAM            0x15  //��ȡ����ʱ����
#define CMD_GET_SOCKET_STATUS				0x18  //��ȡ��������״̬
#define CMD_SEND_SHIELD_LOCK_ALARM			0x19  //����������������
#define CMD_RES_SHIELD_LOCK_ALRAM			0x20  //��������Ӧ��
#define CMD_CLEAR_SHIELD_LOCK_ALRAM			0x21  //������������¼�

/*��ʱ����*/
#define CMD_SET_SHIEL_ALARM_HOLD_PARAM		0x16  //�������ε�ѹ���ֲ���
#define CMD_GET_SHIEL_ALARM_HOLD_PARAM		0x17  //��ȡ���ε�ѹ���ֲ���

/**
 * �豸���Ϻ�����ͨ��CMD_DEV_LOGIN_INFOѯ�ʴӻ���SN��
 * �ӻ����������SN�ź�socksuccess״̬�����socksuccessΪ1����˳�������SN����Ϣ��Ҳ������PT13�ϱ��������豸�ţ�Ҳ���͸�����
 * ����һ��ȷ��PT13����ΪsocksuccessΪ0�������п�����·���ͻᷢCMD_DEV_MASTER_INFO��Ϣ��֪PT13������SN���ҽ�socksuccess��1
 * login״̬�ڳ���3������Э��ͨѶ�Żᱻ���������������CMD_DEV_MASTER_INFO��socksuccess���
 * login�и��£������ڹ㲥�������֣��������������ظ�����
 */
#define CMD_DEV_LOGIN_INFO                  0x30  //�����ϱ���Ϣ
#define CMD_DEV_HEARTBEAT					0x31
#define CMD_DEV_MASTER_INFO					0x32  //����״̬


#define CMD_SET_BROADCAST_NAME          	0xA0//�޸Ĺ㲥����
#define CMD_SET_MODIFY_ADVINTER         	0xA1//�޸Ĺ㲥���
#define CMD_SET_CONINTER         			0xA2//�޸����Ӽ��
#define CMD_HEARTBEAT						0xA3//ͨѶ����
#define CMD_SET_TXPOWER             		0xA7//���书�ʿ���
#define CMD_SET_RTC              			0xA8//����RTCʱ��

#define CMD_DECVICE_CMD						0xA9//ָ��͸��
#define CMD_AUTH_CMD						0xAA//��Ȩ
#define CMD_ONLINE_CTL_CMD					0xB0//ǿ�ƿ���
#define CMD_MODE_CMD						0xB1//�޸�ģʽ


#define CMD_ADCCAL                          0xE0//ADCУ׼

#define CMD_GENERAL_RESPON           		0x80//�豸�ظ�
#define CMD_DECVICE_CMD_RESPON				0x81//ָ��ظ�



uint8_t appPackProtocol(uint8_t *dest,uint8_t cmd,uint8_t *data,uint8_t len);
void bleProtocolParser(uint16_t connhandle, uint8_t *data, uint16_t len);

void bleGeneratePsdProtocol(uint8_t *psd);
void bleSendHbtInfo(uint16_t connhandle);
void bleSendLoginInfo(uint16_t connhandle);
void bleMasterInfo(uint16_t connhandle, char *sn, uint8_t ind, uint8_t socksuccess);



#endif
