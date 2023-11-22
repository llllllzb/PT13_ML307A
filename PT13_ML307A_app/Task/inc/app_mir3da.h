#ifndef APP_MIR3DA
#define APP_MIR3DA


#include "app_mir3da.h"
/*******************************************************************************
Typedef definitions
********************************************************************************/
#define ARM_BIT_8               0

#if ARM_BIT_8
    //����������������8λ���϶���ģ�������ƽ̨������32λ�����ܴ��ڲ����Ҫ����ʵ������޸� ��
    typedef unsigned char    u8_m;                   /* �޷���8λ���ͱ���*/
    typedef signed   char    s8_m;                   /* �з���8λ���ͱ���*/
    typedef unsigned int     u16_m;                  /* �޷���16λ���ͱ���*/
    typedef signed   int     s16_m;                  /* �з���16λ���ͱ���*/
    typedef unsigned long    u32_m;                  /* �޷���32λ���ͱ���*/
    typedef signed   long    s32_m;                  /* �з���32λ���ͱ���*/
    typedef float            fp32_m;                 /* �����ȸ�������32λ���ȣ�*/
    typedef double           fp64_m;                 /* ˫���ȸ�������64λ���ȣ�*/
#else
    //����������������32λ���϶���ģ�������ƽ̨������8λ�����ܴ��ڲ����Ҫ����ʵ������޸� ��
    typedef unsigned char    u8_m;                   /* �޷���8λ���ͱ���*/
    typedef signed   char    s8_m;                   /* �з���8λ���ͱ���*/
    typedef unsigned short   u16_m;                  /* �޷���16λ���ͱ���*/
    typedef signed   short   s16_m;                  /* �з���16λ���ͱ���*/
    typedef unsigned int     u32_m;                  /* �޷���32λ���ͱ���*/
    typedef signed   int     s32_m;                  /* �з���32λ���ͱ���*/
    typedef float            fp32_m;                 /* �����ȸ�������32λ���ȣ�*/
    typedef double           fp64_m;                 /* ˫���ȸ�������64λ���ȣ�*/
#endif


/*******************************************************************************
Macro definitions - Register define for Gsensor
********************************************************************************/
#define REG_SPI_CONFIG              0x00
#define REG_CHIP_ID                 0x01
#define REG_ACC_X_LSB               0x02
#define REG_ACC_X_MSB               0x03
#define REG_ACC_Y_LSB               0x04
#define REG_ACC_Y_MSB               0x05
#define REG_ACC_Z_LSB               0x06
#define REG_ACC_Z_MSB               0x07
#define REG_FIFO_STATUS             0x08
#define REG_MOTION_FLAG             0x09
#define REG_STEPS_MSB               0x0D
#define REG_STEPS_LSB               0x0E
#define REG_RESOLUTION_RANGE        0x0F
#define REG_MODE_ODR                0x10
#define REG_MODE_AXIS               0x11
#define REG_SWAP_POLARITY           0x12
#define REG_FIFO_CTRL               0x14
#define REG_INT_SET0                0x15
#define REG_INT_SET1                0x16
#define REG_INT_SET2                0x17
#define REG_INT_MAP1                0x19
#define REG_INT_MAP2                0x1A
#define REG_INT_MAP3                0x1B
#define REG_INT_CONFIG              0x20
#define REG_INT_LATCH               0x21
#define REG_FREEFALL_DUR            0x22
#define REG_FREEFALL_THS            0x23
#define REG_FREEFALL_HYST           0x24
#define REG_TAP_QUIET               0x29
#define REG_TAP_DUR                 0x2A
#define REG_TAP_THS                 0x2B
#define REG_ORIENT_HYST             0x2C
#define REG_Z_BLOCK                 0x2D
#define REG_RESET_STEP              0x2E
#define REG_STEP_FILTER             0x33
#define	REG_SM_THRESHOLD            0x34
#define REG_ACTIVE_DUR              0x38
#define REG_ACTIVE_X_THS            0x39
#define REG_ACTIVE_Y_THS            0x3A
#define REG_ACTIVE_Z_THS            0x3B


#define DIFF_CALIBRATION 1




#define NSA_REG_SPI_I2C                 0x00
#define NSA_REG_WHO_AM_I                0x01
#define NSA_REG_ACC_X_LSB               0x02
#define NSA_REG_ACC_X_MSB               0x03
#define NSA_REG_ACC_Y_LSB               0x04
#define NSA_REG_ACC_Y_MSB               0x05
#define NSA_REG_ACC_Z_LSB               0x06
#define NSA_REG_ACC_Z_MSB               0x07 
#define NSA_REG_MOTION_FLAG             0x09 
#define NSA_REG_G_RANGE                 0x0f
#define NSA_REG_ODR_AXIS_DISABLE        0x10
#define NSA_REG_POWERMODE_BW            0x11
#define NSA_REG_SWAP_POLARITY           0x12
#define NSA_REG_FIFO_CTRL               0x14
#define NSA_REG_INTERRUPT_SETTINGS0     0x15
#define NSA_REG_INTERRUPT_SETTINGS1     0x16
#define NSA_REG_INTERRUPT_SETTINGS2     0x17
#define NSA_REG_INTERRUPT_MAPPING1      0x19
#define NSA_REG_INTERRUPT_MAPPING2      0x1a
#define NSA_REG_INTERRUPT_MAPPING3      0x1b
#define NSA_REG_INT_PIN_CONFIG          0x20
#define NSA_REG_INT_LATCH               0x21
#define NSA_REG_ACTIVE_DURATION         0x27
#define NSA_REG_ACTIVE_THRESHOLD        0x28
#define NSA_REG_TAP_DURATION            0x2A
#define NSA_REG_TAP_THRESHOLD           0x2B
#define NSA_REG_ENGINEERING_MODE        0x7f
#define NSA_REG_SENS_COMP               0x8c
#define NSA_REG_MEMS_OPTION             0x8f
#define NSA_REG_CHIP_INFO               0xc0

#define NSA_REG_STEPS_MSB				0x0D
#define NSA_REG_STEPS_LSB				0x0E
#define NSA_REG_RESET_STEP			    0x2E
#define NSA_REG_STEP_CONGIF1			0x2F
#define NSA_REG_STEP_CONGIF2			0x30
#define NSA_REG_STEP_CONGIF3			0x31
#define NSA_REG_STEP_CONGIF4			0x32
#define NSA_REG_STEP_FILTER             0x33


s8_m mir3da_open_interrupt(u8_m th);

s8_m mir3da_read_data(s16_m *x, s16_m *y, s16_m *z);

s8_m mir3da_init(void);
s8_m read_gsensor_id(void);
s8_m readInterruptConfig(void);
s8_m mir3da_set_enable(u8_m enable);
void startStep(void);
void stopStep(void);
u16_m getStep(void);
void clearStep(void);
#endif
