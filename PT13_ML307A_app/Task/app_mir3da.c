/******************** (C) COPYRIGHT 2021 MiraMEMS *****************************
* File Name     : mir3da.c
* Author        : dhzhou@miramems.com
* Version       : V1.0
* Date          : 15/01/2021
* Description   : Demo for configuring mir3da
*******************************************************************************/
#include "app_mir3da.h"
#include <stdio.h>
#include "app_port.h"
#include "app_sys.h"


u8_m i2c_addr = 0x26;

s16_m offset_x = 0, offset_y = 0, offset_z = 0;

s8_m mir3da_register_read(u8_m addr, u8_m *data_m, u8_m len)
{
    //To do i2c read api
    iicReadData(i2c_addr<<1, addr, data_m, len);
    //LogPrintf(DEBUG_ALL, "iicRead %s",ret?"Success":"Fail");
    return 0;
}

s8_m mir3da_register_write(u8_m addr, u8_m data_m)
{
    //To do i2c write api
    iicWriteData(i2c_addr<<1, addr, data_m);
    //LogPrintf(DEBUG_ALL, "iicWrite %s",ret?"Success":"Fail");
    return 0;

}

s8_m mir3da_register_mask_write(unsigned char addr, unsigned char mask, unsigned char data){
    int     res = 0;
    unsigned char      tmp_data;

    res = mir3da_register_read(addr, &tmp_data, 1);
    if(res) {
        return res;
    }

    tmp_data &= ~mask; 
    tmp_data |= data & mask;
    res = mir3da_register_write(addr, tmp_data);

    return res;
}



s8_m read_gsensor_id(void)
{
    u8_m data_m = 0;
    //Retry 3 times
    mir3da_register_read(REG_CHIP_ID, &data_m, 1);
    if (data_m != 0x13)
    {
        mir3da_register_read(REG_CHIP_ID, &data_m, 1);
        if (data_m != 0x13)
        {
            mir3da_register_read(REG_CHIP_ID, &data_m, 1);
            if (data_m != 0x13)
            {
                LogPrintf(DEBUG_FACTORY, "Read gsensor chip id error =%x", data_m);
                return -1;
            }
        }
    }
    LogPrintf(DEBUG_FACTORY, "GSENSOR Chk. ID=0x%X", data_m);
    LogMessage(DEBUG_FACTORY, "GSENSOR CHK OK");
    return data_m;
}

s8_m readInterruptConfig(void)
{
    uint8_t data_m = 0;
    mir3da_register_read(REG_INT_SET1, &data_m, 1);
    if (data_m != 0x87)
    {
        mir3da_register_read(REG_INT_SET1, &data_m, 1);
        if (data_m != 0x87)
        {
            LogPrintf(DEBUG_ALL, "Gsensor fail %x", data_m);
            return -1;
        }
    }
    LogPrintf(DEBUG_ALL, "Gsensor OK");
    return 0;
}


//Initialization
s8_m mir3da_init(void){
	s8_m res = 0;
	u8_m data_m = 0;

  //Retry 3 times
	res = mir3da_register_read(NSA_REG_WHO_AM_I,&data_m,1);
    if(data_m != 0x13){
        res = mir3da_register_read(NSA_REG_WHO_AM_I,&data_m,1);
        if(data_m != 0x13){
            res = mir3da_register_read(NSA_REG_WHO_AM_I,&data_m,1);
            if(data_m != 0x13){
                LogPrintf(DEBUG_ALL, "------mir3da read chip id  error= %x-----\r\n",data_m);  
                return -1;
            }
        }
    }
    mir3da_register_mask_write(0x00, 0x24, 0x24);
	DelayMs(50); //delay 50ms
 
	LogPrintf(DEBUG_ALL, "------mir3da chip id = %x-----\r\n",data_m); 

	res |= mir3da_register_write(NSA_REG_G_RANGE, 0x01);               //+/-4G,14bit
	res |= mir3da_register_write(NSA_REG_POWERMODE_BW, 0x14);          //normal mode
	res |= mir3da_register_write(NSA_REG_ODR_AXIS_DISABLE, 0x07);      //ODR = 125hz
	
	//Engineering mode
	res |= mir3da_register_write(NSA_REG_ENGINEERING_MODE, 0x83);
	res |= mir3da_register_write(NSA_REG_ENGINEERING_MODE, 0x69);
	res |= mir3da_register_write(NSA_REG_ENGINEERING_MODE, 0xBD);
  
	//Reduce power consumption
	if(i2c_addr == 0x26){
		mir3da_register_mask_write(NSA_REG_SENS_COMP, 0x40, 0x00);
	}

	return res;	    	
}



int mir3da_open_step_counter(void)
{
	int res = 0;
	
	res |=  mir3da_register_write(NSA_REG_STEP_CONGIF1, 0x01);
	res |=  mir3da_register_write(NSA_REG_STEP_CONGIF2, 0x62);
	res |=  mir3da_register_write(NSA_REG_STEP_CONGIF3, 0x46);
	res |=  mir3da_register_write(NSA_REG_STEP_CONGIF4, 0x33);
	res |=  mir3da_register_write(NSA_REG_STEP_FILTER, 0xA2);	
	
	return res;
}

int mir3da_close_step_counter(void)
{
	mir3da_register_mask_write(NSA_REG_STEP_FILTER, 0x80, 0x00);	
}


int mir3da_get_step(void)
{
	unsigned char    tmp_data[2] = {0};
	u16_m f_step;

	if((mir3da_register_read(NSA_REG_STEPS_MSB,&tmp_data[0], 1) == 0) && (mir3da_register_read(NSA_REG_STEPS_LSB,&tmp_data[1], 1) == 0))
		f_step = (tmp_data[0] << 8 | tmp_data[1])/2; 

	return (f_step);
}



void mir3da_init_pedometer(void)
{
	mir3da_init();
	mir3da_open_step_counter();
}



//enable/disable the chip
s8_m mir3da_set_enable(u8_m enable)
{
	s8_m res = 0;
	if(enable)
		res = mir3da_register_write(NSA_REG_POWERMODE_BW,0x14);
	else	
		res = mir3da_register_write(NSA_REG_POWERMODE_BW,0x80);
	
	return res;	
}

//Read three axis data, 1024 LSB = 1 g
s8_m mir3da_read_data(s16_m *x, s16_m *y, s16_m *z)
{
    u8_m    tmp_data[6] = {0};


	mir3da_register_read(NSA_REG_ACC_X_LSB, &tmp_data[0], 1);
	mir3da_register_read(NSA_REG_ACC_X_MSB, &tmp_data[1], 1);
	mir3da_register_read(NSA_REG_ACC_Y_LSB, &tmp_data[2], 1);
	mir3da_register_read(NSA_REG_ACC_Y_MSB, &tmp_data[3], 1);
	mir3da_register_read(NSA_REG_ACC_Z_LSB, &tmp_data[4], 1);
	mir3da_register_read(NSA_REG_ACC_Z_MSB, &tmp_data[5], 1);
	
    *x = ((s16_m)(tmp_data[1] << 8 | tmp_data[0]))>> 3;
    *y = ((s16_m)(tmp_data[3] << 8 | tmp_data[2]))>> 3;
    *z = ((s16_m)(tmp_data[5] << 8 | tmp_data[4]))>> 3;	
	

    return 0;
}

//open active interrupt
s8_m mir3da_open_interrupt(u8_m th){
	s8_m   res = 0;

	res = mir3da_register_write(NSA_REG_ACTIVE_DURATION,0x02);
	res = mir3da_register_write(NSA_REG_ACTIVE_THRESHOLD,th);
	res = mir3da_register_write(NSA_REG_INTERRUPT_MAPPING1,0x04);
	res = mir3da_register_write(NSA_REG_INT_LATCH, 0xEE);  //latch 100ms
	res = mir3da_register_write(NSA_REG_INTERRUPT_SETTINGS1,0x87);

	return res;
}

//close active interrupt
s8_m mir3da_close_interrupt(void){
	s8_m   res = 0;

	res = mir3da_register_write(NSA_REG_INTERRUPT_SETTINGS1,0x00 );
	res = mir3da_register_write(NSA_REG_INTERRUPT_MAPPING1,0x00 );

	return res;
}



void startStep(void)
{
    mir3da_open_step_counter();
}
void stopStep(void)
{
    mir3da_close_step_counter();
}
u16_m getStep(void)
{
    u16_m step;
    step = mir3da_get_step();
    return step;
}
void clearStep(void)
{
    mir3da_close_step_counter();
}


