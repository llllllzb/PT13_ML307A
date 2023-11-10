/*
 * app_key.h
 *
 *  Created on: Jun 15, 2022
 *      Author: nimo
 */

#ifndef APP_INCLUDE_APP_KEY_H_
#define APP_INCLUDE_APP_KEY_H_

#include "stdint.h"
#include "config.h"
#include "app_port.h"
#include "app_task.h"
#include "app_kernal.h"
#include "app_sys.h"

#define KEY_ON      0
#define KEY_OFF     1
typedef enum
{
    PRESS,
    CLICK,
    DOUBLECLICK,
    TRIPLECLICK,
}KEYFLAGTYPEDEF;

typedef struct
{
    uint8_t click;
    uint8_t longpress;
    uint8_t triclick;
	uint8_t down			: 1;
	uint8_t clossprocess	: 1;	//正在关机
    uint8_t release;
    uint8_t presscomplete;
    
    KEYFLAGTYPEDEF keystatus;


}keyctrl_t;

extern keyctrl_t soskey;

void systemShutDownSuccess(void);
void systemShutDownTimeout(void);

void keyInit(void);
void keyExcuteByStatus(void);
void keyTask(void);


#endif /* APP_INCLUDE_APP_KEY_H_ */
