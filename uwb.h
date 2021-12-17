/*
 * uwb.h
 *
 *  Created on: Dec 15, 2021
 *      Author: patrick
 */

#ifndef INC_UWB_H_
#define INC_UWB_H_

#include "cmsis_os2.h"

#ifdef __cplusplus
extern "C" {
#endif


extern osThreadId_t uwbResponderTaskHandle;

extern const osThreadAttr_t uwbResponderTask_attributes;

void UWB_ResponderTask(void *argument);
void UWB_InitiatorTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* INC_UWB_H_ */
