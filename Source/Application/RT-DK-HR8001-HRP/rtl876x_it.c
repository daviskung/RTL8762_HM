/**
*********************************************************************************************************
*               Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
**********************************************************************************************************
* @file		rtl876x_it.c
* @brief	adc demo--one shot mode
* @details
* @author	tifnan
* @date 	2015-06-01
* @version	v0.1
*********************************************************************************************************
*/

#include "freeRTOS.h"
#include "task.h"
#include "queue.h"
#include "rtl876x_adc.h"
#include "board.h"
#include "trace.h"
#include "pah8001.h"

void ADCIntrHandler(void)
{
    uint8_t event = EVENT_ADC_CONVERT_BUF_FULL;
    portBASE_TYPE TaskWoken = pdFALSE;
    
    ADC_Cmd(ADC, ADC_One_Shot_Mode, DISABLE);
    
    ADC_ClearINTPendingBit(ADC, ADC_INT_ONE_SHOT_DONE);
    
    /* read ADC convert result*/
    extern uint16_t adcConvertRes_HM[];
    
    adcConvertRes_HM_cnt++;
	if ( adcConvertRes_HM_cnt > 99 ) adcConvertRes_HM_cnt=0;
    adcConvertRes_HM[adcConvertRes_HM_cnt] = ADC_Read(ADC, ADC_CH1);
    

    /* send convert result to task*/

    /* send event to heartrate_task_app */
	if ( adcConvertRes_HM_cnt == 99 )
    xQueueSendFromISR(hHeartRateQueueHandle, &event, &TaskWoken);
    
    portEND_SWITCHING_ISR(TaskWoken);
    
    return;
}
