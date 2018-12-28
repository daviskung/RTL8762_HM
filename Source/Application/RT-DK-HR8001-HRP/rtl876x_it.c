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
#include "rtl876x_it.h"
#include "rtl876x_uart.h"
#include "rtl876x_gpio.h"

extern xQueueHandle hHeartRateQueueHandle;
/* globals */
uint8_t RxBuffer[32];
uint8_t RxCount;
uint8_t RxEndFlag;

uint8_t key3status;



void ADCIntrHandler(void)
{
    //uint8_t event = EVENT_ADC_CONVERT_BUF_FULL;
    portBASE_TYPE TaskWoken = pdFALSE;
    
    ADC_Cmd(ADC, ADC_One_Shot_Mode, DISABLE);
    
    ADC_ClearINTPendingBit(ADC, ADC_INT_ONE_SHOT_DONE);
    
    /* read ADC convert result*/
    //extern uint16_t adcConvertRes_HM[];
    
	/*
    adcConvertRes_HM_cnt++;
	if ( adcConvertRes_HM_cnt > 99 ) adcConvertRes_HM_cnt=0;
    adcConvertRes_HM[adcConvertRes_HM_cnt] = ADC_Read(ADC, ADC_CH1);
    */

    /* send convert result to task*/

    /* send event to heartrate_task_app */
	//if ( adcConvertRes_HM_cnt == 99 )
  //  xQueueSendFromISR(hHeartRateQueueHandle, &event, &TaskWoken);
    
    portEND_SWITCHING_ISR(TaskWoken);
    
    return;
}




void DataUartIntrHandler(void)
{
    uint32_t int_status = 0;	
	uint8_t _hr_event;
    
    /* read interrupt id */
    int_status = UART_GetIID(UART);
    /* disable interrupt */
    UART_INTConfig(UART, UART_INT_RD_AVA | UART_INT_LINE_STS, DISABLE);
    
    switch(int_status)
    {
        /* tx fifo empty, not enable */
        case UART_INT_ID_TX_EMPTY:
        break;

        /* rx data valiable */
        case UART_INT_ID_RX_LEVEL_REACH:
            UART_ReceiveData(UART, &RxBuffer[RxCount], 1); // 是資料 分隔 的 數量
            RxCount+= 1;
            break;
        
        case UART_INT_ID_RX_TMEOUT:
            while(UART_GetFlagState(UART, UART_FLAG_RX_DATA_RDY) == SET)
            {
                RxBuffer[RxCount] = UART_ReceiveByte(UART);
                RxCount++;
            }
            //if((RxBuffer[RxCount-1] == '\r') && (RxBuffer[RxCount-2] == '\n'))
			if((RxBuffer[RxCount-1] == '\n') && (RxBuffer[RxCount-2] == '\r'))
			{
                RxEndFlag = 1;
				
				// Send Task
				_hr_event = EVENT_RxEndFlag_SET;
				xQueueSend(hHeartRateQueueHandle, &_hr_event, 1);
            }
        break;
        
        /* receive line status interrupt */
        case UART_INT_ID_LINE_STATUS:
            DBG_BUFFER(MODULE_APP, LEVEL_ERROR, "Line status error!!!!\n", 0);
        break;      

        default:
        break;
    }
    
    /* enable interrupt again */
    UART_INTConfig(UART, UART_INT_RD_AVA | UART_INT_LINE_STS, ENABLE);
    
    return;
}
#if 0
void Gpio27IntrHandler(void)	// P3_3 KEY4
{
	
	uint8_t _hr_event;
    GPIO_MaskINTConfig(GPIO_KEY4_Pin, ENABLE);
    
    key3status = GPIO_ReadInputDataBit(GPIO_KEY4_Pin);
    if (key3status == 0)
    {
       // Send Task
		_hr_event = EVENT_KEY4_PUSH_SET;
		xQueueSend(hHeartRateQueueHandle, &_hr_event, 1);
    }
    else
    {
        // Send Task
		_hr_event = EVENT_KEY4_RELEASE_SET;
		xQueueSend(hHeartRateQueueHandle, &_hr_event, 1);
    }
    
    GPIO_ClearINTPendingBit(GPIO_KEY4_Pin);
    GPIO_MaskINTConfig(GPIO_KEY4_Pin, DISABLE);
}
#endif


void Gpio31IntrHandler(void)	// P4_3 S2 PWR_KEY
{
	
	uint8_t _hr_event;
	
	uint8_t PwrKeyStatus;
    GPIO_MaskINTConfig(GPIO_PWR_KEY_PIN, ENABLE);
    
    PwrKeyStatus = GPIO_ReadInputDataBit(GPIO_PWR_KEY_PIN);
    if (PwrKeyStatus == SET)
    {
       // Send Task
		_hr_event = EVENT_PWR_KEY_PUSH_SET;
		xQueueSend(hHeartRateQueueHandle, &_hr_event, 1);
    }
    else
    {
        // Send Task
		_hr_event = EVENT_PWR_KEY_RELEASE_SET;
		xQueueSend(hHeartRateQueueHandle, &_hr_event, 1);
    }
    
    GPIO_ClearINTPendingBit(GPIO_PWR_KEY_PIN);
    GPIO_MaskINTConfig(GPIO_PWR_KEY_PIN, DISABLE);
}


