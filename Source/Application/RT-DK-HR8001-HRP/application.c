/**
*********************************************************************************************************
*               Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
**********************************************************************************************************
* @file     application.c
* @brief
* @details
* @author   hunter_shuai
* @date     23-June-2015
* @version  v1.0.0
******************************************************************************
* @attention
* <h2><center>&copy; COPYRIGHT 2015 Realtek Semiconductor Corporation</center></h2>
******************************************************************************
*/

#include "rtl876x.h"
#include "application.h"
#include "dlps_platform.h"
#include "SimpleBLEPeripheral.h"
#include "SimpleBLEPheripheral_api.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "blueapi.h"
#include "peripheral.h"
#include "timers.h"

#include "rtl876x_gpio.h"
#include "rtl876x_nvic.h"

//#include "stk8ba50.h"
#include "pah8001.h"

#include "rtl876x_adc.h"
#include "rtl876x_lib_adc.h"
#include "rtl876x_rcc.h"
#include "rtl876x_tim.h"
#include "rtl_delay.h"

#include <stdio.h>
#include "rtl876x_uart.h"


uint8_t KEYscan_fun_cnt2;


extern void AppHandleIODriverMessage(BEE_IO_MSG io_driver_msg_recv);
extern void Driver_Init(void);

extern uint8_t _touch_flag;
extern uint8_t	NSTROBE_LOW_EndSet;

UINT8 key_cnt,KEYscan_fun_cnt,cnt500ms;  // current setting

extern uint8_t PWM_chanEN_Number;


/* globals */
extern uint8_t RxBuffer[32];
extern uint8_t RxCount;
extern uint8_t RxEndFlag;
uint8_t uRxCnt=30;



uint8_t uTxBuf[128];
uint8_t EnPICcmdBuf[5];
//uint8_t uGetFromPICBuf[9];
uint8_t uGetFromPICBuf[7];


uint16_t uTxCnt;

uint8_t _myHR;

uint8_t NoSignalShutdownCnt;



/****************************************************************************/
/* Events                                                                   */
/****************************************************************************/
#define BLUEAPI_MSG_EVENT				0x01
#define EVENT_IODRIVER_TO_APP			0x05

#define BROADCASTER_TASK_PRIORITY		(tskIDLE_PRIORITY + 1)   /* Task priorities. */
#define BROADCASTER_TASK_STACK_SIZE		1024*2

#define MAX_NUMBER_OF_RX_EVENT			0x20
#define MAX_NUMBER_OF_MESSAGE			0x20
#define MAX_HEARTRATE_TASK_COUNT		0x20

 /* event */
#define IO_DEMO_EVENT_ADC_CONVERT_END          0x01

#define	PWR_KEY_OFF_TIME_SET	5	


xTaskHandle  hOTAAppTaskHandle;
xTaskHandle  hHeartRateAppTaskHandle;

xQueueHandle hEventQueueHandle;
xQueueHandle hMessageQueueHandle;
xQueueHandle hIoQueueHandle;
xQueueHandle hHeartRateQueueHandle = NULL;
xTimerHandle hPAH8001_Timer = NULL;

xTimerHandle hADC_AR_CH1_Timer = NULL;
xTimerHandle hIR_PWM_Timer = NULL;
xTimerHandle hKEYscan_Timer = NULL;


void peripheralBlueAPIMessageCB(PBlueAPI_UsMessage pMsg)
{
	uint8_t Event = BLUEAPI_MSG_EVENT;

    if (xQueueSend(hMessageQueueHandle, &pMsg, 0) == errQUEUE_FULL)
    {
        blueAPI_BufferRelease(pMsg);
    }
    else if (xQueueSend(hEventQueueHandle, &Event, 0) == errQUEUE_FULL)
    {

    }
}

void bee_task_app(void *pvParameters);
void heartrate_task_app(void *pvParameters);

void application_task_init()
{
    /* Create APP Task. */
    xTaskCreate(bee_task_app,				/* Pointer to the function that implements the task. */
                "APPTask",					/* Text name for the task.  This is to facilitate debugging only. */
                256,						/* Stack depth in words. 1KB*/
                NULL,						/* We are not using the task parameter. */
                1,							/* This task will run at priority 1. */
                &hOTAAppTaskHandle);		/* the task handle. */

    /* Create APP Task. */
    xTaskCreate(heartrate_task_app,			/* Pointer to the function that implements the task. */
                "HR_APPTask",				/* Text name for the task.  This is to facilitate debugging only. */
                256,						/* Stack depth in words. 1KB*/
                NULL,						/* We are not using the task parameter. */
                2,							/* This task will run at priority 1. */
                &hHeartRateAppTaskHandle);	/* the task handle. */

	/* enable interrupt again */
    UART_INTConfig(UART, UART_INT_RD_AVA | UART_INT_LINE_STS, ENABLE);

	GPIO_WriteBit(GPIO_STATUS_LED_PIN,Bit_SET); // STATUS LED ON
	DBG_BUFFER(MODULE_APP, LEVEL_INFO, "** STATUS_LED_PIN ON !\n", 0);

	if( GPIO_ReadInputDataBit(GPIO_USB_V5_IN_PIN) == SET )
		DBG_BUFFER(MODULE_APP, LEVEL_INFO, "** USB 5V plugin!\n", 0);
	else
	{
		DBG_BUFFER(MODULE_APP, LEVEL_INFO, "** Battery power ON !\n", 0);
		GPIO_WriteBit(GPIO_PWR_CONTROL_PIN,Bit_SET); // PWR_CONTROL_PIN ON
		DBG_BUFFER(MODULE_APP, LEVEL_INFO, "** PWR_CONTROL_PIN ON !\n", 0);
	}
	
}


/**
* @brief
*
*
* @param   pvParameters
* @return  void
*/
void bee_task_app(void *pvParameters )
{
	uint8_t Event;

	hMessageQueueHandle = xQueueCreate(MAX_NUMBER_OF_MESSAGE,
										sizeof(PBlueAPI_UsMessage));

	hIoQueueHandle = xQueueCreate(MAX_NUMBER_OF_MESSAGE,
									sizeof(BEE_IO_MSG));

	hEventQueueHandle = xQueueCreate(MAX_NUMBER_OF_MESSAGE + MAX_NUMBER_OF_RX_EVENT,
										sizeof(unsigned char));

	peripheral_StartBtStack();

	Driver_Init();

	while(true)
    {
		if(xQueueReceive(hEventQueueHandle, &Event, portMAX_DELAY) == pdPASS)
		{
			if(Event == BLUEAPI_MSG_EVENT)  /* BlueAPI */
			{
				PBlueAPI_UsMessage pMsg;

				while(xQueueReceive(hMessageQueueHandle, &pMsg, 0) == pdPASS)
				{
					peripheral_HandleBlueAPIMessage(pMsg);
				}
			}
			else if(Event == EVENT_NEWIODRIVER_TO_APP)
			{
				BEE_IO_MSG io_driver_msg_recv;

				if(xQueueReceive(hIoQueueHandle, &io_driver_msg_recv, 0) == pdPASS)
				{
					AppHandleIODriverMessage(io_driver_msg_recv);
				}
			}
		}
    }
}


void heartrate_task_app(void *pvParameters)
{
	uint8_t Event;
	
//	uint8_t blkcount= 0;
//    uint8_t remainder = 0;
    uint8_t PWRkey_timer_cnt = 0;
    uint8_t i = 0;

	if(hPAH8001_Timer == NULL)
	{
		hPAH8001_Timer = xTimerCreate(	"SENSOR_PAH8001_TIMER",			// Just a text name, not used by the kernel.
										(SENSOR_PAH8001_INTERVAL/portTICK_RATE_MS),	// The timer period in ticks. 
										pdFALSE,						// The timers will auto-reload themselves when they expire.
										(void *)SENSOR_PAH8001_TIMER_ID,						// Assign each timer a unique id equal to its array index.
										(TimerCallbackFunction_t) Pixart_HRD);
	}

	DBG_BUFFER(MODULE_APP, LEVEL_INFO, " ***hPAH8001_Timer value = 0x%x \n", 1,hPAH8001_Timer);

	if(hADC_AR_CH1_Timer == NULL)
	{
		hADC_AR_CH1_Timer = xTimerCreate("HM_ADC_AR_TIMER",			// Just a text name, not used by the kernel.
										(HM_ADC_INTERVAL/portTICK_RATE_MS),	// The timer period in ticks. 
										pdFALSE,						// The timers will auto-reload themselves when they expire.
										(void *)HM_ADC_AR_TIMER_ID,						// Assign each timer a unique id equal to its array index.
										(TimerCallbackFunction_t) AR_ADC_CH1);
	}

	DBG_BUFFER(MODULE_APP, LEVEL_INFO, " ***hADC_AR_CH1_Timer value = 0x%x \n", 1,hADC_AR_CH1_Timer);

	
	if(hKEYscan_Timer == NULL)
	{
		hKEYscan_Timer = xTimerCreate("KEYscan_Timer", 		// Just a text name, not used by the kernel.
										(KEYscan_Timer_INTERVAL/portTICK_RATE_MS), // The timer period in ticks. 
										pdFALSE,						// The timers will auto-reload themselves when they expire.
										(void *)KEYscan_Timer_ID, 					// Assign each timer a unique id equal to its array index.
										(TimerCallbackFunction_t) KEYscan_fun);
	}
	
		DBG_BUFFER(MODULE_APP, LEVEL_INFO, " ***hKEYscan_Timer value = 0x%x \n", 1,hKEYscan_Timer);

	hHeartRateQueueHandle = xQueueCreate(MAX_HEARTRATE_TASK_COUNT, sizeof(uint8_t));	

	
	KEYscan_fun_cnt2 = 0;
	xTimerStart(hKEYscan_Timer, 0);	
		
	//uTxCnt = sprintf((char *)uTxBuf, "HR=%03.0f\n", _myHR);	
	uTxCnt = sprintf((char *)uTxBuf, "RTL\n\r");	// RTL8762 開機完成,sprintf 回傳 字元len
	UART_SendData(UART, uTxBuf, uTxCnt);
	key_cnt = 0;
	cnt500ms = 0;
	i=0;
	
	// PWR_KEY Locked warning
	while( GPIO_ReadInputDataBit(GPIO_PWR_KEY_PIN) == SET )
	{
		if(KEYscan_fun_cnt2 > 6){
		if(KEYscan_fun_cnt2%2 == 0)
			GPIO_WriteBit(GPIO_STATUS_LED_PIN,Bit_SET); // STATUS LED ON
		else 
			GPIO_WriteBit(GPIO_STATUS_LED_PIN,Bit_RESET); 
		}
	}
	
	GPIO_WriteBit(GPIO_STATUS_LED_PIN,Bit_SET); // STATUS LED ON
	
	// Turn On PWR_KEY INT
	GPIO_INTConfig(GPIO_PWR_KEY_PIN, ENABLE);		
	GPIO_MaskINTConfig(GPIO_PWR_KEY_PIN, DISABLE);

	KEYscan_fun_cnt = 1;
	NoSignalShutdownCnt = 0;
	
	DBG_BUFFER(MODULE_APP, LEVEL_INFO, " Ver. %d-%d b \n", 2,12,26);
	
	while(true)
	{
		if(xQueueReceive(hHeartRateQueueHandle, &Event, portMAX_DELAY) == pdPASS)
		{
			if(Event == EVENT_START_HEARTRATE_CALCULATE)
			{
				CalculateHeartRate();
			}

			if(Event == EVENT_ADC_CONVERT_BUF_FULL)
			{
				DBG_BUFFER(MODULE_APP, LEVEL_INFO, "** Into EVENT_ADC_CONVERT_BUF_FULL !  \n", 0);
				Get_AR_ADC();
			}

			if(Event == EVENT_SCAN_KEY_TIMER)
			{
				if((KEYscan_fun_cnt != 0)&&(cnt500ms%5 == 0)) {
					EnPICcmdBuf[0]='E';
					EnPICcmdBuf[1]='N';
					EnPICcmdBuf[2]=KEYscan_fun_cnt+'0';
					EnPICcmdBuf[3]='\n';
					EnPICcmdBuf[4]='\r';
					UART_SendData(UART, EnPICcmdBuf, 5);
					KEYscan_fun_cnt++;
					if(KEYscan_fun_cnt > 9) 
						DBG_BUFFER(MODULE_APP, LEVEL_INFO, "** PIC can NOT work \n", 0);
						
				}
				cnt500ms++;
		
				if( GPIO_ReadInputDataBit(GPIO_S4_TEST_KEY_PIN) == RESET ){				
					key_cnt++;
					DBG_BUFFER(MODULE_APP, LEVEL_INFO, " **S4_TEST_KEY = %d \n", 1,key_cnt);
				
					if(key_cnt%2 == 1) uTxCnt = sprintf((char *)uTxBuf, "STP\n\r");	// RTL8762 開機完成,sprintf 回傳 字元len
					else uTxCnt = sprintf((char *)uTxBuf, "EN1\n\r");	
					
					UART_SendData(UART, uTxBuf, uTxCnt);
					
				}
				
				if( GPIO_ReadInputDataBit(GPIO_PWR_KEY_PIN) == SET ){	
					if(PWRkey_timer_cnt < 100)
						PWRkey_timer_cnt++;
					DBG_BUFFER(MODULE_APP, LEVEL_INFO, " **PWR_KEY = %d \n", 1,PWRkey_timer_cnt);
					if (PWRkey_timer_cnt > PWR_KEY_OFF_TIME_SET)
					{
						GPIO_WriteBit(GPIO_STATUS_LED_PIN,Bit_RESET); 
						DBG_BUFFER(MODULE_APP, LEVEL_INFO, "** STATUS_LED off / PWR_CONTROL_PIN Off !!!\n", 0);
						GPIO_WriteBit(GPIO_PWR_CONTROL_PIN,Bit_RESET); // PWR_CONTROL_PIN Off						
					}
				}

			}

			if( Event == EVENT_RxEndFlag_SET ){
				
				//DBG_BUFFER(MODULE_APP, LEVEL_INFO, "** EVENT_RxEndFlag_SET \n", 0);

				/* rx end */
				if(RxEndFlag == 1)
				{
					//DBG_BUFFER(MODULE_APP, LEVEL_INFO, "RxCount = %d \n", 1,RxCount);
				
					if(RxCount == 15){
						for (i = 0; i < 6; ++i){
							uGetFromPICBuf[i] = RxBuffer[i+1];
						}
						uGetFromPICBuf[6] = '-';
						//uGetFromPICBuf[7] = '\n';
						//uGetFromPICBuf[8] = '\r';
						UART_SendData(UART, uGetFromPICBuf, 7);

						if(uGetFromPICBuf[0] == 'H'){
							switch (uGetFromPICBuf[1])
							{
								case '0' : 
								case '1' : 
									KEYscan_fun_cnt=0;
									uGetFromPICBuf[3] = uGetFromPICBuf[3]-'0';
									uGetFromPICBuf[4] = uGetFromPICBuf[4]-'0';
									uGetFromPICBuf[5] = uGetFromPICBuf[5]-'0';
									_myHR = uGetFromPICBuf[3] *100 + uGetFromPICBuf[4]*10 + uGetFromPICBuf[5];

									if(( _myHR > 40 ) && ( _myHR < 210 )){											
										NoSignalShutdownCnt=0;
										DBG_BUFFER(MODULE_APP, LEVEL_INFO, "H%d_myHR = %d \n", 2,(uGetFromPICBuf[1]-'0'),_myHR);
									}
									else 
									{										
										DBG_BUFFER(MODULE_APP, LEVEL_INFO, "-> HR out of Range %d", 1,NoSignalShutdownCnt);
										NoSignalShutdownCnt++;
										_myHR=0;
									}
					        			break; 
								case '2' :
									NoSignalShutdownCnt++;
									DBG_BUFFER(MODULE_APP, LEVEL_INFO, "H2 status %d", 1,NoSignalShutdownCnt);
									_myHR=0;
										break; 
								case '3' :
									
									NoSignalShutdownCnt=21;
									DBG_BUFFER(MODULE_APP, LEVEL_INFO, "H3 status-> No signal %d",1,NoSignalShutdownCnt);
									_myHR=0;
										break; 
								default:
										break;
														
							}
							if( NoSignalShutdownCnt > 20 ){
								DBG_BUFFER(MODULE_APP, LEVEL_INFO, "< No Signal Shutdown >",0);
								GPIO_WriteBit(GPIO_STATUS_LED_PIN,Bit_RESET); 
								DBG_BUFFER(MODULE_APP, LEVEL_INFO, "** STATUS_LED off / PWR_CONTROL_PIN Off !!!\n", 0);
								GPIO_WriteBit(GPIO_PWR_CONTROL_PIN,Bit_RESET); // PWR_CONTROL_PIN Off	
							}

						}
					}
					else if( RxCount == 7 ){
						if((RxBuffer[1] == 'E') && (RxBuffer[2] == 'N') && (RxBuffer[3] == 'B'))
							KEYscan_fun_cnt=0;
						}
					RxEndFlag = 0;
					RxCount = 0;
				}
			}
			if( Event == EVENT_PWR_KEY_PUSH_SET ){
				
				DBG_BUFFER(MODULE_APP, LEVEL_INFO, "** EVENT_PWR_KEY_PUSH_SET \n", 0);
				key_cnt++;
				PWRkey_timer_cnt = 0;
				DBG_BUFFER(MODULE_APP, LEVEL_INFO, " PWR_KEY INT cnt = %d / timer = %d \n", 2,key_cnt,PWRkey_timer_cnt);
			}
			if( Event == EVENT_PWR_KEY_RELEASE_SET ){
				
				DBG_BUFFER(MODULE_APP, LEVEL_INFO, " EVENT_PWR_KEY_RELEASE_SET \n", 0);
				
			}
		
			
			
		}
		
		
		
	}

}


void Timer2IntrHandler(void)
{

	TIM_ClearINT(TIM_ID);    
}



void Gpio16IntrHandler(void)
{
	// Disable Interrupt
	GPIO_ClearINTPendingBit(GPIO_GetPin(P2_0));
	GPIO_INTConfig(GPIO_GetPin(P2_0), DISABLE);
	GPIO_MaskINTConfig(GPIO_GetPin(P2_0), ENABLE);

	// Start Timer
//	xTimerStart(hPAH8001_Timer, 0);
}
