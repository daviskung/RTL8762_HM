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



extern void AppHandleIODriverMessage(BEE_IO_MSG io_driver_msg_recv);
extern void Driver_Init(void);

extern uint8_t _touch_flag;


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


xTaskHandle  hOTAAppTaskHandle;
xTaskHandle  hHeartRateAppTaskHandle;

xQueueHandle hEventQueueHandle;
xQueueHandle hMessageQueueHandle;
xQueueHandle hIoQueueHandle;
xQueueHandle hHeartRateQueueHandle = NULL;
xTimerHandle hPAH8001_Timer = NULL;

xTimerHandle hADC_AR_CH1_Timer = NULL;

// HM ckt initial --- start


void NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStruct;
    
    /* ADC IRQ */  
    NVIC_InitStruct.NVIC_IRQChannel = ADC_IRQ;
    NVIC_InitStruct.NVIC_IRQChannelPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    
    NVIC_Init(&NVIC_InitStruct);

    return;
}


// HM ckt initial --- End


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

#if 0	
	// Initial STK8BA50
	STK8BA50_Settings();

	// Initial PAH8001
	PAH8001_Settings();

	/***********************Algorithm default setting start********************/
	//Algorithm default is 2G scale, disable fast output and Auto mode.
	//If users want to use default Algorithm setting, don't write these API functions. 

	// 2G scale
	PxiAlg_SetMemsScale(0);

	// FastOut Disable
	PxiAlg_EnableFastOutput(0);

	// Auto mode
	PxiAlg_EnableAutoMode(1);
	PxiAlg_EnableMotionMode(0);

	// Sleep Mode
	EnterSleepMode();

	/***********************Algorithm default setting end**********************/
#endif	

	// Initial HM ckt
	
	/* NVIC configuration */
    //NVIC_Configuration();

    /******************** ADC init ******************************/
	// BatteryMonitor_Init() setup 
#if 0
	RCC_PeriphClockCmd(APBPeriph_ADC, APBPeriph_ADC_CLOCK, ENABLE);
	
    ADC_InitTypeDef adcInitStruct;
    
    ADC_StructInit(&adcInitStruct);
    
    /* use channel 1 */
    adcInitStruct.channelMap = ADC_CH1;
    ADC_Init(ADC, &adcInitStruct);
    ADC_INTConfig(ADC, ADC_INT_ONE_SHOT_DONE, ENABLE);
    ADC_Cmd(ADC, ADC_One_Shot_Mode, ENABLE);
#endif

	if(hPAH8001_Timer == NULL)
	{
		hPAH8001_Timer = xTimerCreate(	"SENSOR_PAH8001_TIMER",			// Just a text name, not used by the kernel.
										(SENSOR_PAH8001_INTERVAL/portTICK_RATE_MS),	// The timer period in ticks. 
										pdFALSE,						// The timers will auto-reload themselves when they expire.
										(void *)SENSOR_PAH8001_TIMER_ID,						// Assign each timer a unique id equal to its array index.
										(TimerCallbackFunction_t) Pixart_HRD);
	}

	DBG_BUFFER(MODULE_APP, LEVEL_INFO, " **hADC_AR_CH1_Timer value = 0x%x \n", 1,hADC_AR_CH1_Timer);

	if(hADC_AR_CH1_Timer == NULL)
	{
		hADC_AR_CH1_Timer = xTimerCreate("HM_ADC_AR_TIMER",			// Just a text name, not used by the kernel.
										(HM_ADC_INTERVAL/portTICK_RATE_MS),	// The timer period in ticks. 
										pdFALSE,						// The timers will auto-reload themselves when they expire.
										(void *)HM_ADC_AR_TIMER_ID,						// Assign each timer a unique id equal to its array index.
										(TimerCallbackFunction_t) AR_ADC_CH1);
	}

	DBG_BUFFER(MODULE_APP, LEVEL_INFO, " **hADC_AR_CH1_Timer value = 0x%x \n", 1,hADC_AR_CH1_Timer);


	hHeartRateQueueHandle = xQueueCreate(MAX_HEARTRATE_TASK_COUNT, sizeof(uint8_t));	
	
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
				DBG_BUFFER(MODULE_APP, LEVEL_INFO, " Into EVENT_ADC_CONVERT_BUF_FULL !  \n", 0);
				Get_AR_ADC();
			}
			
		}
	}
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
