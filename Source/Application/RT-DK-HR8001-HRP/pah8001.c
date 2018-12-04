#include <stdio.h>
#include "rtl876x.h"
#include "rtl876x_gpio.h"
#include "rtl876x_i2c.h"
#include "rtl876x_uart.h"
#include "rtl_delay.h"
#include "dlps_platform.h"
#include "SimpleBLEPeripheral.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "timers.h"

//#include "stk8ba50.h"
#include "pah8001.h"

#include "rtl876x_adc.h"
#include "rtl876x_lib_adc.h"

//--------------------------------------------------------------------------
// External data & function
//--------------------------------------------------------------------------

extern xQueueHandle hHeartRateQueueHandle;

extern bool is_heart_rate_service_notification_enabled;
extern bool is_heart_rate_monitor_notification_enabled;

extern bool HeartRateServiceValueNotify(void);
extern bool HeartRateMonitorValueNotify(void);

extern xTimerHandle hPAH8001_Timer;
extern xTimerHandle hADC_AR_CH1_Timer;
extern xTimerHandle hIR_PWM_Timer;
extern xTimerHandle hKEYscan_Timer;



extern uint16_t adcConvertRes_HM[ARY_SIZE];
extern uint8_t	adcConvertRes_HM_cnt;
extern uint8_t	HM_100ms_cnt;

UINT8 key_cnt;  // current setting


//--------------------------------------------------------------------------
// Global
//--------------------------------------------------------------------------
ppg_mems_data_t _ppg_mems_data, hr_ppg_mems_data;
ppg_mems_data_t __ppg_mems_data[FIFO_SIZE];



uint8_t _frame_Count;
uint8_t _led_step;
uint8_t _led_current_change_flag;
uint8_t _touch_flag;
uint8_t _state, _state_count;
uint8_t _write_index, _read_index;
uint8_t _time_stamp;
uint8_t _sleepflag;
uint8_t _cnt_to_update_heart_rate;
uint8_t update_cnt;
uint8_t ready_flag;

//uint16_t uTxCnt;
//uint8_t uTxBuf[128];
//float _myHR;

bool KEYscan_fun(void)
{
	uint8_t _hr_event;

	// Reset Timer 
	xTimerReset(hKEYscan_Timer, KEYscan_Timer_INTERVAL);
		_hr_event = EVENT_SCAN_KEY;

	// Send Task
	xQueueSend(hHeartRateQueueHandle, &_hr_event, 1);
		return TRUE;
}



/*--------------------------------------------------------------------------
// Function name: bool AR_ADC_CH0(void)
// Parameters: None
// Return: None
// Description:
//--------------------------------------------------------------------------*/
bool AR_ADC_CH1(void)
{
	uint8_t _hr_event;

	//	_time_stamp++;

	// Reset Timer 
	xTimerReset(hADC_AR_CH1_Timer, HM_ADC_INTERVAL);
		//DBG_BUFFER(MODULE_APP, LEVEL_INFO, "**  AR_ADC_CH1 timer ** ", 0);

	//enable ADC read and wait for ADC data ready interrupt
    	ADC_Cmd(ADC, ADC_One_Shot_Mode, DISABLE);   //must disable first
	    ADC_Cmd(ADC, ADC_One_Shot_Mode, ENABLE);

    /*wait for adc sample ready*/
    while (ADC_GetIntFlagStatus(ADC, ADC_INT_ONE_SHOT_DONE) != SET);
    ADC_ClearINTPendingBit(ADC, ADC_INT_ONE_SHOT_DONE);
    /*read sample result*/
    
	adcConvertRes_HM_cnt++;

	
	
	if( adcConvertRes_HM_cnt >= ARY_SIZE ) adcConvertRes_HM_cnt=0;
    adcConvertRes_HM[adcConvertRes_HM_cnt] = ADC_Read(ADC, ADC_CH1);

	//DBG_DIRECT("adcConvertRes_HM[%d] = %d \n",adcConvertRes_HM_cnt,adcConvertRes_HM[adcConvertRes_HM_cnt]);

	if( adcConvertRes_HM_cnt == (ARY_SIZE-1) )	_hr_event = EVENT_ADC_CONVERT_BUF_FULL;

	// Send Task
		xQueueSend(hHeartRateQueueHandle, &_hr_event, 1);
		return TRUE;
}



/*--------------------------------------------------------------------------
// Function name: bool Pixart_HRD(void)
// Parameters: None
// Return: None
// Description:
//--------------------------------------------------------------------------*/
bool Pixart_HRD(void)
{
	uint8_t _hr_event;
//	uint8_t _g_sensor_buf[6];
//	static int16_t _g_sensor_x, _g_sensor_y, _g_sensor_z;

	//	_time_stamp++;

		// Reset Timer 
		xTimerReset(hPAH8001_Timer, 2000);
		DBG_BUFFER(MODULE_APP, LEVEL_INFO, "**  HRD RTOS timer ** ", 0);
		_hr_event = EVENT_START_HEARTRATE_CALCULATE;

		// Send Task
		xQueueSend(hHeartRateQueueHandle, &_hr_event, 1);
		return TRUE;
	
#if 0	
	// Check Touch Status for power saving (Bank 0)
	PAH8001_Write(0x7F,0x00);

	PAH8001_Read(0x59, &_touch_flag);

	_touch_flag &= 0x80;

	if(_touch_flag != 0x80)
	{
		if(_sleepflag == 0)
		{
			EnterSleepMode();
		}

		// Stop Timer
		xTimerStop(hPAH8001_Timer, 0);

		// Enable Interrupt
		GPIO_ClearINTPendingBit(GPIO_GetPin(P2_0));
		GPIO_INTConfig(GPIO_GetPin(P2_0), ENABLE);
		GPIO_MaskINTConfig(GPIO_GetPin(P2_0), DISABLE);

		// Send Task
		xQueueSend(hHeartRateQueueHandle, &_hr_event, 1);

		return FALSE;
	}

	LED_Ctrl();

	// Change to Bank 1
	PAH8001_Write(0x7F,0x01);

	//check status: 0 is not ready, 1 is ready, 2 is loss one data 
	PAH8001_Read(0x68, &_ppg_mems_data.HRD_Data[0]);
	
	_ppg_mems_data.HRD_Data[0] &= 0x0F;

	if(_ppg_mems_data.HRD_Data[0] == 0)
	{
		return FALSE;
	}
	else
	{
		
		//Only support burst read (0x64~0x67), when using I2C interface
		PAH8001_Burst_Read(0x64, &_ppg_mems_data.HRD_Data[1], 4);

		//Only support burst read (0x1A~0x1C), when using I2C interface
		PAH8001_Burst_Read(0x1A, &_ppg_mems_data.HRD_Data[5], 3);

		_ppg_mems_data.HRD_Data[8] = _frame_Count++;

		/***********************HW counter explanation start***********************************/ 
		//HW counter is MCU background counter.  
		//These functions read heart rate data ready interval time then provide to lib. (Unit is ms) 
		//For example: polling time is 40ms 
		//Heart rate data ready interval time is 40ms or 80ms, because sensor ready is about 50ms. 
		//ppg_mems_data.HRD_Data[9]=40 or 80, first time provide 0 is OK. 
		/*********************** HW counter explanation end***********************************/

		// Fix Time Counter
		if(_time_stamp == 1)
			_ppg_mems_data.HRD_Data[9] = 40;
		else if(_time_stamp == 2)
			_ppg_mems_data.HRD_Data[9] = 80;
		else
			_ppg_mems_data.HRD_Data[9] = 0;

		// Clear Time Counter
		_time_stamp = 0;

		_ppg_mems_data.HRD_Data[10] = _led_current_change_flag;

		// bit7 is Touch Flag (bit7=1 is meant Touch, and bit7=0 is meant No Touch)
		//PAH8001_Write(0x7F,0x00);
		//PAH8001_Read(0x59, &ppg_mems_data.HRD_Data[11]);
		//ppg_mems_data.HRD_Data[11] &= 0x80;

		_ppg_mems_data.HRD_Data[11] = _touch_flag;

		_ppg_mems_data.HRD_Data[12] = _ppg_mems_data.HRD_Data[6];

		/***********************G sensor explanation start***********************************/
		// If no G sensor, please set ppg_mems_data.MEMS_Data[3] = {0};
		// Please set G sensor default range = 2G.
		// If G sensor output data format is not 16bit, please change to 16bit format.
		// For example: G sensor is 12bit
		// ppg_mems_data.MEMS_Data[0] = ReadGSensorX()<<4;
		// ppg_mems_data.MEMS_Data[1] = ReadGSensorY()<<4;
		// ppg_mems_data.MEMS_Data[2] = ReadGSensorZ()<<4;
		// MEMS_SensorX() absolute data must be largest when let hand down at static state.
		// MEMS_SensorZ() absolute data must be largest when let hand horizontal on the table at static state.
		/***********************G sensor explanation end***********************************/
		
		

		// Busrt read sensor address 0x02~0x07
		STK8BA50_Burst_Read(0x02, &_g_sensor_buf[0], 6);

		_g_sensor_x = (int16_t)((_g_sensor_buf[1]<<8)+_g_sensor_buf[0]);
		_g_sensor_y = (int16_t)((_g_sensor_buf[3]<<8)+_g_sensor_buf[2]);
		_g_sensor_z = (int16_t)((_g_sensor_buf[5]<<8)+_g_sensor_buf[4]);

		_ppg_mems_data.MEMS_Data[0] = _g_sensor_x;
		_ppg_mems_data.MEMS_Data[1] = _g_sensor_y;
		_ppg_mems_data.MEMS_Data[2] = _g_sensor_z;

		// Save data into FIFO
		Push(&_ppg_mems_data);


		// Send Task
		xQueueSend(hHeartRateQueueHandle, &_hr_event, 1);

		return TRUE;
	}
#endif	
}


/*--------------------------------------------------------------------------
// Function name: void LED_Ctrl(void)
// Parameters: None
// Return: None
// Description:
//--------------------------------------------------------------------------*/
void LED_Ctrl(void)
{
	uint8_t _ep_l, _ep_h;
	uint16_t _exposure_line;

	// Disable Sleep Mode (Bank 0)
	//PAH8001_Write(0x05,0x98);		

	if(_sleepflag == 1)
	{
		ExitSleepMode();
	}

	if(_state_count <= STATE_COUNT_TH)
	{
		_state_count++;

		_led_current_change_flag = 0;
	}
	else
	{
		_state_count = 0;

		// Read Exposure Line
		//PAH8001_Read(0x32, &_ep_l);
		//PAH8001_Read(0x33, &_ep_h);
		_exposure_line = (_ep_h<<8)+_ep_l;

		// Change to Bank 1
		//PAH8001_Write(0x7F,0x01);

		if(_state == 0)
		{
			if((_exposure_line >= LED_CTRL_EXPO_TIME_HI_BOUND)||(_exposure_line <= LED_CTRL_EXPO_TIME_LOW_BOUND))
			{
				// Read LED Step (Bank 1)
				//PAH8001_Read(0x38, &_led_step);

				_led_step &= 0x1F;

				if((_exposure_line >= LED_CTRL_EXPO_TIME_HI_BOUND)&&(_led_step < LED_CURRENT_HI))
				{
					_state = 1;

					_led_step = _led_step+LED_INC_DEC_STEP;

					if(_led_step > LED_CURRENT_HI)
						_led_step = LED_CURRENT_HI;

					//PAH8001_Write(0x38,(_led_step|0xE0));

					_led_current_change_flag = 1;
				}
				else if((_exposure_line <= LED_CTRL_EXPO_TIME_LOW_BOUND)&&(_led_step > LED_CURRENT_LOW))
				{
					_state = 2;

					if(_led_step <= (LED_CURRENT_LOW+LED_INC_DEC_STEP))
						_led_step = LED_CURRENT_LOW;
					else
						_led_step = _led_step-LED_INC_DEC_STEP;

					//PAH8001_Write(0x38,(_led_step|0xE0));

					_led_current_change_flag = 1;
				}
				else
				{
					_state = _led_current_change_flag = 0;
				} 
			} 
			else
			{
				_led_current_change_flag = 0;
			}
		}
		else if(_state == 1)
		{
			if(_exposure_line > LED_CTRL_EXPO_TIME_HI)
			{
				_state = 1;

				_led_step = _led_step+LED_INC_DEC_STEP;

				if(_led_step >= LED_CURRENT_HI)
				{
					_state = 0;
					_led_step = LED_CURRENT_HI;
				}

				// Change LED Step (Bank 1)
				//PAH8001_Write(0x38,(_led_step|0xE0));

				 _led_current_change_flag = 1;
			}
			else
			{
				_state = _led_current_change_flag = 0;
			}
		}
		else
		{
			if(_exposure_line < LED_CTRL_EXPO_TIME_LOW)
			{
				_state = 2 ;

				if(_led_step <= (LED_CURRENT_LOW+LED_INC_DEC_STEP))
				{
					_state = 0;
					_led_step = LED_CURRENT_LOW;
				}
				else
				{
					_led_step = _led_step-LED_INC_DEC_STEP;
				}

				// Change LED Step (Bank 1)
				//PAH8001_Write(0x38,(_led_step|0xE0));

				_led_current_change_flag = 1;
			}     
			else
			{
				_state = _led_current_change_flag = 0;
			}
		}
	}
}


/*--------------------------------------------------------------------------
// Function name: void EnterSleepMode(void)
// Parameters: None
// Return: None
// Description:
//--------------------------------------------------------------------------*/
void EnterSleepMode(void)
{
	// Enable Sleep Mode (Bank 0)
	//PAH8001_Write(0x7F,0x00);
	//PAH8001_Write(0x05,0xBC);

	// Change LED Step (Bank 1)
	//PAH8001_Write(0x7F,0x01);
	//PAH8001_Write(0x38,(DEFAULT_LED_STEP|0xE0));

	// Set Suspend Mode
	//STK8BA50_Write(REG_POWMODE, SUSPEND);

	// Reset Algorithm
	PxiAlg_Close();

	_led_step = DEFAULT_LED_STEP;

	_led_current_change_flag = 0;

	_frame_Count = 0;

	_time_stamp = 0;

	//_myHR = 0;

	_sleepflag = 1;
}


/*--------------------------------------------------------------------------
// Function name: void ExitSleepMode(void)
// Parameters: None
// Return: None
// Description:
//--------------------------------------------------------------------------*/
void ExitSleepMode(void)
{
	// Change LED Step (Bank 1)
	//PAH8001_Write(0x7F,0x01);
	//PAH8001_Write(0x38,(DEFAULT_LED_STEP|0xE0));

	// Set Normal Mode
	//STK8BA50_Write(REG_POWMODE, NORMAL_PWR);

	_cnt_to_update_heart_rate = 0;

	_write_index = _read_index = 0;

	_state_count = _state = 0;

	_sleepflag = 0;
}


/*--------------------------------------------------------------------------
// Function name: uint8_t AddCntToUpdate(void)
// Parameters: None
// Return: None
// Description:
//--------------------------------------------------------------------------*/
uint8_t AddCntToUpdate(void)
{
	return(_cnt_to_update_heart_rate++);	
}


/*--------------------------------------------------------------------------
// Function name: void ClearCntToUpdate(void)
// Parameters: None
// Return: None
// Description:
//--------------------------------------------------------------------------*/
void ClearCntToUpdate(void)
{
	_cnt_to_update_heart_rate = 0;	
}


/*--------------------------------------------------------------------------
// Function name: bool isFIFOEmpty(void)
// Parameters: None
// Return: None
// Description:
//--------------------------------------------------------------------------*/
bool isFIFOEmpty(void)
{
	return(_write_index == _read_index); 
}


/*--------------------------------------------------------------------------
// Function name: bool Push(ppg_mems_data_t *data)
// Parameters: None
// Return: None
// Description:
//--------------------------------------------------------------------------*/
bool Push(ppg_mems_data_t *data)
{
	uint8_t tmp;

	tmp = _write_index;

	tmp++;

	if(tmp >= FIFO_SIZE)
		tmp = 0;

	if(tmp == _read_index)
		return FALSE;

	__ppg_mems_data[tmp] = *data;

	_write_index = tmp;

	return TRUE;
}


/*--------------------------------------------------------------------------
// Function name: bool Pop(ppg_mems_data_t *data)
// Parameters: None
// Return: None
// Description:
//--------------------------------------------------------------------------*/
bool Pop(ppg_mems_data_t *data)
{
	uint8_t tmp;

	if(isFIFOEmpty())
		return FALSE;

	*data = __ppg_mems_data[_read_index];

	tmp = _read_index + 1;

	if(tmp >= FIFO_SIZE)
		tmp = 0;

	_read_index = tmp;

	return TRUE;
}

/*--------------------------------------------------------------------------
// Function name: void Get_AR_ADC(void)
// Parameters: None
// Return: None
// Description:
//--------------------------------------------------------------------------*/


void Get_AR_ADC(void)
{

	for(uint8_t i =0; i<ARY_SIZE;i++)
	{
	   DBG_DIRECT("adcConvertRes_HM[%d] = %d ",i,adcConvertRes_HM[i]);
	   if(i%10 == 9)
	   DBG_DIRECT("\n");
	}
	
   /* start adc convert again */
				  
	DBG_DIRECT(" ********************* \n");

 	return;
	
}


/*--------------------------------------------------------------------------
// Function name: void CalculateHeartRate(void)
// Parameters: None
// Return: None
// Description:
//--------------------------------------------------------------------------*/

//#define DEBUG_PRINT_HR_RAW_DATA

void CalculateHeartRate(void)
{
	DBG_BUFFER(MODULE_APP, LEVEL_INFO, "**  CalculateHeartRate ** ", 0);
	if(is_heart_rate_service_notification_enabled == TRUE)
	{
		
		HeartRateServiceValueNotify();
//		HeartRateMonitorValueNotify();

	}
	
	
	
#if 0	
	uint8_t ready_flag;
		
	if(!isFIFOEmpty())
	{
		if(Pop(&hr_ppg_mems_data))
		{
			#ifdef DEBUG_PRINT_HR_RAW_DATA
				uTxCnt = sprintf((char *)uTxBuf, "PPG_GSENSOR_RAW_DATA,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
				hr_ppg_mems_data.HRD_Data[0],hr_ppg_mems_data.HRD_Data[1],
				hr_ppg_mems_data.HRD_Data[2],hr_ppg_mems_data.HRD_Data[3],
				hr_ppg_mems_data.HRD_Data[4],hr_ppg_mems_data.HRD_Data[5],
				hr_ppg_mems_data.HRD_Data[6],hr_ppg_mems_data.HRD_Data[7],
				hr_ppg_mems_data.HRD_Data[8],hr_ppg_mems_data.HRD_Data[9],
				hr_ppg_mems_data.HRD_Data[10],hr_ppg_mems_data.HRD_Data[11],
				hr_ppg_mems_data.HRD_Data[12],
				(int16_t)hr_ppg_mems_data.MEMS_Data[0],
				(int16_t)hr_ppg_mems_data.MEMS_Data[1],
				(int16_t)hr_ppg_mems_data.MEMS_Data[2]);
				UART_SendData(UART, uTxBuf, uTxCnt);
			#else
				ready_flag = PxiAlg_Process(hr_ppg_mems_data.HRD_Data,hr_ppg_mems_data.MEMS_Data);

				if(ready_flag == FLAG_DATA_READY)
				{
					PxiAlg_HrGet(&_myHR);

					_cnt_to_update_heart_rate++;

					if(_cnt_to_update_heart_rate >= EVERY_N_PPG_SAMPLES_TO_UPDATE_CHART)
					{
						_cnt_to_update_heart_rate = 0;

						uTxCnt = sprintf((char *)uTxBuf, "HR=%03.0f\n", _myHR);

						UART_SendData(UART, uTxBuf, uTxCnt);

						if(is_heart_rate_service_notification_enabled == TRUE)
						{
							HeartRateServiceValueNotify();
						}
						else if(is_heart_rate_monitor_notification_enabled == TRUE)
						{
							HeartRateMonitorValueNotify();
						}
					}
				}
				else
				{
					#if 0
					if(ready_flag == FLAG_DATA_NOT_READY)
					{
						uTxCnt = sprintf((char *)uTxBuf, "HR=NRD\n");
						UART_SendData(UART, uTxBuf, uTxCnt);
					}
					else if(ready_flag == FLAG_DATA_LOSS)
					{
						uTxCnt = sprintf((char *)uTxBuf, "HR=LOS\n");
						UART_SendData(UART, uTxBuf, uTxCnt);
					}
					else if(ready_flag == FLAG_NO_TOUCH)
					{
						uTxCnt = sprintf((char *)uTxBuf, "HR=NTH\n");
						UART_SendData(UART, uTxBuf, uTxCnt);
					}
					else if(ready_flag == FLAG_DATA_ERROR)
					{
						uTxCnt = sprintf((char *)uTxBuf, "HR=ERR\n");
						UART_SendData(UART, uTxBuf, uTxCnt);
					}
					else if(ready_flag == FLAG_POOR_SIGNAL)
					{
						uTxCnt = sprintf((char *)uTxBuf, "HR=POR\n");
						UART_SendData(UART, uTxBuf, uTxCnt);
					}
					else if(ready_flag == FLAG_FIFO_ERROR)
					{
						uTxCnt = sprintf((char *)uTxBuf, "HR=FIO\n");
						UART_SendData(UART, uTxBuf, uTxCnt);
					}
					else if(ready_flag == FLAG_TIMING_ERROR)
					{
						uTxCnt = sprintf((char *)uTxBuf, "HR=TIM\n");
						UART_SendData(UART, uTxBuf, uTxCnt);
					}
					#endif
				}
			#endif
		}
	}	
#endif	
	
}



