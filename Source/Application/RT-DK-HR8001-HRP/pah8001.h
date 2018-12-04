#ifndef __PAH8001_H__
#define __PAH8001_H__

#include "rtl876x.h"

//--------------------------------------------------------------------------
// Constant Define
//--------------------------------------------------------------------------
#define PXIALG_API

#define FIFO_SIZE			32

#define LED_INC_DEC_STEP	2
#define LED_CTRL_EXPO_TIME_HI_BOUND		496
#define LED_CTRL_EXPO_TIME_LOW_BOUND	32
#define LED_CTRL_EXPO_TIME_HI	420
#define LED_CTRL_EXPO_TIME_LOW	64
#define LED_CURRENT_HI		31
#define LED_CURRENT_LOW		1

#define STATE_COUNT_TH		3

#define DEFAULT_LED_STEP	4

#define EVERY_N_PPG_SAMPLES_TO_UPDATE_CHART	23

#define SENSOR_PAH8001_INTERVAL 2000    // 40
#define SENSOR_PAH8001_TIMER_ID 3552


#define HM_ADC_INTERVAL 		40    // 40
#define HM_ADC_AR_TIMER_ID		3553

#define KEYscan_Timer_INTERVAL 	500    // 40
#define KEYscan_Timer_ID		3554


#define EVENT_START_HEARTRATE_CALCULATE 1
#define EVENT_START_AR_ADC				2
#define EVENT_ADC_CONVERT_BUF_FULL		3
#define EVENT_SCAN_KEY					4
#define EVENT_RxEndFlag_SET				5
#define EVENT_KEY4_PUSH_SET				6
#define EVENT_KEY4_RELEASE_SET			7




#define ARY_SIZE		100

/* Defines ------------------------------------------------------------------*/

//#define LED1_Pin            P0_3	// LOG_UART_TX can not use
//#define GPIO_LED1_Pin       GPIO_GetPin(LED1_Pin)

#define LED2_Pin            P2_2	// AJ no work,ONLY AG & AK
#define GPIO_LED2_Pin       GPIO_GetPin(LED2_Pin)

#define LED3_Pin            P0_5
#define GPIO_LED3_Pin       GPIO_GetPin(LED3_Pin)


#define KEY1_Pin            P0_1
#define GPIO_KEY1_Pin       GPIO_GetPin(KEY1_Pin)
//#define GPIOTestIntrHandler Gpio1IntrHandler


#define KEY2_Pin            P0_2
#define GPIO_KEY2_Pin       GPIO_GetPin(KEY2_Pin)

#define KEY4_Pin            P3_3
#define GPIO_KEY4_Pin       GPIO_GetPin(KEY4_Pin)

#define DTAT_UART_TX_Pin            P4_0
#define DTAT_UART_RX_Pin            P4_1

#define HCI_UART_TX_Pin            P3_0
#define HCI_UART_RX_Pin            P3_1

#define PWR_CONTROL_Pin            P3_2
#define GPIO_PWR_CONTROL_Pin       GPIO_GetPin(PWR_CONTROL_Pin)



#if 0
/* Defines HM control pin ----------------------------*/
#define NSTROBE_R1_Pin            P1_2
#define GPIO_NSTROBE_R1_Pin       GPIO_GetPin(NSTROBE_R1_Pin)
#define NSTROBE_R2_Pin            P1_3
#define GPIO_NSTROBE_R2_Pin       GPIO_GetPin(NSTROBE_R2_Pin)
#define NSTROBE_R3_Pin            P0_6
#define GPIO_NSTROBE_R3_Pin       GPIO_GetPin(NSTROBE_R3_Pin)
#define NSTROBE_R4_Pin            P0_7
#define GPIO_NSTROBE_R4_Pin       GPIO_GetPin(NSTROBE_R4_Pin)

#define NDISCH_Pin            P3_2
#define GPIO_NDISCH_Pin       GPIO_GetPin(NDISCH_Pin)

#define SAMP_Pin              P4_0
#define GPIO_SAMP_Pin         GPIO_GetPin(SAMP_Pin)

#define GCS_Pin               P4_1
#define GPIO_GCS_Pin          GPIO_GetPin(GCS_Pin)

#define GUD_Pin               P4_2
#define GPIO_GUD_Pin          GPIO_GetPin(GUD_Pin)



#define NSTROBE_R1_control            GPIO_Pin_10
#define NSTROBE_R2_control            GPIO_Pin_11
#define NSTROBE_R3_control            GPIO_Pin_6
#define NSTROBE_R4_control            GPIO_Pin_7

#define NDISCH_control            GPIO_Pin_26
#define SAMP_control              GPIO_Pin_28
#define GCS_control               GPIO_Pin_29
#define GUD_control               GPIO_Pin_30
#endif

#define key1_control               GPIO_Pin_1
#define key2_control               GPIO_Pin_2
#define key4_control               GPIO_Pin_27


#define HM_ADC_AR               ADC_CH0
#define HM_ADC_AN0              ADC_CH1
#define HM_ADC_AN1              ADC_CH2


#define MAXGAIN 63
#define MINGAIN 0
#define MIDGAIN 32


/* TIM Defines ------------------------------------------------------------------*/
#define TIM_ID                  TIM2

#define NSTROBE_LOW_start		0
#define NSTROBE_LOW_end			1





//--------------------------------------------------------------------------
//  Structure
//--------------------------------------------------------------------------
typedef struct
{
	uint8_t HRD_Data[13];
	float MEMS_Data[3];
}ppg_mems_data_t;


//--------------------------------------------------------------------------
//  Enum
//--------------------------------------------------------------------------
typedef enum
{
	FLAG_DATA_READY = 0,
	FLAG_DATA_NOT_READY,
	FLAG_DATA_LOSS,
	FLAG_NO_TOUCH,
	FLAG_DATA_ERROR,
	FLAG_POOR_SIGNAL,
	FLAG_FIFO_ERROR,
	FLAG_TIMING_ERROR
}PXI_STATUS_FLAG;


//--------------------------------------------------------------------------
// Prototypes
//--------------------------------------------------------------------------
void PAH8001_Settings(void);
bool Pixart_HRD(void);
void LED_Ctrl(void);
void EnterSleepMode(void);
void ExitSleepMode(void);
uint8_t AddCntToUpdate(void);
void ClearCntToUpdate(void);
void PushHeart(float myHR);

bool isFIFOEmpty(void);
bool Push(ppg_mems_data_t *data);
bool Pop(ppg_mems_data_t *data);

void CalculateHeartRate(void);

bool AR_ADC_CH1(void);
void Get_AR_ADC(void);
bool KEYscan_fun(void);


void GainDelay (void);	// changing GUD or GCS requires a delay depending on digital pot chip
void GainStepUp (void);
void GainStepDown (void);
void InitGain(UINT8 val);
void GainUp (void);
void GainDown (void);






//--------------------------------------------------------------------------
// Library API
//--------------------------------------------------------------------------
/**
 * @brief When HRD and MEMS data are ready, call this function to do the algorithm processing
 *
 * @param[in] HRD_Data   Pointer to the buffer where HRD data (13 Bytes) is stored.
 * @param[in] MEMS_Data  Pointer to the buffer where MEMS data (3*sizeof(float) Bytes) is stored.
 *
 * @return Return one of the PXI_STATUS_FLAG types.
 */
PXIALG_API int32_t PxiAlg_Process(unsigned char *HRD_Data, float *MEMS_Data);

/**
 * @brief Call this function to get Heart Rate
 *
 * @param[out] hr   Pointer to a float variable where heart rate is stored
 *
 * @return None
 */
PXIALG_API void PxiAlg_HrGet(float *hr);

/**
 * @brief Call this function to determine the version of the algorithm
 *
 *
 * @return Version of the algorithm
 */
PXIALG_API int32_t PxiAlg_Version(void);

/**
 * @brief Call this funtion to get Ready_Flag
 *
 * @return Return Ready_Flag
 */
PXIALG_API unsigned char PxiAlg_GetReadyFlag(void);

/**
 * @brief Call this funtion to get Motion_Flag
 *
 * @return Return Motion_Flag
 */
PXIALG_API unsigned char PxiAlg_GetMotionFlag(void);

/**
 * @brief Call this function to notify algorithm the MEMS Scale of Motion Sensor
 *
 * @param[in] scale The MEMS Scale of Motion Sensor. Only 0(2G),1(4G~16G) are supported.
 *
 * @return 1 for success. 0 for failure.
 */
PXIALG_API uint8_t PxiAlg_SetMemsScale(int scale);

/**
 * @brief Call this function to get PPG Signal Grade
 *
 * @param[out] grade	Pointer to a float variable where signal grade is stored.
 *
 * @return  Return 1 when Signal Grade is ready. Otherwise, return 0.
 */
PXIALG_API uint8_t PxiAlg_GetSigGrade(float *grade);

/**
 * @brief Call this function to set PPG Signal Grade Threshold
 *
 * @param[in] threshold	The PPG Signal Grade Threshold. Its value ranges from 0 to 100.
 *
 * @return 1 for success. 0 for failure.
 */
PXIALG_API uint8_t PxiAlg_SetSigGradeThrd(float thrd);

/**
 * @brief Call this function to enable or disable fast output mode
 *
 * @param[in] en The flag of fast output mode.
 */
PXIALG_API void PxiAlg_EnableFastOutput(uint8_t en);

/**
 * @brief Call this function to enable or disable motion mode
 *
 * @param[in] en The flag of motion mode.
 */
PXIALG_API void PxiAlg_EnableMotionMode(uint8_t en);

/**
 * @brief Call this function to enable or disable auto mode
 * @param[in] enable The flag of auto mode.
 */
PXIALG_API void PxiAlg_EnableAutoMode(uint8_t en);

/**
 * @brief Call this function to open algorithm
 */
PXIALG_API void PxiAlg_Open(void);

/**
 * @brief Call this function to close/reset algorithm
 */
PXIALG_API void PxiAlg_Close(void);

/**
 * @brief Call this function to get the alarm flag of fast output
* @param[out] get the alarm flag of fast output
 * @return  Return 1 when the flag is set. Otherwise, return 0.
 */
PXIALG_API uint8_t PxiAlg_GetFastOutAlarmFlag(void);

/**
 * @brief Call this function to enable or disable mems0 signal grade mode
 * @param[in] 1 : enable. 0 : disable.
 */
PXIALG_API void PxiAlg_EnableMEMS0SigGrade(uint8_t en);

/**
 * @brief Call this function to get the alarm flag of signal bad
 * @param[out] get the alarm flag of bad signal
 * @return  Return 1 when the flag is set. Otherwise, return 0.
 */
PXIALG_API uint8_t PxiAlg_GetSignalBadAlarm(void);

/**  * @brief Call this function to set PPG buffer pointer and size
 * @param[in] ppg_buffer   Pointer to a float buffer where the PPG data would be stored
 * @param[in] size   PPG buffer size  
 * @return None  */ 
PXIALG_API void PxiAlg_SetPPGBuffer(float *ppg_buffer, int32_t size);

#endif
