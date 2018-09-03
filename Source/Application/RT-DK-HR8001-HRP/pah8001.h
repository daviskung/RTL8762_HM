#ifndef __PAH8001_H__
#define __PAH8001_H__

#include "rtl876x.h"

//--------------------------------------------------------------------------
// I2C Addr
//--------------------------------------------------------------------------
#define PAH8001_ADR			0x57

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


#define HM_ADC_INTERVAL 		1000    // 40
#define HM_ADC_AR_TIMER_ID		3553

#define EVENT_START_HEARTRATE_CALCULATE 1
#define EVENT_START_AR_ADC				2
#define EVENT_ADC_CONVERT_BUF_FULL		3


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

void PAH8001_Write(uint8_t Write_Addr, uint8_t Write_Data);
void PAH8001_Read(uint8_t Read_Addr, uint8_t *Read_Data);
void PAH8001_Burst_Read(uint8_t Read_Addr, uint8_t *Read_Data, uint8_t Read_Size);

bool AR_ADC_CH1(void);
void Get_AR_ADC(void);



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
