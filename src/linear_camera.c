/*==================================================================================================
*    Copyright 2021-2024 NXP
*
*    NXP Confidential and Proprietary. This software is owned or controlled by NXP and may only be
*    used strictly in accordance with the applicable license terms. By expressly
*    accepting such terms or by downloading, installing, activating and/or otherwise
*    using the software, you are agreeing that you have read, and that you agree to
*    comply with and are bound by, such license terms. If you do not agree to be
*    bound by the applicable license terms, then you may not retain, install,
*    activate or otherwise use the software.
==================================================================================================*/

#ifdef __cplusplus
extern "C" {
#endif

/*==================================================================================================
*                                        INCLUDE FILES
* 1) system and project includes
* 2) needed interfaces from external units
* 3) internal and external interfaces from this unit
==================================================================================================*/
#include "linear_camera.h"
#include "Adc_Types.h"
#include "pixy2.h"
/*==================================================================================================
*                          LOCAL TYPEDEFS (STRUCTURES, UNIONS, ENUMS)
==================================================================================================*/

/*==================================================================================================
*                                       LOCAL MACROS
==================================================================================================*/

/*==================================================================================================
*                                      LOCAL CONSTANTS
==================================================================================================*/

/*==================================================================================================
*                                      LOCAL VARIABLES
==================================================================================================*/
static LinearCamera LinearCameraInstance;
static volatile uint8 CurrentAdcIndex = 0U;
static volatile uint16 CurrentPwmIndex = 0U;
/*==================================================================================================
*                                      GLOBAL CONSTANTS
==================================================================================================*/

/*==================================================================================================
*                                      GLOBAL VARIABLES
==================================================================================================*/
volatile uint8 CameraResultsBuffer[128];
static Adc_ValueGroupType AdcResultBuffer;
volatile uint8 AdcConvertedValues = 0;
volatile uint16 AdcMissedValues = 0;
volatile uint16 ClocksPerShutter = 0;
/*==================================================================================================
*                                   LOCAL FUNCTION PROTOTYPES
==================================================================================================*/

/*==================================================================================================
*                                       LOCAL FUNCTIONS
==================================================================================================*/
void NewCameraFrame(void){
    ClocksPerShutter = CurrentPwmIndex;
    AdcMissedValues += CurrentPwmIndex - AdcConvertedValues;
    AdcConvertedValues = CurrentAdcIndex;
    CurrentAdcIndex = 0U;
    CurrentPwmIndex = 0U;
    Pwm_SetDutyCycle(LinearCameraInstance.ClkPwmChannel, 0x4000);
    Dio_WriteChannel(LinearCameraInstance.ShutterDioChannel, (Dio_LevelType)STD_LOW);
    Pwm_EnableNotification(LinearCameraInstance.ClkPwmChannel, PWM_FALLING_EDGE);
}

void CameraClock(void){
    Adc_StartGroupConversion(LinearCameraInstance.InputAdcGroup);
    CurrentPwmIndex++;
}

void CameraAdcFinished(void){
    if(CurrentPwmIndex < 128U){
        CameraResultsBuffer[CurrentPwmIndex] = AdcResultBuffer*25U/64U;
        CurrentAdcIndex++;
    }
    else{
        Pwm_SetDutyCycle(LinearCameraInstance.ClkPwmChannel, 0U);
        Dio_WriteChannel(LinearCameraInstance.ShutterDioChannel, (Dio_LevelType)STD_HIGH);
        Gpt_StartTimer(LinearCameraInstance.ShutterGptChannel, 10U);
        Gpt_EnableNotification(LinearCameraInstance.ShutterGptChannel);
    }
}
/*==================================================================================================
*                                       GLOBAL FUNCTIONS
==================================================================================================*/

void LinearCameraInit(Pwm_ChannelType ClkPwmChannel, Gpt_ChannelType ShutterGptChannel, Adc_GroupType InputAdcGroup, Dio_ChannelType ShutterDioChannel){
    LinearCameraInstance.ClkPwmChannel = ClkPwmChannel;
    LinearCameraInstance.ShutterGptChannel = ShutterGptChannel;
    LinearCameraInstance.InputAdcGroup = InputAdcGroup;
    LinearCameraInstance.ShutterDioChannel = ShutterDioChannel;
    Dio_WriteChannel(LinearCameraInstance.ShutterDioChannel, (Dio_LevelType)STD_LOW);
    Adc_SetupResultBuffer(LinearCameraInstance.InputAdcGroup , &AdcResultBuffer);
    Adc_EnableGroupNotification(LinearCameraInstance.InputAdcGroup);
    Pwm_SetDutyCycle(LinearCameraInstance.ClkPwmChannel, 0x4000);
    Pwm_EnableNotification(LinearCameraInstance.ClkPwmChannel, PWM_FALLING_EDGE);
}

#ifdef __cplusplus
}
#endif

/** @} */
