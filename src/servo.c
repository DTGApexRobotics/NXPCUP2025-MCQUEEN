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
#include "servo.h"

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
static Servo ServoInstance;
static volatile uint16 ServoDutyCycle;
/*==================================================================================================
*                                      GLOBAL CONSTANTS
==================================================================================================*/

/*==================================================================================================
*                                      GLOBAL VARIABLES
==================================================================================================*/

/*==================================================================================================
*                                   LOCAL FUNCTION PROTOTYPES
==================================================================================================*/

/*==================================================================================================
*                                       LOCAL FUNCTIONS
==================================================================================================*/
void ServoPeriodFinished(void){
    Pwm_SetDutyCycle(ServoInstance.ServoPwmChannel, ServoDutyCycle);
}
/*==================================================================================================
*                                       GLOBAL FUNCTIONS
==================================================================================================*/

void ServoInit(Pwm_ChannelType ServoPwmChannel, uint16 MaxDutyCycle, uint16 MinDutyCycle, uint16 MedDutyCycle ){
    ServoInstance.ServoPwmChannel = ServoPwmChannel;
    ServoInstance.MaxDutyCycle = MaxDutyCycle;
    ServoInstance.MinDutyCycle = MinDutyCycle;
    ServoInstance.MedDutyCycle = MedDutyCycle;
    ServoDutyCycle = ServoInstance.MedDutyCycle;
    Pwm_SetDutyCycle(ServoInstance.ServoPwmChannel, ServoInstance.MedDutyCycle);
    Pwm_EnableNotification(ServoInstance.ServoPwmChannel, PWM_RISING_EDGE);/*enables interrupt for servo update at every PWM signal rising edge*/
}

void Steer(int Direction){
    if(Direction>(int)0){
        if(Direction >= 100){
            Direction = 100;
        }
        ServoDutyCycle = ServoInstance.MedDutyCycle + Direction*(int)(ServoInstance.MinDutyCycle-ServoInstance.MedDutyCycle)/100;
        //ServoDutyCycle = 2500U + (Direction*(int)(1700U-2500U))/100;
    }
    else{
        if(Direction <= -100){
            Direction = -100;
        }
        ServoDutyCycle = ServoInstance.MedDutyCycle - Direction*(int)(ServoInstance.MaxDutyCycle-ServoInstance.MedDutyCycle)/100;
        //ServoDutyCycle = 2500U - (Direction*(int)(3000U-2500U))/100;
    }
}

void SteerLeft(void){
    ServoDutyCycle = ServoInstance.MaxDutyCycle;
}

void SteerRight(void){
    ServoDutyCycle = ServoInstance.MinDutyCycle;
}

void SteerStraight(void){
    ServoDutyCycle = ServoInstance.MedDutyCycle;
}

#ifdef __cplusplus
}
#endif

/** @} */
