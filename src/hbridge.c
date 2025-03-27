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
#include "hbridge.h"
#include "Pwm.h"
#include "Dio.h"
/*==================================================================================================
*                          LOCAL TYPEDEFS (STRUCTURES, UNIONS, ENUMS)
==================================================================================================*/

/*==================================================================================================
*                                       LOCAL MACROS
==================================================================================================*/

/*==================================================================================================
*                                      LOCAL CONSTANTS
==================================================================================================*/
static Hbridge HbridgeInstance;
/*==================================================================================================
*                                      LOCAL VARIABLES
==================================================================================================*/

/*==================================================================================================
*                                      GLOBAL CONSTANTS
==================================================================================================*/

/*==================================================================================================
*                                      GLOBAL VARIABLES
==================================================================================================*/
/*hbridge variables*/
static uint8 Handbrake = 0U; /*value is zero or non zero: val == 0 means brake, val != 0 means no brake*/
static int CarSpeed = 0;    /*values between -100 and 100: val >= 0 means Forward, val < 0 means Reverse*/
static uint16 SpeedDutyCycle;

static int CarSpeedMotor1 = 0;     /* Valori da -100 a 100 per il motore 1 */
static int CarSpeedMotor2 = 0;     /* Valori da -100 a 100 per il motore 2 */
static uint16 SpeedDutyCycle1;     /* Duty cycle per il motore 1 */
static uint16 SpeedDutyCycle2;     /* Duty cycle per il motore 2 */

/*==================================================================================================
*                                   LOCAL FUNCTION PROTOTYPES
==================================================================================================*/

/*==================================================================================================
*                                       LOCAL FUNCTIONS
==================================================================================================*/
void Hbridge_Period_Finished(void){
    Pwm_SetDutyCycle(HbridgeInstance.Motor1_Speed, SpeedDutyCycle);
    Pwm_SetDutyCycle(HbridgeInstance.Motor2_Speed, SpeedDutyCycle);
}

void Hbridge_Period_Finished_1(void){
    Pwm_SetDutyCycle(HbridgeInstance.Motor1_Speed, SpeedDutyCycle1);
    Pwm_SetDutyCycle(HbridgeInstance.Motor2_Speed, SpeedDutyCycle2);
}
/*==================================================================================================
*                                       GLOBAL FUNCTIONS
==================================================================================================*/

void HbridgeInit(Pwm_ChannelType Motor1_Speed, Pwm_ChannelType Motor2_Speed, Dio_ChannelType Motor1_Forward,
    Dio_ChannelType Motor1_Backward, Dio_ChannelType Motor2_Forward, Dio_ChannelType Motor2_Backward){
    HbridgeInstance.Motor1_Speed = Motor1_Speed;
    HbridgeInstance.Motor2_Speed = Motor2_Speed;
    HbridgeInstance.Motor1_Forward = Motor1_Forward;
    HbridgeInstance.Motor1_Backward = Motor1_Backward;
    HbridgeInstance.Motor2_Forward = Motor2_Forward;
    HbridgeInstance.Motor2_Backward = Motor2_Backward;
}

void HbridgeInit_1(Pwm_ChannelType Motor1_Speed, Pwm_ChannelType Motor2_Speed,
    Dio_ChannelType Motor1_Forward, Dio_ChannelType Motor1_Backward,
    Dio_ChannelType Motor2_Forward, Dio_ChannelType Motor2_Backward){
    HbridgeInstance.Motor1_Speed    = Motor1_Speed;
    HbridgeInstance.Motor2_Speed    = Motor2_Speed;
    HbridgeInstance.Motor1_Forward  = Motor1_Forward;
    HbridgeInstance.Motor1_Backward = Motor1_Backward;
    HbridgeInstance.Motor2_Forward  = Motor2_Forward;
    HbridgeInstance.Motor2_Backward = Motor2_Backward;
}
void HbridgeSetSpeed(int Speed){

    if(Handbrake == 0U){
        if(Speed > 100){
            Speed = 100;
        }
        else if(Speed < -100){
            Speed = -100;
        }
        if(Speed >= 0){
            SpeedDutyCycle = (uint32)Speed*8192UL/25UL;
            Dio_WriteChannel(HbridgeInstance.Motor1_Forward, (Dio_LevelType)STD_HIGH);
            Dio_WriteChannel(HbridgeInstance.Motor1_Backward, (Dio_LevelType)STD_LOW);
            Dio_WriteChannel(HbridgeInstance.Motor2_Forward, (Dio_LevelType)STD_HIGH);
            Dio_WriteChannel(HbridgeInstance.Motor2_Backward, (Dio_LevelType)STD_LOW);
        }
        else if(Speed < 0){
            SpeedDutyCycle = (uint32)(Speed*-1)*8192UL/25UL;
            Dio_WriteChannel(HbridgeInstance.Motor1_Forward, (Dio_LevelType)STD_LOW);
            Dio_WriteChannel(HbridgeInstance.Motor1_Backward, (Dio_LevelType)STD_HIGH);
            Dio_WriteChannel(HbridgeInstance.Motor2_Forward, (Dio_LevelType)STD_LOW);
            Dio_WriteChannel(HbridgeInstance.Motor2_Backward, (Dio_LevelType)STD_HIGH);
        }
        if(CarSpeed == 0 && Speed != 0){
            Pwm_SetDutyCycle(HbridgeInstance.Motor1_Speed, SpeedDutyCycle);
            Pwm_SetDutyCycle(HbridgeInstance.Motor2_Speed, SpeedDutyCycle);
            Pwm_EnableNotification(HbridgeInstance.Motor1_Speed, PWM_RISING_EDGE);/*enables interrupt for hbridge update at every PWM signal rising edge*/
        }
    }
    CarSpeed = Speed;
}

void HbridgeSetSpeedIndependent(int SpeedMotor1, int SpeedMotor2) {
    if(Handbrake == 0U){
        /* Limitiamo i valori di velocità */
        if(SpeedMotor1 > 100){
            SpeedMotor1 = 100;
        } else if(SpeedMotor1 < -100){
            SpeedMotor1 = -100;
        }
        if(SpeedMotor2 > 100){
            SpeedMotor2 = 100;
        } else if(SpeedMotor2 < -100){
            SpeedMotor2 = -100;
        }

        /* Gestione del motore 1 */
        if(SpeedMotor1 >= 0){
            SpeedDutyCycle1 = (uint32)SpeedMotor1 * 8192UL / 25UL;
            Dio_WriteChannel(HbridgeInstance.Motor1_Forward, (Dio_LevelType)STD_HIGH);
            Dio_WriteChannel(HbridgeInstance.Motor1_Backward, (Dio_LevelType)STD_LOW);
        } else {
            SpeedDutyCycle1 = (uint32)(-SpeedMotor1) * 8192UL / 25UL;
            Dio_WriteChannel(HbridgeInstance.Motor1_Forward, (Dio_LevelType)STD_LOW);
            Dio_WriteChannel(HbridgeInstance.Motor1_Backward, (Dio_LevelType)STD_HIGH);
        }

        /* Gestione del motore 2 */
        if(SpeedMotor2 >= 0){
            SpeedDutyCycle2 = (uint32)SpeedMotor2 * 8192UL / 25UL;
            Dio_WriteChannel(HbridgeInstance.Motor2_Forward, (Dio_LevelType)STD_HIGH);
            Dio_WriteChannel(HbridgeInstance.Motor2_Backward, (Dio_LevelType)STD_LOW);
        } else {
            SpeedDutyCycle2 = (uint32)(-SpeedMotor2) * 8192UL / 25UL;
            Dio_WriteChannel(HbridgeInstance.Motor2_Forward, (Dio_LevelType)STD_LOW);
            Dio_WriteChannel(HbridgeInstance.Motor2_Backward, (Dio_LevelType)STD_HIGH);
        }

        /* Aggiorna i duty cycle dei due canali PWM */
        Pwm_SetDutyCycle(HbridgeInstance.Motor1_Speed, SpeedDutyCycle1);
        Pwm_SetDutyCycle(HbridgeInstance.Motor2_Speed, SpeedDutyCycle2);

        /* Abilita le notifiche PWM per entrambi i motori se necessario */
        Pwm_EnableNotification(HbridgeInstance.Motor1_Speed, PWM_RISING_EDGE);
        Pwm_EnableNotification(HbridgeInstance.Motor2_Speed, PWM_RISING_EDGE);
    }

    /* Aggiorna le velocità correnti memorizzate */
    CarSpeedMotor1 = SpeedMotor1;
    CarSpeedMotor2 = SpeedMotor2;
}

int HbridgeGetSpeed(){
    return CarSpeed;
}

int HbridgeGetSpeedMotor1(){
    return CarSpeedMotor1;
}

int HbridgeGetSpeedMotor2(){
    return CarSpeedMotor2;
}

void HbridgeSetBrake(uint8 Brake){
    if(Brake != 0U){
        Handbrake = 1U;
        SpeedDutyCycle = 0;
    }
    else{
        Handbrake = 0U;
        SpeedDutyCycle = (uint16)((uint32)(CarSpeed*-1)*8192UL/25UL);
    }
}

void HbridgeSetBrake_1(uint8 Brake){
    if(Brake != 0U){
        Handbrake = 1U;
        SpeedDutyCycle1 = 0;
        SpeedDutyCycle2 = 0;
    }
    else{
        Handbrake = 0U;
        /* Ricalcola i duty cycle in base alle velocità correnti */
        if(CarSpeedMotor1 < 0){
            SpeedDutyCycle1 = (uint32)(-CarSpeedMotor1) * 8192UL / 25UL;
        } else {
            SpeedDutyCycle1 = (uint32)CarSpeedMotor1 * 8192UL / 25UL;
        }
        if(CarSpeedMotor2 < 0){
            SpeedDutyCycle2 = (uint32)(-CarSpeedMotor2) * 8192UL / 25UL;
        } else {
            SpeedDutyCycle2 = (uint32)CarSpeedMotor2 * 8192UL / 25UL;
        }
    }
}

uint8 HbridgeGetBrake(){
    return Handbrake;
}

#ifdef __cplusplus
}
#endif

/** @} */


