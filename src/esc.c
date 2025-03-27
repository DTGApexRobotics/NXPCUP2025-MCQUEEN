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
#include "esc.h"

/*==================================================================================================
*                          LOCAL TYPEDEFS (STRUCTURES, UNIONS, ENUMS)
==================================================================================================*/

/*==================================================================================================
*                                       LOCAL MACROS
==================================================================================================*/

/*==================================================================================================
*                                      LOCAL CONSTANTS
==================================================================================================*/
static Esc EscInstance;
/*==================================================================================================
*                                      LOCAL VARIABLES
==================================================================================================*/

/*==================================================================================================
*                                      GLOBAL CONSTANTS
==================================================================================================*/

/*==================================================================================================
*                                      GLOBAL VARIABLES
==================================================================================================*/
/*esc variables*/
volatile enum EscStates EscState = Forward;
static volatile uint8 Handbrake = 0U; /*value is zero or non zero: val == 0 means brake, val != 0 means no brake*/
static volatile int CarSpeed = 0;    /*values between -100 and 100: val >= 0 means Forward, val < 0 means Reverse*/

/*==================================================================================================
*                                   LOCAL FUNCTION PROTOTYPES
==================================================================================================*/

/*==================================================================================================
*                                       LOCAL FUNCTIONS
==================================================================================================*/
void Pwm_Period_Finished(void){
    int EscSpeedCommand;
    /*if your chosen ESC has braking capabilities, set this on STD_ON. If not, set it on STD_OFF.
     * Using the wrong configuration can make the car go full speed backward on braking!*/
#if (ESC_HAS_BRAKE == STD_ON)
    /* state machine updates here at every PWM signal edge*/
    switch(EscState){
    case Forward:
        if(Handbrake != 0U || CarSpeed < 0){
            EscState = Braking;
        }
        break;
    case Braking:
        if(Handbrake == 0U && CarSpeed >= 0){
            EscState = Forward;
        }
        else if(Handbrake == 0U && CarSpeed < 0){
            EscState = Neutral;
        }
        break;
#else
    switch(EscState){
        case Forward:
            if(Handbrake != 0U){
                EscState = Braking;
            }
            else if(CarSpeed < 0){
                EscState = Reverse;
            }
            break;
        case Braking:
            if(Handbrake != 0U){
                EscState = Neutral;
            }
            else if(CarSpeed < 0){
                EscState = Reverse;
            }
            else if(CarSpeed >= 0){
                EscState = Forward;
            }
            break;
#endif
        case Neutral:
            if(Handbrake == 0U && CarSpeed < 0){
                EscState = Reverse;
            }
            else if(Handbrake == 0U && CarSpeed >= 0){
                EscState = Forward;
            }
            break;
        case Reverse:
            if(Handbrake != 0U){
                EscState = Braking;
            }
            else if(CarSpeed >= 0){
                EscState = Forward;
            }
            break;
        default:/*invalid case, set safe values for esc*/
            CarSpeed = 0;
    }

    /*update the car's speed*/
    switch(EscState){
        case Forward:
        case Reverse:
            EscSpeedCommand = CarSpeed;
            break;
        case Braking:
#if (ESC_HAS_BRAKE == STD_ON)
            EscSpeedCommand = -100;
#else
            EscSpeedCommand = -CarSpeed;
#endif
            break;
        case Neutral:
        default:
            EscSpeedCommand = 0;
    }
    SetPwm(EscSpeedCommand);
}
/*==================================================================================================
*                                       GLOBAL FUNCTIONS
==================================================================================================*/

void EscInit(Pwm_ChannelType EscPwmChannel, uint16 MinDutyCycle, uint16 MedDutyCycle, uint16 MaxDutyCycle){
    EscInstance.Channel = EscPwmChannel;
    EscInstance.MinDutyCycle = MinDutyCycle;/*1ms*/
    EscInstance.MedDutyCycle = MedDutyCycle;/*1.5ms*/
    EscInstance.MaxDutyCycle = MaxDutyCycle;/*2ms*/
    Pwm_SetDutyCycle(EscInstance.Channel, EscInstance.MedDutyCycle);/*sending the 'Neutral' command arms the esc*/
    Pwm_EnableNotification(EscInstance.Channel, PWM_FALLING_EDGE);/*enables interrupt for state machine update at every PWM signal falling edge*/
}

void SetPwm(int EscSpeedCommand) {
    uint16 EscDutyCycle;
    if(EscSpeedCommand >= 0) {
        if(EscSpeedCommand > 100)
            EscSpeedCommand = 100;
        EscDutyCycle = (uint16)(EscInstance.MedDutyCycle + EscSpeedCommand*(int)(EscInstance.MaxDutyCycle-EscInstance.MedDutyCycle)/100);
        Pwm_SetDutyCycle(EscInstance.Channel, EscDutyCycle);
    } else {
        if(EscSpeedCommand < -100)
            EscSpeedCommand = -100;
        EscDutyCycle = (uint16)(EscInstance.MedDutyCycle + EscSpeedCommand*(int)(EscInstance.MedDutyCycle-EscInstance.MinDutyCycle)/100);
        Pwm_SetDutyCycle(EscInstance.Channel, EscDutyCycle);
    }
}

void SetSpeed(int Speed){
    CarSpeed = Speed;
}

int GetSpeed(){
    return CarSpeed;
}

void SetBrake(uint8 Brake){
    if(Brake != 0U){
        Handbrake = 1U;
    }
    else{
        Handbrake = 0U;
    }
}

uint8 GetBrake(){
    return Handbrake;
}

enum EscStates GetEscState(void){
    return EscState;
}

#ifdef __cplusplus
}
#endif

/** @} */

