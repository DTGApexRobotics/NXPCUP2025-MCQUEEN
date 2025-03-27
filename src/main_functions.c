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
#include "main_functions.h"
#include "Mcu.h"
#include "Mcl.h"
#include "Platform.h"
#include "Port.h"
#include "CDD_I2c.h"
#include "Pwm.h"
#include "Icu.h"
#include "Gpt.h"
#include "Adc.h"
#include "Mcal.h"

#include "display.h"
#include "receiver.h"
#include "servo.h"
#include "pixy2.h"
#include "hbridge.h"
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

/*==================================================================================================
 *                                      GLOBAL CONSTANTS
==================================================================================================*/

/*==================================================================================================
 *                                      GLOBAL VARIABLES
==================================================================================================*/
volatile uint8 I2cReturnCode = 0U;
volatile uint32 I2cTotalErrors = 0U;
extern uint8 CameraResultsBuffer[128U];
extern uint16 AdcMissedValues;
/*==================================================================================================
 *                                   LOCAL FUNCTION PROTOTYPES
==================================================================================================*/

/*==================================================================================================
 *                                       LOCAL FUNCTIONS
==================================================================================================*/
void i2ccallback (uint8 event, uint8 channel){
    I2cReturnCode = event;
}

void I2c_ErrorCallback(uint8 event, uint8 channel){
    I2cTotalErrors++;
    I2cReturnCode = event;
}
/*==================================================================================================
 *                                       GLOBAL FUNCTIONS
==================================================================================================*/
void DriversInit(void){
    uint8 Index;
    /* Init system clock */
#if (MCU_PRECOMPILE_SUPPORT == STD_ON)
    Mcu_Init(NULL_PTR);
#elif (MCU_PRECOMPILE_SUPPORT == STD_OFF)
    Mcu_Init(&Mcu_Config_VS_0);
#endif

    /* Initialize the clock tree and apply PLL as system clock */
    Mcu_InitClock(McuClockSettingConfig_0);
    #if (MCU_NO_PLL == STD_OFF)
    while (MCU_PLL_LOCKED != Mcu_GetPllStatus())
        {
            /* Busy wait until the System PLL is locked */
        }
    Mcu_DistributePllClock();
    #endif
    Mcu_SetMode(McuModeSettingConf_0);

    /* Initialize Platform driver */
    Platform_Init(NULL_PTR);

    /* Initialize Port driver */
    Port_Init(NULL_PTR);

    /* Initialize Mcl */
    Mcl_Init(NULL_PTR);

    /* Init i2c instances */
    I2c_Init(NULL_PTR);

    /* Initialize the Icu driver */
    Icu_Init(NULL_PTR);

    /*Init gpt driver*/
    Gpt_Init(NULL_PTR);

    /*Init adc driver*/
    Adc_Init(NULL_PTR);
    Adc_CalibrationStatusType CalibStatus;
    for(Index = 0; Index <= 5; Index++)
    {
        Adc_Calibrate(0U, &CalibStatus);
        if(CalibStatus.AdcUnitSelfTestStatus == E_OK)
        {
            break;
        }
    }

    /*Init pwm driver*/
    Pwm_Init(NULL_PTR);
}

/*Vector NormalizePixyVector(Vector PixyVector){
    Vector NormalizedVector;
    NormalizedVector.VectorIndex = PixyVector.VectorIndex;
    NormalizedVector.x0 = PixyVector.x0 * 100U / 78U;
    NormalizedVector.y0 = PixyVector.y0 * 100U / 51U;
    NormalizedVector.x1 = PixyVector.x1 * 100U / 78U;
    NormalizedVector.y1 = PixyVector.y1 * 100U / 51U;
    return NormalizedVector;
}*/

void Pixy2Test(){
    DetectedVectors PixyVectors;
    volatile uint16 Delay = 10000;
    PixySetLed(0U,255U,0U);
    while(1){
        DisplayClear();
        PixyVectors = PixyGetVectors();
        for(uint8 Index = 0U; Index < PixyVectors.NumberOfVectors; Index++){
            DisplayVector(NormalizePixyVector(PixyVectors.Vectors[Index]));
        }
        DisplayRefresh();
        Delay=10000U;
        while(Delay){
            Delay--;
        }
    }
}

void DisplayTest(){
    uint16 DisplayValueTest = 0UL;
    uint8 DisplayGraphTest[128];
    for(uint8 i=0; i<128U;i++){
        DisplayGraphTest[i] = i/2;
    }
    while(1){
        DisplayText(0U, "Display test", 12U, 0U);
        DisplayValue(0U, DisplayValueTest, 4U, 12U);
        DisplayGraph(1U, DisplayGraphTest, 128U, 3U);
        volatile int Delay = 100000;
        while(Delay){
            Delay--;
        }

        DisplayValueTest++;
        for(uint8 i=0; i<128U;i++){
            DisplayGraphTest[i]++;
            if(DisplayGraphTest[i]>=100U){
                DisplayGraphTest[i] = 0U;
            }
        }
        DisplayRefresh();
    }
}

void ReceiverTest(){
    int ReceiverChannels[8];
    uint8 DisplayValueOffset;
    /*Display static text first*/
    DisplayText(0U, "Ch0:    Ch1:", 12U, 0U);
    DisplayText(1U, "Ch2:    Ch3:", 12U, 0U);
    DisplayText(2U, "Ch4:    Ch5:", 12U, 0U);
    DisplayText(3U, "Ch6:    Ch7:", 12U, 0U);

    while(1){
        for(uint8 i=0U; i<8U; i++){
            ReceiverChannels[i] = GetReceiverChannel(i);/*Save all receiver channel values*/
            DisplayValueOffset = 4U+8U*(i%2U);
            DisplayValue(i/2U, ReceiverChannels[i], 4U, DisplayValueOffset);
        }
        DisplayRefresh();
    }

}

void ServoTest() {
    volatile int Delay;
    volatile int a = 0;
    while(1){
    	a= a+5 ;
        Delay = 8000000;
        Steer(a); //destra
        //HbridgeSetSpeed(-100);
        while(Delay){
            Delay--;
        }
//      Delay = 8000000;
        Steer(-a);//sinistra
        //HbridgeSetSpeed(100);
        while(Delay){
            Delay--;
        }

    }
}

void MotorTest() {
    while(1){
        HbridgeSetSpeed(-80);
        }
}

void LinearCameraTest(){
    DisplayText(0U, "Missed ADC:", 11U, 0U);
    while(1){
        DisplayValue(0U, AdcMissedValues, 5U, 11U);
        DisplayGraph(1U, CameraResultsBuffer, 128U, 3U);
        DisplayRefresh();
    }
}

#ifdef __cplusplus
}
#endif

/** @} */
