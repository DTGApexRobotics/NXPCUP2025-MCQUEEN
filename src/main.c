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

#include "display.h"
#include "receiver.h"
#include "CDD_I2c.h"
#include "esc.h"
#include "servo.h"
#include "linear_camera.h"
#include "main_functions.h"
#include "hbridge.h"
#include "Mcal.h"
#include "pixy2.h"
#include "Osif.h"
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
void Delay(uint32_t delay_ms);  // Aggiungi il prototipo della funzione Delay qui

/*==================================================================================================
 *                                   LOCAL FUNCTION PROTOTYPES
==================================================================================================*/
void Delay(uint32_t delay_ms) {
    uint32_t start_time = OsIf_GetCounter(OSIF_COUNTER_DUMMY);  // Use dummy counter if system counter is not defined
    uint32_t delay_ticks = OsIf_MicrosToTicks(delay_ms * 1000, OSIF_COUNTER_DUMMY);  // Convert milliseconds to ticks

    while (OsIf_GetCounter(OSIF_COUNTER_DUMMY) - start_time < delay_ticks) {
        // Loop until the elapsed time reaches the desired delay
    }
}

/*==================================================================================================
 *                                       LOCAL FUNCTIONS
==================================================================================================*/

/*==================================================================================================
 *                                       GLOBAL FUNCTIONS
==================================================================================================*/
/**
 * @brief        Main function of the example
 * @details      Initializes the used drivers, hbridge, Pixy and servo.  The code
 *                  tries to keep the car between the two black lines using a Pixy2 camera.
 */


Vector NormalizePixyVector(Vector PixyVector){
    Vector NormalizedVector;
    NormalizedVector.VectorIndex = PixyVector.VectorIndex;
    NormalizedVector.x0 = PixyVector.x0 * 100U / 78U;
    NormalizedVector.y0 = PixyVector.y0 * 100U / 51U;
    NormalizedVector.x1 = PixyVector.x1 * 100U / 78U;
    NormalizedVector.y1 = PixyVector.y1 * 100U / 51U;
    return NormalizedVector;
}

void processCarSteer(int CarSteer) {
    if (CarSteer >= -5 && CarSteer <= 5) {
    	SteerStraight();
    } else if (CarSteer > 5 && CarSteer <= 10) {
    	SteerStraight();
    } else if (CarSteer > 10 && CarSteer <= 20) {
    	SteerStraight();
    } else if (CarSteer > 20 && CarSteer <= 30) {
    	SteerStraight();
    } else if (CarSteer > 30 && CarSteer <= 40) {
    	SteerRight();
    } else if (CarSteer > 40 && CarSteer <= 50) {
    	SteerRight();
    } else if (CarSteer > 50 && CarSteer <= 60) {
    	SteerRight();
    } else if (CarSteer > 60 && CarSteer <= 70) {
    	SteerRight();
    } else if (CarSteer > 70 && CarSteer <= 80) {
    	SteerRight();
    } else if (CarSteer > 80 && CarSteer <= 90) {
    	SteerRight();
    } else if (CarSteer > 90 && CarSteer <= 100) {
    	SteerRight();
    } else if (CarSteer < -5 && CarSteer >= -10) {
    	SteerStraight();
    } else if (CarSteer < -10 && CarSteer >= -20) {
    	SteerStraight();
    } else if (CarSteer < -20 && CarSteer >= -30) {
    	SteerStraight();
    } else if (CarSteer < -30 && CarSteer >= -40) {
    	SteerLeft();
    } else if (CarSteer < -40 && CarSteer >= -50) {
    	SteerLeft();
    } else if (CarSteer < -50 && CarSteer >= -60) {
    	SteerLeft();
    } else if (CarSteer < -60 && CarSteer >= -70) {
    	SteerLeft();
    } else if (CarSteer < -70 && CarSteer >= -80) {
    	SteerLeft();
    } else if (CarSteer < -80 && CarSteer >= -90) {
    	SteerLeft();
    } else if (CarSteer < -90 && CarSteer >= -100) {
    	SteerLeft();
    }
}

int main(void)
{
	static int CarSteer = 0;
	double m0 = 0 , m1 = 0;
	volatile uint16 Delay = 10000;
    //Initialize RTD drivers with the compiled configurations/
    DriversInit();
    ServoInit(1U, 3300U, 1700U, 2500U);
    HbridgeInit(2U, 3U, 32U, 33U, 6U, 64U);
    DisplayInit(1U);
    Pixy2Init(0x54, 0U);
    //ServoTest();
    //Pixy2Test();

    DetectedVectors PixyVectors;


    while(1){
    	DisplayClear();
    	PixyVectors = PixyGetVectors();

        for(uint8 Index = 0U; Index < 2; Index++){
            DisplayVector(NormalizePixyVector(PixyVectors.Vectors[Index]));
        }
    	if (PixyVectors.NumberOfVectors > 0) {
    	            // Calcola il coefficiente angolare per la prima linea
    	     //Vector1 = PixyVectors.Vectors[0];
    	            // Evito divisione per zero: controllo che il denominatore sia diverso da 0
    	     if ((PixyVectors.Vectors[0].y0 - PixyVectors.Vectors[0].y1) != 0)
    	         m0 = (double)(PixyVectors.Vectors[0].x0 - PixyVectors.Vectors[0].x1) / (double)(PixyVectors.Vectors[0].y0 - PixyVectors.Vectors[0].y1);
    	     else
    	    	 m0 = 0;

    	     // Se sono presenti almeno due vettori, calcolo anche il secondo coefficiente angolare
    	     if (PixyVectors.NumberOfVectors > 1) {
    	    	 //Vector2 = PixyVectors.Vectors[1];
    	    	 if ((PixyVectors.Vectors[1].y0 - PixyVectors.Vectors[1].y1) != 0)
    	    		 m1 = (double)(PixyVectors.Vectors[1].x0 - PixyVectors.Vectors[1].x1) / (double)(PixyVectors.Vectors[1].y0 - PixyVectors.Vectors[1].y1);
    	    	 else
    	    		 m1 = 0;
    	     }

    	     else {
    	         m1 = 0;  // Reset m1 se c'è solo un vettore
    	     }
    	}
    	//error = (m0 + m1)/2;
        //m0 = (double)(PixyVectors.Vectors[0].x0 - PixyVectors.Vectors[0].x1) / (double)(PixyVectors.Vectors[0].y0 - PixyVectors.Vectors[0].y1);
        //m1 = (double)(PixyVectors.Vectors[1].x0 - PixyVectors.Vectors[1].x1) / (double)(PixyVectors.Vectors[1].y0 - PixyVectors.Vectors[1].y1);

        CarSteer = (int)((((m0 + m1)/2)*100) -50) ;
        //processCarSteer(CarSteer);
        Steer(CarSteer);
        HbridgeSetSpeed(40);

        DisplayRefresh();
        Delay=10000U;
        while(Delay){
            Delay--;
        }

    }
}


#ifdef __cplusplus
}
#endif

/** @} */
