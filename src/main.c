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
#include "OsIf_Timer_System_Internal_Systick.h"
#include "Mcal.h"
#include "pixy2.h"
#include "Osif.h"
#include "Dio.h"
#include "Gpt.h"
#include <math.h>
#include <stddef.h>
/*==================================================================================================
 *                          LOCAL TYPEDEFS (STRUCTURES, UNIONS, ENUMS)
==================================================================================================*/

/*==================================================================================================
 *                                       LOCAL MACROS
==================================================================================================*/

/*==================================================================================================
 *                                      LOCAL CONSTANTS
==================================================================================================*/
#define ULTRASONIC_TIMEOUT 65535U /* Timeout per la misura (5.46 ms in microsecondi) */
#define SOUND_SPEED_CM_PER_US 0.0343f /* Velocità del suono in cm/us */
#define TRIGGER_PULSE_DURATION_US 10U /* Durata impulso trigger in microsecondi */

#define MIN_DIFF -5
#define MAX_DIFF 5
/*==================================================================================================
 *                                      LOCAL VARIABLES
==================================================================================================*/
static Gpt_ChannelType EchoTimer;
static Dio_ChannelType TriggerPin;
static Dio_ChannelType EchoPin1;
static Dio_ChannelType EchoPin2;

static volatile double EchoTime;
static volatile boolean MeasurementDone;
volatile boolean triggerdone;

unsigned long startTime = 0, endTime = 0;

/*==================================================================================================
 *                                      GLOBAL CONSTANTS
==================================================================================================*/

/*==================================================================================================
 *                                      GLOBAL VARIABLES
==================================================================================================*/
extern uint8 CameraResultsBuffer[128U];
extern uint16 AdcMissedValues;
Vector NormalizedVectors[4];
/*==================================================================================================
 *                                   LOCAL FUNCTION PROTOTYPES
==================================================================================================*/
/*==================================================================================================
 *                                       LOCAL FUNCTIONS
==================================================================================================*/
void Ultrasonic_Init(Dio_ChannelType trigger, Dio_ChannelType echo1, Dio_ChannelType echo2, Gpt_ChannelType echo_timer) {
    TriggerPin = trigger;
    EchoPin1 = echo1;
    EchoPin2 = echo2;
    EchoTimer = echo_timer;
    MeasurementDone = FALSE;
    triggerdone = FALSE;
    EchoTime = 0;
    Gpt_EnableNotification(EchoTimer);
}

void Ultrasonic_Trigger(void) {

    Dio_WriteChannel(TriggerPin, (Dio_LevelType)STD_HIGH);
    for (volatile int i = 0; i < 1; i++); /* 10 us Delay per garantire impulso trigger */
    Dio_WriteChannel(TriggerPin, (Dio_LevelType)STD_LOW);
    triggerdone = TRUE;
}

/*float Ultrasonic_CalculateDistance() {
    float echoTime1 = 0.0f, echoTime2 = 0.0f;
    uint16 start1 = 0, start2 = 0, end1 = 0, end2 = 0;
    bool started1 = false, started2 = false;
    bool ended1 = false, ended2 = false;

    Gpt_StartTimer(EchoTimer, (Gpt_ValueType)ULTRASONIC_TIMEOUT);

    // Loop di polling per leggere i due EchoPin simultaneamente
    while (!(ended1 && ended2)) {
        Dio_LevelType level1 = Dio_ReadChannel(EchoPin1);
        Dio_LevelType level2 = Dio_ReadChannel(EchoPin2);
        Gpt_ValueType now = Gpt_GetTimeElapsed(EchoTimer);

        // Gestione EchoPin1
        if (!started1 && level1 == STD_HIGH) {
            start1 = now;
            started1 = true;
        }
        if (started1 && !ended1 && level1 == STD_LOW) {
            end1 = now;
            ended1 = true;
        }

        // Gestione EchoPin2
        if (!started2 && level2 == STD_HIGH) {
            start2 = now;
            started2 = true;
        }
        if (started2 && !ended2 && level2 == STD_LOW) {
            end2 = now;
            ended2 = true;
        }
    }

    Gpt_StopTimer(EchoTimer);

    // Calcolo dei tempi di Echo per ciascun sensore
    if (end1 >= start1)
        echoTime1 = (float)(end1 - start1);
    else
        echoTime1 = (float)((65535 - start1) + end1);

    if (end2 >= start2)
        echoTime2 = (float)(end2 - start2);
    else
        echoTime2 = (float)((65535 - start2) + end2);

    // Conversione da ticks a µs
    echoTime1 *= 0.08333f;
    echoTime2 *= 0.08333f;

    MeasurementDone = TRUE;

    // Restituisce la distanza media in cm
    return ((echoTime1 / 58.0f) + (echoTime2 / 58.0f)) / 2.0f;
}*/

double Ultrasonic_CalculateDistance(void) {
	Gpt_StartTimer(EchoTimer, (Gpt_ValueType) ULTRASONIC_TIMEOUT);

	if (Dio_ReadChannel(EchoPin1) == (Dio_LevelType)STD_HIGH){
	    startTime = Gpt_GetTimeElapsed(EchoTimer);
	    while (Dio_ReadChannel(EchoPin1) == (Dio_LevelType)STD_HIGH);
	    endTime = Gpt_GetTimeElapsed(EchoTimer);
	    if (endTime >= startTime) {
	        EchoTime = (double)(endTime - startTime);
	    }
	    else {
	        EchoTime = (double)((65535 - startTime) + endTime);
	    }
	    //endTime = Gpt_GetTimeElapsed(EchoTimer);
	    EchoTime = EchoTime*0.08333;
	    MeasurementDone = TRUE;
	}

	Gpt_StopTimer(EchoTimer);
	return (EchoTime)/58; // * SOUND_SPEED_CM_PER_US)/2.0f;
}

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


int main(void)
{
	static int CarSteer = 0;
	//double m0 = 0 , m1 = 0;
    DetectedVectors PixyVectors;
    double distance = 0.0;
	volatile uint16 Delay = 10000;
	//int motorSpeedLeft, motorSpeedRight;
	int a = 0;
	int b = 0;

	double m[4] = {0};        // Array per le pendenze
	size_t validCount = 0;    // Quanti vettori validi troviamo
	double sum = 0.0;         // Somma delle pendenze valide

    //Initialize RTD drivers with the compiled configurations/
    DriversInit();
    ServoInit(1U, 3300U, 1700U, 2500U);
    HbridgeInit(2U, 3U, 32U, 33U, 6U, 64U);
    DisplayInit(1U);
    Pixy2Init(0x54, 0U);
    LinearCameraInit(4U, 1U, 0U, 97U);
    //Inizializza il sensore ultrasonico
    Ultrasonic_Init(99U, 101U, 108U, 2U); //trig ptd3,echo1 ptd5,echo2 ptd12,id Gpt timer

    HbridgeSetBrake(0);

    while(1){

    	DisplayClear();

        Ultrasonic_Trigger();

        while(!MeasurementDone){
        	distance = Ultrasonic_CalculateDistance();
        }

        distance = distance*1.64; // rapporto di conversione K è di 1.64

    	PixyVectors = PixyGetVectors();

        for(uint8 Index = 0U; Index < 4; Index++){
        	NormalizedVectors[Index] = NormalizePixyVector(PixyVectors.Vectors[Index]);
            DisplayVector(NormalizePixyVector(PixyVectors.Vectors[Index]));
        }

        for (size_t i = 0; i < 4 && i < PixyVectors.NumberOfVectors; i++) {
            double dx = (double)(PixyVectors.Vectors[i].x0 - PixyVectors.Vectors[i].x1);
            double dy = (double)(PixyVectors.Vectors[i].y0 - PixyVectors.Vectors[i].y1);

            // Se la variazione verticale è troppo piccola, setta m[i] a 0
            if (fabs(dy) <= 5) {
                m[i] = 0;
            } else {
                m[i] = dx / dy;  // Calcola il coefficiente angolare per il vettore valido
            }

            // Controllo per l'area centrale della finestra
            if (PixyVectors.Vectors[i].x0 >= 26 && PixyVectors.Vectors[i].x0 <= 52 &&
                PixyVectors.Vectors[i].y0 >= 1 && PixyVectors.Vectors[i].y0 <= 51 &&
                PixyVectors.Vectors[i].y1 >= 1 && PixyVectors.Vectors[i].y1 <= 51 &&
                PixyVectors.Vectors[i].y0 != 0 && PixyVectors.Vectors[i].y1 != 0) {

            	// Aggiungi logica aggiuntiva se il vettore è dentro l'area centrale
                if (fabs(dy) <= 5) {
                   b = b+1;
                }
            }
            else{
            	// Aggiungi logica aggiuntiva se il vettore è dentro l'area centrale
                if (fabs(dy) <= 5) {
                	m[i] = 0;  // Se la variazione verticale è troppo piccola, mantieni 0
                }
            }

            sum += m[i];  // Somma il valore di m[i] (sia che sia 0 o il valore calcolato)
        }

        // Calcolo media e CarSteer
        double averageSlope = sum/2;
        CarSteer = (int)(averageSlope * 100);

        Steer(CarSteer);


        if (distance <= 50) {
            // Se la distanza è inferiore a 50 cm, fermiamo la macchina
        	a = a+1;
        	if(a >= 100){
        		HbridgeSetSpeed(0);
        	 }
        	else {
        		HbridgeSetSpeed(-35);
        	}

        } else {
        	a = 0;
        	if( b == 0){

                HbridgeSetSpeed(50);
        	}
        	if(b >= 2){
        		HbridgeSetSpeed(30);
        	}

         }

        //DisplayValue(3U, a, 4U, 5U);
        //DisplayValue(2U, CarSteer, 4U, 5U);
        //DisplayValue(1U, HbridgeGetSpeedMotor1(), 4U, 5U);
        //DisplayValue(0U, HbridgeGetSpeedMotor2(), 4U, 5U);

        DisplayRefresh();
        MeasurementDone = FALSE;
        CarSteer = 0;
        averageSlope = 0;
        sum = 0;

    }
}


#ifdef __cplusplus
}
#endif

/** @} */
