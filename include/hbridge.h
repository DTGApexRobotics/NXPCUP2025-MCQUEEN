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
#ifndef HBRIDGE_H
#define HBRIDGE_H

#ifdef __cplusplus
extern "C" {
#endif

/*==================================================================================================
*                                        INCLUDE FILES
* 1) system and project includes
* 2) needed interfaces from external units
* 3) internal and external interfaces from this unit
==================================================================================================*/
#include "Pwm.h"
#include "Dio.h"    /* Assicurarsi che Dio.h fornisca i tipi necessari (es. Dio_ChannelType) */

/*==================================================================================================
*                          LOCAL TYPEDEFS (STRUCTURES, UNIONS, ENUMS)
==================================================================================================*/
/**
 * @brief  Struttura che definisce i canali utilizzati dall'H-bridge per il controllo dei due motori.
 */
typedef struct {
    Pwm_ChannelType Motor1_Speed;
    Pwm_ChannelType Motor2_Speed;
    Dio_ChannelType Motor1_Forward;
    Dio_ChannelType Motor1_Backward;
    Dio_ChannelType Motor2_Forward;
    Dio_ChannelType Motor2_Backward;
} Hbridge;

/*==================================================================================================
*                                       GLOBAL FUNCTION PROTOTYPES
==================================================================================================*/

/**
 * @brief Aggiorna il duty cycle per entrambi i motori con lo stesso valore.
 */
void Hbridge_Period_Finished(void);

/**
 * @brief Aggiorna il duty cycle dei due motori in maniera indipendente.
 */
void Hbridge_Period_Finished_1(void);

/**
 * @brief Inizializza l'H-bridge impostando i canali PWM e i canali DIO per il controllo dei motori.
 *
 * @param Motor1_Speed    Canale PWM per il motore 1.
 * @param Motor2_Speed    Canale PWM per il motore 2.
 * @param Motor1_Forward  Canale DIO per la direzione "avanti" del motore 1.
 * @param Motor1_Backward Canale DIO per la direzione "indietro" del motore 1.
 * @param Motor2_Forward  Canale DIO per la direzione "avanti" del motore 2.
 * @param Motor2_Backward Canale DIO per la direzione "indietro" del motore 2.
 */
void HbridgeInit(Pwm_ChannelType Motor1_Speed, Pwm_ChannelType Motor2_Speed,
                 Dio_ChannelType Motor1_Forward, Dio_ChannelType Motor1_Backward,
                 Dio_ChannelType Motor2_Forward, Dio_ChannelType Motor2_Backward);

/**
 * @brief Variante di inizializzazione dell'H-bridge (idem a HbridgeInit).
 */
void HbridgeInit_1(Pwm_ChannelType Motor1_Speed, Pwm_ChannelType Motor2_Speed,
                   Dio_ChannelType Motor1_Forward, Dio_ChannelType Motor1_Backward,
                   Dio_ChannelType Motor2_Forward, Dio_ChannelType Motor2_Backward);

/**
 * @brief Imposta la velocità per entrambi i motori utilizzando lo stesso valore.
 *
 * @param Speed Valore della velocità (da -100 a 100; >=0 avanti, <0 retromarcia).
 */
void HbridgeSetSpeed(int Speed);

/**
 * @brief Imposta in modo indipendente la velocità dei due motori.
 *
 * @param SpeedMotor1 Velocità per il motore 1 (da -100 a 100).
 * @param SpeedMotor2 Velocità per il motore 2 (da -100 a 100).
 */
void HbridgeSetSpeedIndependent(int SpeedMotor1, int SpeedMotor2);

/**
 * @brief Restituisce la velocità impostata (comune) per entrambi i motori.
 *
 * @return Velocità corrente.
 */
int HbridgeGetSpeed(void);

/**
 * @brief Restituisce la velocità corrente impostata per il motore 1.
 *
 * @return Velocità del motore 1.
 */
int HbridgeGetSpeedMotor1(void);

/**
 * @brief Restituisce la velocità corrente impostata per il motore 2.
 *
 * @return Velocità del motore 2.
 */
int HbridgeGetSpeedMotor2(void);

/**
 * @brief Imposta lo stato del freno (handbrake) per il controllo comune.
 *
 * @param Brake Valore non zero per attivare il freno, zero per rilasciarlo.
 */
void HbridgeSetBrake(uint8 Brake);

/**
 * @brief Imposta lo stato del freno (handbrake) per il controllo indipendente dei motori.
 *
 * @param Brake Valore non zero per attivare il freno, zero per rilasciarlo.
 */
void HbridgeSetBrake_1(uint8 Brake);

/**
 * @brief Restituisce lo stato attuale del freno.
 *
 * @return Stato del freno (0: disattivato, diverso da 0: attivato).
 */
uint8 HbridgeGetBrake(void);

#ifdef __cplusplus
}
#endif

#endif /* HBRIDGE_H */
