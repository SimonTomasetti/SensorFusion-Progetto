/*
 * dwm_high_precision.h
 *

 */
//Definisce l'interfaccia pulita per l'applicativo, nascondendo i dettagli del DWM1001.
#ifndef INC_DWM_HIGH_PRECISION_H_
#define INC_DWM_HIGH_PRECISION_H_

#include "stm32h7xx_hal.h"
#include <stdbool.h>

typedef struct {
    uint16_t anchor_id;
    float distance_m;
    uint8_t quality;
} DistanceMeasurement;

HAL_StatusTypeDef DWM_Init(UART_HandleTypeDef *huart); // Inizializza con HAL
HAL_StatusTypeDef DWM_GetDistances(DistanceMeasurement *meas, uint8_t *count); // Legge distanze
// Converti i dati "grezzi" del DWM1001 in un formato più usabile (float invece di uint32_t).
//Usa i tipi HAL (HAL_StatusTypeDef) per compatibilità con STM32CubeIDE

#endif /* INC_DWM_HIGH_PRECISION_H_ */
