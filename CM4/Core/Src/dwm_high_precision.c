/*
 * dwm_high_precision.c
 *
 */
//Implementa la logica di alto livello, usando le funzioni di api_trimmed.c.
#include "dwm_high_precision.h"
#include "api_trimmed.h"

/*HAL_StatusTypeDef DWM_Init(UART_HandleTypeDef *huart) {
    // 1. Init hardware
    if(dwm_init() != 0) return HAL_ERROR;

    // 2. Configurazione tag
    dwm_cfg_tag_t cfg = {
        .common.uwb_mode = 2, // DWM_UWB_MODE_ACTIVE
        .common.ble_en = false,
        .meas_mode = 0, // DWM_MEAS_MODE_TWR
        .loc_engine_en = true
    };
    if(dwm_cfg_tag_set(&cfg) != 0) return HAL_ERROR;

    // 3. Parametri UWB
    if(dwm_uwb_cfg_set(0xC0, 0x1F1F1F1F) != 0)
        return HAL_ERROR;

    return HAL_OK;
}*/
// Sostituisci la funzione DWM_Init con questa versione modificata
HAL_StatusTypeDef DWM_Init(UART_HandleTypeDef *huart) {
    // 1. Init hardware
    if(dwm_init() != 0) return HAL_ERROR;

    // 2. Configurazione tag (verificata nel datasheet)
    dwm_cfg_tag_t cfg = {
        .common.uwb_mode = 2,    // DWM_UWB_MODE_ACTIVE
        .common.ble_en = true,
        .meas_mode = 0,          // DWM_MEAS_MODE_TWR
        .loc_engine_en = true
    };
    if(dwm_cfg_tag_set(&cfg) != 0) return HAL_ERROR;

    // 3. Parametri UWB modificati (pg_delay e tx_power ridotto)
    if(dwm_uwb_cfg_set(0xC0, 0x0D0D0D0D) != 0)  // pg_delay=0xC0, tx_power=0x0D0D0D0D
        return HAL_ERROR;

    return HAL_OK;
}


HAL_StatusTypeDef DWM_GetDistances(DistanceMeasurement *meas, uint8_t *count) {
    dwm_loc_data_t loc;
    *count = 0;
    if (dwm_loc_get(&loc) != 0) {
            return HAL_ERROR;  // Fallimento UART
        }

        *count = loc.anchors.dist.cnt;  // Potrebbe essere 0
        for(uint8_t i = 0; i < *count; i++) {
            meas[i].anchor_id = loc.anchors.dist.addr[i];
            meas[i].distance_m = loc.anchors.dist.dist[i] / 1000.0f;
            meas[i].quality = loc.anchors.dist.qf[i];
        }
    /*if(dwm_loc_get(&loc) != 0) return HAL_ERROR; // Legge dati low-level

    *count = loc.anchors.dist.cnt;
    for(uint8_t i=0; i<*count; i++) {
        meas[i].anchor_id = loc.anchors.dist.addr[i];
        meas[i].distance_m = loc.anchors.dist.dist[i] / 1000.0f;
        meas[i].quality = loc.anchors.dist.qf[i];
    }*/

    return HAL_OK;
}

//Conversione Unità: Trasforma i mm in metri direttamente qui.
//Gestione Errori: Mappa i codici errore del DWM1001 su HAL_StatusTypeDef.
