/*
 * api_trimmed.h
 *
 */
/*È un'interfaccia minimalista per il DWM1001, contenente solo le strutture e funzioni necessarie per le misurazioni di distanza.
 * Sostituisce l'header originale del vendor (dwm_api.h), rimuovendo il 70% di codice non necessario, migliorando leggibilità e prestazioni. */

#ifndef INC_API_TRIMMED_H_
#define INC_API_TRIMMED_H_

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint8_t cnt;
    uint16_t addr[14];
    uint32_t dist[14];
    uint8_t qf[14];
} dwm_distance_t;

typedef struct {
    dwm_distance_t dist;
} dwm_ranging_anchors_t;

typedef struct {
    void* p_pos;
    dwm_ranging_anchors_t anchors;
} dwm_loc_data_t;

int dwm_init(void);
int dwm_loc_get(dwm_loc_data_t* loc);
int dwm_uwb_cfg_set(uint8_t pg_delay, uint32_t tx_power);

typedef struct {
    struct {
        uint8_t uwb_mode;
        bool ble_en;
    } common;
    uint8_t meas_mode;
    bool loc_engine_en;
} dwm_cfg_tag_t;

int dwm_cfg_tag_set(dwm_cfg_tag_t* cfg);

#endif
