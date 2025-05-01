/*
 * api_trimmed.c
 *

 */
//Fornisce l'implementazione delle funzioni dichiarate in api_trimmed.h
#include "api_trimmed.h"
#include "stm32h7xx_hal.h"

extern UART_HandleTypeDef* dwm_huart;  // Definita in main.c

//static uint8_t uart_rx_buffer[128];
//static uint8_t uart_tx_buffer[128];

// Funzioni private
static int _dwm_send_command(const uint8_t* cmd, uint8_t cmd_len) {
    return (HAL_UART_Transmit(dwm_huart, cmd, cmd_len, 100) == HAL_OK) ? 0 : -1;
}

static int _dwm_receive_response(uint8_t* buf, uint16_t buf_size, uint32_t timeout) {
    return (HAL_UART_Receive(dwm_huart, buf, buf_size, timeout) == HAL_OK) ? 0 : -1;
}

// Funzioni pubbliche
int dwm_init(void) {
    const uint8_t reset_cmd[] = {0xDE, 0x01};  // Formato corretto dal datasheet
    return _dwm_send_command(reset_cmd, sizeof(reset_cmd));
}

/*int dwm_init(void) {
    const uint8_t reset_cmd[] = {0x01};  // Comando reset
    return _dwm_send_command(reset_cmd, sizeof(reset_cmd));
}*/

int dwm_cfg_tag_set(dwm_cfg_tag_t* cfg) {
    uint8_t config_cmd[] = {
        0x02,  // Opcode configurazione
        (cfg->common.uwb_mode << 2) | (cfg->loc_engine_en << 4),
        cfg->meas_mode
    };
    return _dwm_send_command(config_cmd, sizeof(config_cmd));
}

int dwm_uwb_cfg_set(uint8_t pg_delay, uint32_t tx_power) {
    uint8_t config_cmd[] = {
        0x03,  // Opcode UWB config
        pg_delay,
        (tx_power >> 24) & 0xFF,
        (tx_power >> 16) & 0xFF,
        (tx_power >> 8) & 0xFF,
        tx_power & 0xFF
    };
    return _dwm_send_command(config_cmd, sizeof(config_cmd));
}
int dwm_loc_get(dwm_loc_data_t* loc) {
    const uint8_t get_dist_cmd[] = {0x0C, 0x00};  // TLV: T=0x0C (get distances), L=0

    if (_dwm_send_command(get_dist_cmd, sizeof(get_dist_cmd)) != 0) {
        return -1;
    }
    uint8_t response[64];  // buffer temporaneo
    if (_dwm_receive_response(response, sizeof(response), 1000) != 0) {
        return -1;
    }
    // Controlla tipo risposta
    if (response[0] != 0x0C) {
        return -1;
    }
    uint8_t len = response[1];  // L = lunghezza payload
    if (len == 0 || len > sizeof(response) - 2) {
        return -1;
    }
    uint8_t anchor_cnt = response[2];  // primo byte utile = numero di anchor
    loc->anchors.dist.cnt = anchor_cnt;

    for (uint8_t i = 0; i < anchor_cnt; i++) {
        uint8_t base = 3 + i * 7;
        loc->anchors.dist.addr[i] = (response[base] << 8) | response[base + 1];
        loc->anchors.dist.dist[i] = (response[base + 2] << 24) |
                                    (response[base + 3] << 16) |
                                    (response[base + 4] << 8) |
                                    response[base + 5];
        loc->anchors.dist.qf[i] = response[base + 6];
    }

    return 0;
}


/*int dwm_loc_get(dwm_loc_data_t* loc) {
    const uint8_t get_dist_cmd[] = {0x04};  // Opcode get distance (verifica dal datasheet)

    if (_dwm_send_command(get_dist_cmd, sizeof(get_dist_cmd)) != 0)
        return -1;*/


/*int dwm_loc_get(dwm_loc_data_t* loc) {
    const uint8_t get_dist_cmd[] = {0xDE, 0x04};  // Formato corretto
    if (_dwm_send_command(get_dist_cmd, sizeof(get_dist_cmd)) != 0) return -1;
    uint8_t response[20];
    if (_dwm_receive_response(response, sizeof(response), 2000) != 0)
        return -1;

    // Parsing dei dati (adatta al formato reale del DWM1001)
    loc->anchors.dist.cnt = response[0];
    for (uint8_t i = 0; i < loc->anchors.dist.cnt; i++) {
        loc->anchors.dist.addr[i] = (response[1 + i*7] << 8) | response[2 + i*7];
        loc->anchors.dist.dist[i] = (response[3 + i*7] << 24) | (response[4 + i*7] << 16) |
                                   (response[5 + i*7] << 8) | response[6 + i*7];
        loc->anchors.dist.qf[i] = response[7 + i*7];
    }
    return 0;
}*/
/*#include "api_trimmed.h"
#include "stm32h7xx_hal.h"

// Dichiarazioni esterne (implementate nel tuo main.c)
extern int dwm_spi_read(uint16_t reg_addr, uint8_t* data, uint8_t len);
extern int dwm_spi_write(uint16_t reg_addr, uint8_t* data, uint8_t len);
extern void dwm_delay(uint32_t ms);

int dwm_init(void) {
    uint8_t reset_cmd = 0x01;
    return dwm_spi_write(0x04, &reset_cmd, 1); // Soft reset
}

int dwm_cfg_tag_set(dwm_cfg_tag_t* cfg) {
    uint8_t config_data[3] = {
        (cfg->common.uwb_mode << 2) | (cfg->loc_engine_en << 4),
        cfg->meas_mode,
        0x00 // Padding
    };
    return dwm_spi_write(0x0D, config_data, 3); // Scrittura configurazione
}

int dwm_uwb_cfg_set(uint8_t pg_delay, uint32_t tx_power) {
    uint8_t config_data[5] = {
        pg_delay,
        (tx_power >> 24) & 0xFF,
        (tx_power >> 16) & 0xFF,
        (tx_power >> 8) & 0xFF,
        tx_power & 0xFF
    };
    return dwm_spi_write(0x1A, config_data, 5); // Scrittura parametri UWB
}

int dwm_loc_get(dwm_loc_data_t* loc) {
    uint8_t rx_data[14];
    if (dwm_spi_read(0x15, rx_data, 14) != 0) return -1;

    // Esempio: estrae solo il primo anchor rilevato
    loc->anchors.dist.cnt = 1;
    loc->anchors.dist.addr[0] = (rx_data[1] << 8) | rx_data[0]; // ID anchor
    loc->anchors.dist.dist[0] = (rx_data[5] << 24) | (rx_data[4] << 16) |
                               (rx_data[3] << 8) | rx_data[2]; // Distanza in mm
    loc->anchors.dist.qf[0] = rx_data[6]; // Quality factor

    return 0;
}*/
