#pragma once
typedef union {
    struct 
    {
        uint32_t free:10;
        uint32_t player_id_ext:8;
        uint32_t damage:4;
        uint32_t team:2;
        uint32_t player_id_base:7;
        uint32_t shot_long_bit:1;
    } shot_packet;
    struct 
    {
        uint32_t byte_ext:8;
        uint32_t byte_3:8;
        uint32_t byte_2:8;
        uint32_t byte_1:8;
    } long_packet;
    uint32_t val;
} olt_packet_t;

typedef struct 
{
    uint32_t channel;
    olt_packet_t data;
} olt_rx_data_t;

#ifdef __cplusplus
extern "C"
{
#endif
esp_err_t olt_tx_channel_init(void);
esp_err_t olt_rx_channels_init(void);
esp_err_t olt_tx_channel_free(void);
esp_err_t olt_rx_channels_free(void);
esp_err_t olt_tx_data(olt_packet_t data,TickType_t xTicksToWait);
esp_err_t olt_tx_wait_all_done(TickType_t xTicksToWait);
esp_err_t olt_rx_data(olt_rx_data_t *data,TickType_t xTicksToWait);
#ifdef __cplusplus
}
#endif