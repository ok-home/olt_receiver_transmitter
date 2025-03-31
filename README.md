# Simple exchange according to the OpenLaserTag IR protocol (ESP32, ESP32S3, ESP32C3)
  - [OpenLaserTag IR protocol](https://openlasertag.org/language/en/openlasertag-ir-communication-protocol/) for ESP32 RMT
  - Protocol extended to 32 bits for Command (Long) Packet
  - Support Tx Carrier, Rx carrier detect (ESP32S3,ESP32C3)
## interface functions and gpio setting
```
// https://github.com/ok-home/olt_receiver_transmitter/blob/main/src/olt_receiver_transmitter.c
// max rx channels esp32 = 8, esp32s3 = 4, esp32c3 = 2
// for esp32 rx&tx channels shared -> rx channels = 1->tx, 7->rx
#define OLT_RX_CHANNELS_NUM 7
static const gpio_num_t olt_tx_gpio = 2; // tx channel gpio
static const gpio_num_t olt_rx_gpio[OLT_RX_CHANNELS_NUM] = {2,2,2,2,2,2,2}; // rx channels gpio, for test rx gpio = tx gpio

// https://github.com/ok-home/olt_receiver_transmitter/blob/main/include/olt_receiver_transmitter.h
esp_err_t olt_tx_channel_init(void);    // init tx channel
esp_err_t olt_rx_channels_init(void);   // init up to 8 rx channels (ESP32), 4 rx channels(esp32s3), 2 rx channels (esp32c3)
esp_err_t olt_tx_channel_free(void);    // disable rmt tx channel & free all resource
esp_err_t olt_rx_channels_free(void);   // disable all rmt rx channel & free all resource
esp_err_t olt_tx_data(olt_packet_t data,TickType_t xTicksToWait); // transmit olt_packet_t data
esp_err_t olt_tx_wait_all_done(TickType_t xTicksToWait); // wait all rmt data transmitted
esp_err_t olt_rx_data(olt_rx_data_t *data,TickType_t xTicksToWait); // receive olt_packet_t data
```
## transmit/receive data format
```
// OpenlaserTag item
// https://github.com/ok-home/olt_receiver_transmitter/blob/main/include/olt_receiver_transmitter.h

typedef union {
    struct 
    {
        uint32_t free:10;           // not used on shot packet
        uint32_t player_id_ext:8;
        uint32_t damage:4;
        uint32_t team:2;
        uint32_t player_id_base:7;
        uint32_t shot_long_bit:1;
    } shot_packet;
    struct 
    {
        uint32_t byte_ext:8;        // extend long packet
        uint32_t byte_3:8;
        uint32_t byte_2:8;
        uint32_t byte_1:8;
    } long_packet;
    uint32_t val;
} olt_packet_t;

// rx packet with received channel
typedef struct 
{
    uint32_t channel;
    olt_packet_t data;
} olt_rx_data_t;
```
## connects as a standard ESP-IDF component
## Simple example of use
  - https://github.com/ok-home/olt_receiver_transmitter/blob/main/olt_tx_rx_example/main/olt_tx_rx_example.c