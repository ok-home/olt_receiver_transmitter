/* OpenLaserTag IR receiver/transmitter

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_rx.h"
#include "driver/gpio.h"

#include "olt_receiver_transmitter.h"

#define TAG "OLT_TX_RX"

//#define ENABLE_TX_CARRIER

// max rx channels esp32 = 8, esp32s3 = 4, esp32c3 = 2
// for esp32 rx&tx channels shared -> rx channels = 1->tx, 7->rx
#define OLT_RX_CHANNELS_NUM 7
static const gpio_num_t olt_tx_gpio = 2; // tx channel gpio
static const gpio_num_t olt_rx_gpio[OLT_RX_CHANNELS_NUM] = {2,2,2,2,2,2,2}; // rx channels gpio, for test rx gpio = tx gpio

#ifdef CONFIG_IDF_TARGET_ESP32
#define OLT_CHANNEL_SIZE 64 // 48->esp32s3 64->esp32
#else
#define OLT_CHANNEL_SIZE 48 // 48->esp32s3 64->esp32
#endif
// define olt timing
#define OLT_RMT_CLK 1*1000*1000 // 1 MHz
#define OLT_RMT_TICK 600 // 600 mksek
static const rmt_symbol_word_t olt_rmt_start = {
    .level0 = 1,
    .duration0 = OLT_RMT_TICK*4,    // start bit = 2400 mks
    .level1 = 0,
    .duration1 = OLT_RMT_TICK,      // pause 600 msks
};
static const rmt_symbol_word_t olt_rmt_one = {
    .level0 = 1,
    .duration0 = OLT_RMT_TICK*2,    // bit one = 1200 mks 
    .level1 = 0,
    .duration1 = OLT_RMT_TICK,      // pause 600 msks
};
static const rmt_symbol_word_t olt_rmt_zero = {
    .level0 = 1,
    .duration0 = OLT_RMT_TICK,      // bit zero = 600 mks
    .level1 = 0,
    .duration1 = OLT_RMT_TICK,      // pause 600 msks
};
static const rmt_symbol_word_t olt_rmt_pause = { // pause between tx packet 6000 mksek 
    .level0 = 0,
    .duration0 = OLT_RMT_TICK*5, // pause  3000 mksek 
    .level1 = 0,
    .duration1 = OLT_RMT_TICK*5, // pause 3000 mksek            
};
static const rmt_symbol_word_t olt_rmt_stop = { //  stop rmt word
    .level0 = 0,
    .duration0 = 0, 
    .level1 = 0,
    .duration1 = 0,             
};
static const rmt_receive_config_t receive_config = { // default receiver config for all channels
    .signal_range_min_ns = 10,                   // the shortest duration
    .signal_range_max_ns = OLT_RMT_TICK*5*1000, // the longest duration 3000 mks, end packet
};
// tx rmt handle
static rmt_channel_handle_t tx_chan_handle = NULL;
static rmt_encoder_handle_t tx_encoder = NULL;
// rx rmt channels handle & receive buffers 
typedef struct rmt_rx_channel_param
{
    rmt_channel_handle_t rx_chan_handle;
    rmt_symbol_word_t olt_channel_rx_data[OLT_CHANNEL_SIZE];
} rmt_rx_channel_param_t;
static rmt_rx_channel_param_t rmt_channels_param[OLT_RX_CHANNELS_NUM];
// rx callback queue
static QueueHandle_t olt_rx_queue = NULL;

// decode data from olt_packet_t to rmt_symbol_word_t packet 
static size_t olt_tx_rmt_decode(olt_packet_t data_in, rmt_symbol_word_t *data_out)
{
    uint32_t mask = 0x80000000;
    size_t size = 0;
    // shot_long_bit define command(24-32 bit)/damage(22 bit) packet
    // byte_ext define extended info, if the content is not zero there is an extended package (32 bit)
    if(data_in.shot_packet.shot_long_bit == 0){size = 22;}
    else {size = (data_in.long_packet.byte_ext == 0) ? 24 : 32;}
    size++;

    data_out[0] = olt_rmt_start; // start bit
    int i;
    // fill rmt data olt_packet_t to rmt_symbol_word_t
    // msb first
    for(i=1;i < size ; i++)
    {
        data_out[i] = (mask & data_in.val) ?  olt_rmt_one : olt_rmt_zero;
        mask >>= 1;
    }
    data_out[i++] = olt_rmt_pause;  // add pause between tx packet, at the end of packet
    data_out[i] = olt_rmt_stop;     // stop rmt
    return i;
}
// transmit olt_packet_t data
// xTicksToWait -> portMAX_DELAY for blocking transmit, 0 for nonblocking transmit
// for nonblocking transmit use olt_tx_wait_all_done() for check transmit done
// return ESP_OK or ESP_ERR_TIMEOUT
esp_err_t olt_tx_data(olt_packet_t tx_packet,TickType_t xTicksToWait)
{
    rmt_transmit_config_t rmt_tx_config = {}; // tx config with all data = 0;
    rmt_symbol_word_t rmt_data_out[OLT_CHANNEL_SIZE]; // rmt transmit buffer
    size_t decoded_data_size = olt_tx_rmt_decode(tx_packet,rmt_data_out); // decode olt_packet_t to rmt_symbol_word_t buffer
    ESP_ERROR_CHECK(rmt_transmit(tx_chan_handle, tx_encoder, rmt_data_out, decoded_data_size*sizeof(rmt_symbol_word_t), &rmt_tx_config)); // start transmit
    return rmt_tx_wait_all_done(tx_chan_handle, xTicksToWait); // wait transmit done
}
// wait all rmt data transmitted
// return ESP_OK or ESP_ERR_TIMEOUT
esp_err_t olt_tx_wait_all_done(TickType_t xTicksToWait)
{
    return rmt_tx_wait_all_done(tx_chan_handle, xTicksToWait); 
}
// init tx channel
esp_err_t olt_tx_channel_init(void)
{
    ESP_LOGI(TAG,"Init tx channel");
    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT, // select source clock
        .gpio_num = olt_tx_gpio,        // tx gpio
        .mem_block_symbols = OLT_CHANNEL_SIZE,
        .flags.io_loop_back = 1,    // gpio output/input mode // for test only
        .resolution_hz = OLT_RMT_CLK,   // channel clk -> 1mHz
        .trans_queue_depth = 1, // set the maximum number of transactions that can pend in the background
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &tx_chan_handle));

#ifdef ENABLE_TX_CARRIER
    // carrier config
    rmt_carrier_config_t tx_carrier_cfg = {
        .duty_cycle = 0.33,                 // duty cycle 33%
        .frequency_hz = 56*1000,              // 56 KHz
        .flags.polarity_active_low = false, // carrier should be modulated to high level
    };
    // modulate carrier to TX channel
    ESP_ERROR_CHECK(rmt_apply_carrier(tx_chan_handle, &tx_carrier_cfg));
#endif

    rmt_copy_encoder_config_t tx_encoder_config = {};                       // default copy encoder config
    ESP_ERROR_CHECK(rmt_new_copy_encoder(&tx_encoder_config, &tx_encoder)); // create copy encoder
    ESP_ERROR_CHECK(rmt_enable(tx_chan_handle));                            // enable tx channel   

    return ESP_OK; // allready ESP_OK,  error detected on ESP_ERROR_CHECK
}
// disable rmt tx channel & free all resource
esp_err_t olt_tx_channel_free(void)
{
    rmt_disable(tx_chan_handle);
    rmt_del_encoder(tx_encoder);
    rmt_del_channel(tx_chan_handle);
    return ESP_OK;
}
// encode rx data from rmt_rx_done_event_data_t rmt_event_data (rmt_symbol_word_t* & received size) to uint32_t enc_data (olt_packet_t.val)
// return ESP_OK or ESP_FAIL if rmt_event_data damaged
static esp_err_t IRAM_ATTR olt_rx_encode(uint32_t *enc_data, const rmt_rx_done_event_data_t *rmt_event_data)
{
    *enc_data = 0;
    size_t size = rmt_event_data->num_symbols; // packet size
    switch(size) 
    {
        case 23:
        case 25:
        case 33:
            break; // check size, only 23/25/33 bit in packet (data+start bit)
        default:
    //        ESP_EARLY_LOGE(TAG,"ERR bit count %d out of range",size);
            return ESP_FAIL;
    }
    // chesk duration of start bit -> 2400 mks
    if ( rmt_event_data->received_symbols[0].duration0 > (OLT_RMT_TICK*4+OLT_RMT_TICK/3) || rmt_event_data->received_symbols[0].duration0 < (OLT_RMT_TICK*4-OLT_RMT_TICK/3) )
    {
    //    ESP_EARLY_LOGE(TAG,"ERR start bit duration %d out of range",buf[0].duration0);
        return ESP_FAIL; // out of range
    }
    // encode all packet, msb first
    for(int i = 1; i < 33 ; i++ )
    {
        *enc_data <<= 1;
        if(i >= size){ continue;}  // last bits zero
        if(rmt_event_data->received_symbols[i].duration0 < (OLT_RMT_TICK*2+OLT_RMT_TICK/3) && rmt_event_data->received_symbols[i].duration0 > (OLT_RMT_TICK*2-OLT_RMT_TICK/3) )
            {*enc_data |= 0x1;  continue;} // one
        if(rmt_event_data->received_symbols[i].duration0 < (OLT_RMT_TICK*1+OLT_RMT_TICK/3) && rmt_event_data->received_symbols[i].duration0 > (OLT_RMT_TICK*1-OLT_RMT_TICK/3) )
            { continue;} // zero
    //    ESP_EARLY_LOGE(TAG,"ERR bits %d duration %d out of range",i,buf[i].duration0);
        return ESP_FAIL; // out of range
    }
    return ESP_OK;
}
// rmt callback, call when rmt packet received
// user data -> channel number from olt_rx_gpio[OLT_RX_CHANNELS_NUM]
// encode data, restart receive, send encoded data to receive queue
// already return ESP_OK, error detected ESP_ERROR_CHECK
static bool IRAM_ATTR rmt_rx_done_callback(rmt_channel_handle_t channel_handle, const rmt_rx_done_event_data_t *edata, void *user_data)
{
    BaseType_t high_task_wakeup = pdFALSE;
    olt_rx_data_t q_data; // encoded data
    q_data.channel = (uint32_t)user_data; // received channel
    esp_err_t err = olt_rx_encode(&q_data.data.val, edata); // encode
    // restart channel receive
    rmt_receive(rmt_channels_param[q_data.channel].rx_chan_handle, rmt_channels_param[q_data.channel].olt_channel_rx_data, sizeof(rmt_channels_param[q_data.channel].olt_channel_rx_data), &receive_config);
    // if the packet is encoded without errors, we send it to the queue
    if(err == ESP_OK){
        xQueueSendFromISR(olt_rx_queue, &q_data, &high_task_wakeup);
    }
    // return whether any task is woken up
    return high_task_wakeup == pdTRUE;
}
// init all rx channels, create received queue, start first receive
// the number of channels and connection to the gpio is determined olt_rx_gpio[OLT_RX_CHANNELS_NUM]
esp_err_t olt_rx_channels_init()
{
    ESP_LOGI(TAG, "Init all rx channels");
    rmt_rx_channel_config_t rx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,         // select source clock
        .resolution_hz = OLT_RMT_CLK,           // tick resolution, 
        .mem_block_symbols = OLT_CHANNEL_SIZE,  // memory block size, 48/64
        .flags.invert_in = false,               // do not invert input signal
        .flags.with_dma = false,                // do not need DMA backend
        .flags.io_loop_back = 1,
    };
#ifdef ENABLE_RX__CARRIER
    rmt_carrier_config_t rx_carrier_cfg = {
        .duty_cycle = 0.33,                 // duty cycle 33%
        .frequency_hz = 56000,              // 25 KHz carrier, should be smaller than the transmitter's carrier frequency
        .flags.polarity_active_low = false, // the carrier is modulated to high level
    };
#endif
    // demodulate carrier from RX channel
    rmt_rx_event_callbacks_t cbs = {
        .on_recv_done = rmt_rx_done_callback,
    };
    olt_rx_queue = xQueueCreate(10, sizeof(olt_rx_data_t));
    // init all received channels
    for(int ch = 0;ch < OLT_RX_CHANNELS_NUM;ch++){
        rx_chan_config.gpio_num = olt_rx_gpio[ch];                // GPIO number
        ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_chan_config, &(rmt_channels_param[ch].rx_chan_handle)));
#ifdef ENABLE_RX__CARRIER
        ESP_ERROR_CHECK(rmt_apply_carrier(rmt_channels_param[ch].rx_chan_handle, &rx_carrier_cfg));
#endif
        ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rmt_channels_param[ch].rx_chan_handle, &cbs, (void*)ch));
        ESP_ERROR_CHECK(rmt_enable(rmt_channels_param[ch].rx_chan_handle));
        ESP_ERROR_CHECK(rmt_receive(rmt_channels_param[ch].rx_chan_handle, rmt_channels_param[ch].olt_channel_rx_data, sizeof(rmt_channels_param[ch].olt_channel_rx_data), &receive_config));
    }
    return ESP_OK;
}
// disable all rmt rx channel & free all resource
esp_err_t olt_rx_channels_free(void)
{
    rmt_rx_event_callbacks_t cbs = {
        .on_recv_done = NULL,
    };

    for(int ch = 0;ch < OLT_RX_CHANNELS_NUM;ch++){
        rmt_disable(rmt_channels_param[ch].rx_chan_handle);
        rmt_rx_register_event_callbacks(rmt_channels_param[ch].rx_chan_handle, &cbs, NULL);
        rmt_del_channel(rmt_channels_param[ch].rx_chan_handle);
    }
    vQueueDelete(olt_rx_queue);
    return ESP_OK;
}
// receive olt_packet_t data olt_rx_data_t *olt_data
// xTicksToWait -> portMAX_DELAY for blocking receive, 0 for nonblocking transmit
// return ESP_OK or ESP_ERR_TIMEOUT
esp_err_t olt_rx_data(olt_rx_data_t *olt_data,TickType_t xTicksToWait)
{
    if (xQueueReceive(olt_rx_queue, olt_data, xTicksToWait) == pdFALSE)
    {
        ESP_LOGE(TAG, "RMT Recive timeout ");
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

