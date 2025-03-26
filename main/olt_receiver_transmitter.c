/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_rx.h"


#include "olt_receiver_transmitter.h"


#define TAG "OLT_TX_RX"


#define OLT_TX_PIN 2
#define OLT_RX_PIN 4

#define OLT_RMT_CLK 1*1000*1000 // 1 MHz
#define OLT_RMT_TICK 600 // 600 mksek
#define OLT_CHANNEL_SIZE 48 // 48->esp32s3 64->esp32

static const rmt_symbol_word_t olt_rmt_start = {
    .level0 = 1,
    .duration0 = OLT_RMT_TICK*4,
    .level1 = 0,
    .duration1 = OLT_RMT_TICK,
};
static const rmt_symbol_word_t olt_rmt_one = {
    .level0 = 1,
    .duration0 = OLT_RMT_TICK*2,
    .level1 = 0,
    .duration1 = OLT_RMT_TICK,
};
static const rmt_symbol_word_t olt_rmt_zero = {
    .level0 = 1,
    .duration0 = OLT_RMT_TICK,
    .level1 = 0,
    .duration1 = OLT_RMT_TICK,
};
static const rmt_symbol_word_t olt_rmt_stop = {
    .level0 = 0,
    .duration0 = 0,
    .level1 = 0,
    .duration1 = 0,
};

static QueueHandle_t olt_tx_queue = 0;
static QueueHandle_t olt_rx_queue = 0;

static TaskHandle_t olt_tx_task_handle = 0;

static rmt_channel_handle_t tx_chan_handle = NULL;
static rmt_encoder_handle_t tx_encoder = NULL;
static rmt_channel_handle_t rx_chan = NULL;


static size_t olt_tx_rmt_decode(olt_packet_t data_in, rmt_symbol_word_t *data_out)
{
    uint32_t mask = 0x80000000;
    size_t size = 0;
    if(data_in.shot_packet.shot_long_bit == 0){size = 22;}
    else {size = (data_in.long_packet.byte_ext == 0) ? 24 : 32;}
    size++;

    data_out[0] = olt_rmt_start;
    int i;
    for(i=1;i < size ; i++)
    {
        data_out[i] = (mask & data_in.val) ?  olt_rmt_one : olt_rmt_zero;
        mask >>= 1;
    }
    data_out[i] = olt_rmt_stop;
    return i;
}

static void olt_tx_task(void* p)
{
    ESP_LOGI(TAG,"Create tx task");
    rmt_transmit_config_t rmt_tx_config = {0};
    //rmt_tx_config.loop_count = 0;
    rmt_symbol_word_t rmt_data_out[OLT_CHANNEL_SIZE];
    olt_packet_t receive_packet = {0};
    while(1){
    xQueueReceive(olt_tx_queue,&receive_packet,portMAX_DELAY);
    size_t decoded_data_size = olt_tx_rmt_decode(receive_packet,rmt_data_out);
    ESP_ERROR_CHECK(rmt_transmit(tx_chan_handle, tx_encoder, rmt_data_out, decoded_data_size*sizeof(rmt_symbol_word_t), &rmt_tx_config));
    rmt_tx_wait_all_done(tx_chan_handle, portMAX_DELAY);
    }
}

esp_err_t olt_tx_data(olt_packet_t data)
{
    const void *tx_packet = &data;
    xQueueSend(olt_tx_queue, tx_packet, portMAX_DELAY);
    return ESP_OK;
}

esp_err_t olt_tx_channel_init(void)
{
    ESP_LOGI(TAG,"Init tx channel");
    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT, // select source clock
        .gpio_num = OLT_TX_PIN,
        .mem_block_symbols = OLT_CHANNEL_SIZE,
        .flags.io_loop_back = 1,    // gpio output/input mode // for test only
        .resolution_hz = OLT_RMT_CLK,
        .trans_queue_depth = 5, // set the maximum number of transactions that can pend in the background
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &tx_chan_handle));

#if 0
    rmt_carrier_config_t tx_carrier_cfg = {
        .duty_cycle = 0.33,                 // duty cycle 33%
        .frequency_hz = 56*1000,              // 56 KHz
        .flags.polarity_active_low = false, // carrier should be modulated to high level
    };
    // modulate carrier to TX channel
    ESP_ERROR_CHECK(rmt_apply_carrier(tx_chan_handle, &tx_carrier_cfg));
#endif

    rmt_copy_encoder_config_t tx_encoder_config = {0};
    ESP_ERROR_CHECK(rmt_new_copy_encoder(&tx_encoder_config, &tx_encoder));
    ESP_ERROR_CHECK(rmt_enable(tx_chan_handle));

    olt_tx_queue = xQueueCreate(5,sizeof(olt_packet_t));

    xTaskCreate(olt_tx_task,"olt_tx_task",2048*4,NULL,5,&olt_tx_task_handle);

    return ESP_OK;
}
esp_err_t olt_tx_channel_deinit(void)
{
    rmt_disable(tx_chan_handle);
    rmt_del_encoder(tx_encoder);
    rmt_del_channel(tx_chan_handle);
    vQueueDelete(olt_tx_queue);
    vTaskDelete(olt_tx_task_handle);
    return ESP_OK;
}

#if 0
static bool rmt_rx_done_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_data)
{
    BaseType_t high_task_wakeup = pdFALSE;
    QueueHandle_t receive_queue = (QueueHandle_t)user_data;
    // send the received RMT symbols to the parser task
    xQueueSendFromISR(receive_queue, edata, &high_task_wakeup);
    // return whether any task is woken up
    return high_task_wakeup == pdTRUE;
}
esp_err_t olt_rx_channels_init()
{
    ESP_LOGI(TAG, "Start receive");

    rmt_rx_channel_config_t rx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,        // select source clock
        .resolution_hz = OLT_RMT_CLK, // tick resolution, 
        .mem_block_symbols = OLT_CHANNEL_SIZE,  // memory block size, 48/64
        .gpio_num = OLT_RX_PIN,                 // GPIO number
        .flags.invert_in = false,              // do not invert input signal
        .flags.with_dma = false,               // do not need DMA backend
        .flags.io_loop_back = 1,
    };
    ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_chan_config, &rx_chan));
    olt_rx_queue = xQueueCreate(10, sizeof(rmt_rx_done_event_data_t));
    rmt_rx_event_callbacks_t cbs = {
        .on_recv_done = rmt_rx_done_callback,
    };
    ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rx_chan, &cbs, olt_rx_queue));

    ESP_ERROR_CHECK(rmt_enable(rx_chan));
    rmt_receive_config_t receive_config = {
        .signal_range_min_ns = 10,                       // the shortest duration
        .signal_range_max_ns = OLT_RMT_TICK*8*1000*1000, // the longest duration 4800 mks
    };

    ESP_ERROR_CHECK(rmt_receive(rx_chan, rmt_tools_cfg.rmt_data_in, sizeof(rmt_tools_cfg.rmt_data_in), &receive_config));
    rmt_rx_done_event_data_t rx_data;

    if (xQueueReceive(receive_queue, &rx_data, 10000 / portTICK_PERIOD_MS) == pdFALSE)
    {
        ESP_LOGI(TAG, "RMT Recrive timeout 10 sec");
    }
    else
    {
        ESP_LOGI(TAG, "RMT Recrive data");
    }

}
esp_err_t olt_rx_channels_deinit(void)
{
    rmt_disable(rx_chan);
    rmt_rx_event_callbacks_t cbs = {
        .on_recv_done = NULL,
    };
    rmt_rx_register_event_callbacks(rx_chan, &cbs, receive_queue);
    rmt_del_channel(rx_chan);
    vQueueDelete(olt_rx_queue);
    ESP_LOGI(TAG, "Receive OK");

}
esp_err_t olt_rx_data(olt_rx_data_t data,TickType_t xTicksToWait)
{

}
#endif

#include "logic_analyzer_ws_server.h"
int app_main()
{
    logic_analyzer_ws_server();
//    olt_rx_channels_init();
    olt_tx_channel_init();
    while(1)
    {
        olt_packet_t tx_data;
        tx_data.val = 0x80000010;
        olt_tx_data(tx_data);
//        olt_rx_data(rx_data,portMAX_DELAY);
        vTaskDelay(10);
    }
}


