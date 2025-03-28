#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_rx.h"
#include "driver/gpio.h"

#include "olt_receiver_transmitter.h"

#define TAG "OLT_TX_RX_EXAMPLE"

// test transmit
void test_tx(void *p){
    olt_packet_t tx_data;
    while(1){
        tx_data.val = 0x81000000;
        olt_tx_data(tx_data,portMAX_DELAY);
        tx_data.val = 0x11000000;
        olt_tx_data(tx_data,portMAX_DELAY);
        tx_data.val = 0x81000001;
        olt_tx_data(tx_data,portMAX_DELAY);
        vTaskDelay(10);
    }
}
// test receive
void test_rx(void *p){
    olt_rx_data_t rx_data;
    while(1){
        olt_rx_data(&rx_data,portMAX_DELAY);
        ESP_LOGI(TAG,"Received data %lx, on channel %ld",rx_data.data.val,rx_data.channel);
    }
}


#include "logic_analyzer_ws_server.h"
int app_main()
{
    logic_analyzer_ws_server();

    olt_rx_channels_init();
    olt_tx_channel_init();

    xTaskCreate(test_tx,"test_tx",2048*2,NULL,5,NULL);
    xTaskCreate(test_rx,"test_rx",2048*2,NULL,5,NULL);
    
    while(1)
    {
        vTaskDelay(100);
    }
    return ESP_OK;
}

