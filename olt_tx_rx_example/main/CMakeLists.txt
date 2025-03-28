idf_component_register(SRCS "olt_tx_rx_example.c" 
                       INCLUDE_DIRS "."
                       REQUIRES driver olt_receiver_transmitter logic_analyzer)
