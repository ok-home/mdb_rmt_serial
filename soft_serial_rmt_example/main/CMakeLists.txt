idf_component_register(SRCS "soft_serial_rmt_example_main.c"
                    REQUIRES 
#                        logic_analyzer 
                        driver 
                        uart_rmt
                    INCLUDE_DIRS ".")
