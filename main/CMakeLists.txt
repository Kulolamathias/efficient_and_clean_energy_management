idf_component_register(
    SRCS 
        "main.c"
                       
    INCLUDE_DIRS 
        "."

    REQUIRES
        electrical_param_sensor
        gsm_sim800
        # esp_pzem
        lcd_i2c
        driver
)
