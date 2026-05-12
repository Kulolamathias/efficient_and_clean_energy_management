    idf_component_register(
        SRCS 
            "src/gsm_sim800.c"

        INCLUDE_DIRS
            "include"

        REQUIRES
            esp_timer
            driver
    )