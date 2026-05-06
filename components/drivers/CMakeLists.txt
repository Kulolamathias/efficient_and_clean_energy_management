idf_component_register(SRCS "src/gsm.c"
                       INCLUDE_DIRS "include"
                       REQUIRES driver freertos esp_timer)