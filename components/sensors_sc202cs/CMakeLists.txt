idf_component_register(
    SRCS 
        "sc202cs.c"
    INCLUDE_DIRS 
        "."
    REQUIRES
        esp_driver_i2c
)

target_link_libraries(${COMPONENT_LIB} INTERFACE "-u sc202cs_detect")
