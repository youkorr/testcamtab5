set(srcs "src/esp_cam_sensor.c")

set(include_dirs "include")
set(requires "driver" "esp_sccb_intf")


if(CONFIG_CAMERA_SC202CS)
    list(APPEND srcs "esp_cam_sensor/sensors/sc202cs/sc202cs.c")
    list(APPEND include_dirs "esp_cam_sensorsensors/sc202cs/include")
    list(APPEND priv_include_dirs "esp_cam_sensor/sensors/sc202cs/private_include")
endif()



idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS ${include_dirs}
                       PRIV_INCLUDE_DIRS ${priv_include_dirs}
                       REQUIRES ${requires}
                       LDFRAGMENTS linker.lf)


if(CONFIG_CAMERA_SC202CS_AUTO_DETECT)
    target_link_libraries(${COMPONENT_LIB} INTERFACE "-u sc202cs_detect")

endif()

include(package_manager)
cu_pkg_define_version(${CMAKE_CURRENT_LIST_DIR})
