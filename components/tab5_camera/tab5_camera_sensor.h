#pragma once

#ifdef USE_ESP32_VARIANT_ESP32P4

extern "C" {
#include "driver/i2c_master.h"
#include "esp_err.h"

// Définitions minimales tirées de esp_cam_sensor
#define ESP_CAM_SENSOR_MIPI_CSI 1

#define ESP_CAM_SENSOR_IOC_S_STREAM 0x04000004

typedef struct esp_sccb_io_t* esp_sccb_io_handle_t;

typedef enum {
    I2C_ADDR_BIT_LEN_7 = 0,
    I2C_ADDR_BIT_LEN_10 = 1,
} i2c_addr_bit_len_t;

typedef struct {
    i2c_addr_bit_len_t dev_addr_length;
    uint16_t device_address;
    uint32_t scl_speed_hz;
    uint32_t addr_bits_width;
    uint32_t val_bits_width;
} sccb_i2c_config_t;

typedef struct {
    uint8_t midh;
    uint8_t midl;
    uint16_t pid;
    uint8_t ver;
} esp_cam_sensor_id_t;

typedef enum {
    ESP_CAM_SENSOR_DVP,
    ESP_CAM_SENSOR_MIPI_CSI,
} esp_cam_sensor_port_t;

typedef struct {
    esp_sccb_io_handle_t sccb_handle;
    int8_t reset_pin;
    int8_t pwdn_pin;
    int8_t xclk_pin;
    int32_t xclk_freq_hz;
    esp_cam_sensor_port_t sensor_port;
} esp_cam_sensor_config_t;

struct _esp_cam_sensor_ops;

typedef struct {
    char *name;
    esp_sccb_io_handle_t sccb_handle;
    int8_t xclk_pin;
    int8_t reset_pin;
    int8_t pwdn_pin;
    esp_cam_sensor_port_t sensor_port;
    const void *cur_format;
    esp_cam_sensor_id_t id;
    uint8_t stream_status;
    const struct _esp_cam_sensor_ops *ops;
    void *priv;
} esp_cam_sensor_device_t;

typedef struct {
    union {
        esp_cam_sensor_device_t *(*detect)(void *);
        esp_cam_sensor_device_t *(*fn)(void *);
    };
    esp_cam_sensor_port_t port;
    uint16_t sccb_addr;
} esp_cam_sensor_detect_fn_t;

typedef struct {
    const char *name;
    uint32_t format;
    esp_cam_sensor_port_t port;
    int xclk;
    uint16_t width;
    uint16_t height;
    const void *regs;
    int regs_size;
    uint8_t fps;
    const void *isp_info;
    void *mipi_info;
    void *reserved;
} esp_cam_sensor_format_t;

// Fonctions externes
esp_err_t sccb_new_i2c_io(i2c_master_bus_handle_t bus_handle, 
                          const sccb_i2c_config_t *config,
                          esp_sccb_io_handle_t *io_handle);

esp_err_t esp_cam_sensor_get_format(esp_cam_sensor_device_t *dev, 
                                    esp_cam_sensor_format_t *format);

esp_err_t esp_cam_sensor_ioctl(esp_cam_sensor_device_t *dev, 
                               uint32_t cmd, 
                               void *arg);

// Symboles externes pour la détection automatique
extern esp_cam_sensor_detect_fn_t __esp_cam_sensor_detect_fn_array_start;
extern esp_cam_sensor_detect_fn_t __esp_cam_sensor_detect_fn_array_end;

} // extern "C"

#endif // USE_ESP32_VARIANT_ESP32P4
