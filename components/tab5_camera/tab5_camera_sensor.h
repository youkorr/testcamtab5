#pragma once

#ifdef USE_ESP32_VARIANT_ESP32P4

// Inclure les vrais headers en copiant leur contenu
// Ces fichiers viennent de esp_cam_sensor_esphome

extern "C" {

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

// ============================================================================
// esp_sccb_types.h
// ============================================================================
typedef struct esp_sccb_io_t* esp_sccb_io_handle_t;

// ============================================================================
// esp_cam_sensor_types.h (extrait)
// ============================================================================
#ifndef ARRAY_SIZE
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof(arr[0]))
#endif

#define ESP_CAM_SENSOR_MIPI_CSI_PORT 1

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
    uint32_t mipi_clk;
    uint32_t hs_settle;
    uint32_t lane_num;
    bool line_sync_en;
} esp_cam_sensor_mipi_info_t;

typedef struct {
    esp_sccb_io_handle_t sccb_handle;
    int8_t reset_pin;
    int8_t pwdn_pin;
    int8_t xclk_pin;
    int32_t xclk_freq_hz;
    esp_cam_sensor_port_t sensor_port;
} esp_cam_sensor_config_t;

struct _esp_cam_sensor_ops;

// Structure d'opérations du capteur
typedef struct _esp_cam_sensor_ops {
    int (*query_para_desc)(struct esp_cam_sensor_device_t *dev, void *qdesc);
    int (*get_para_value)(struct esp_cam_sensor_device_t *dev, uint32_t id, void *arg, size_t size);
    int (*set_para_value)(struct esp_cam_sensor_device_t *dev, uint32_t id, const void *arg, size_t size);
    int (*query_support_formats)(struct esp_cam_sensor_device_t *dev, void *parry);
    int (*query_support_capability)(struct esp_cam_sensor_device_t *dev, void *arg);
    int (*set_format)(struct esp_cam_sensor_device_t *dev, const void *format);
    int (*get_format)(struct esp_cam_sensor_device_t *dev, void *format);
    int (*priv_ioctl)(struct esp_cam_sensor_device_t *dev, uint32_t cmd, void *arg);
    int (*del)(struct esp_cam_sensor_device_t *dev);
} esp_cam_sensor_ops_t;

// Structure complète (pas typedef pour éviter les conflits)
struct esp_cam_sensor_device_t {
    char *name;
    esp_sccb_io_handle_t sccb_handle;
    int8_t xclk_pin;
    int8_t reset_pin;
    int8_t pwdn_pin;
    esp_cam_sensor_port_t sensor_port;
    const void *cur_format;
    esp_cam_sensor_id_t id;
    uint8_t stream_status;
    const esp_cam_sensor_ops_t *ops;
    void *priv;
};

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
    esp_cam_sensor_mipi_info_t mipi_info;
    void *reserved;
} esp_cam_sensor_format_t;

// ============================================================================
// esp_sccb_i2c.h
// ============================================================================
typedef struct {
    i2c_addr_bit_len_t dev_addr_length;
    uint16_t device_address;
    uint32_t scl_speed_hz;
    uint32_t addr_bits_width;
    uint32_t val_bits_width;
} sccb_i2c_config_t;

esp_err_t sccb_new_i2c_io(i2c_master_bus_handle_t bus_handle, 
                          const sccb_i2c_config_t *config,
                          esp_sccb_io_handle_t *io_handle);

// ============================================================================
// esp_cam_sensor_detect.h
// ============================================================================
extern esp_cam_sensor_detect_fn_t __esp_cam_sensor_detect_fn_array_start;
extern esp_cam_sensor_detect_fn_t __esp_cam_sensor_detect_fn_array_end;

// ============================================================================
// esp_cam_sensor.h (fonctions)
// ============================================================================
#define ESP_CAM_SENSOR_IOC_S_STREAM 0x04000004

esp_err_t esp_cam_sensor_get_format(esp_cam_sensor_device_t *dev, 
                                    esp_cam_sensor_format_t *format);

esp_err_t esp_cam_sensor_ioctl(esp_cam_sensor_device_t *dev, 
                               uint32_t cmd, 
                               void *arg);

} // extern "C"

#endif // USE_ESP32_VARIANT_ESP32P4
