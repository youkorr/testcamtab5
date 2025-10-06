#include "tab5_camera.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"

#ifdef USE_ESP32_VARIANT_ESP32P4

// ============================================================================
// HEADERS ESP-IDF NÉCESSAIRES
// ============================================================================

extern "C" {
#include "driver/i2c_master.h"
}

// ============================================================================
// DÉFINITIONS COMPLÈTES DES STRUCTURES CAMERA SENSOR
// ============================================================================

extern "C" {

// Type pour SCCB handle
typedef struct esp_sccb_io_t* esp_sccb_io_handle_t;

// Structure SCCB IO config
typedef struct {
    i2c_addr_bit_len_t dev_addr_length;
    uint16_t device_address;
    uint32_t scl_speed_hz;
    uint32_t addr_bits_width;
    uint32_t val_bits_width;
} sccb_i2c_config_t;

// Structure pour l'ID du capteur
typedef struct {
    uint8_t midh;
    uint8_t midl;
    uint16_t pid;
    uint8_t ver;
} esp_cam_sensor_id_t;

// Type de port du capteur
typedef enum {
    ESP_CAM_SENSOR_DVP,
    ESP_CAM_SENSOR_MIPI_CSI,
} esp_cam_sensor_port_t;

// Forward declaration pour les opérations
struct esp_cam_sensor_device_t;
typedef struct esp_cam_sensor_ops_t {
    int (*set_format)(struct esp_cam_sensor_device_t *dev, const void *format);
    int (*get_format)(struct esp_cam_sensor_device_t *dev, void *format);
    int (*priv_ioctl)(struct esp_cam_sensor_device_t *dev, uint32_t cmd, void *arg);
    int (*del)(struct esp_cam_sensor_device_t *dev);
} esp_cam_sensor_ops_t;

// Structure principale du device
typedef struct esp_cam_sensor_device_t {
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
} esp_cam_sensor_device_t;

// Configuration du capteur
typedef struct {
    esp_sccb_io_handle_t sccb_handle;
    int8_t reset_pin;
    int8_t pwdn_pin;
    int8_t xclk_pin;
    int32_t xclk_freq_hz;
    esp_cam_sensor_port_t sensor_port;
} esp_cam_sensor_config_t;

// Type pour detect function
typedef struct {
    union {
        esp_cam_sensor_device_t *(*detect)(void *);
        esp_cam_sensor_device_t *(*fn)(void *);
    };
    esp_cam_sensor_port_t port;
    uint16_t sccb_addr;
} esp_cam_sensor_detect_fn_t;

// Structure pour le format du capteur
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
    void *reserved;
} esp_cam_sensor_format_t;

// ============================================================================
// CODE COMPLET DU DRIVER SC202CS INTÉGRÉ
// ============================================================================

#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"

// Types SC202CS
typedef struct {
    uint16_t reg;
    uint8_t val;
} sc202cs_reginfo_t;

// Registres SC202CS
#define SC202CS_REG_END   0xffff
#define SC202CS_REG_DELAY 0xfffe
#define SC202CS_REG_SENSOR_ID_H 0x3107
#define SC202CS_REG_SENSOR_ID_L 0x3108
#define SC202CS_REG_SLEEP_MODE  0x0100
#define SC202CS_REG_DIG_COARSE_GAIN 0x3e06
#define SC202CS_REG_DIG_FINE_GAIN   0x3e07
#define SC202CS_REG_ANG_GAIN        0x3e09
#define SC202CS_REG_SHUTTER_TIME_H 0x3e00
#define SC202CS_REG_SHUTTER_TIME_M 0x3e01
#define SC202CS_REG_SHUTTER_TIME_L 0x3e02
#define SC202CS_REG_GROUP_HOLD 0x3812
#define SC202CS_REG_TOTAL_WIDTH_H  0x320c
#define SC202CS_REG_TOTAL_WIDTH_L  0x320d
#define SC202CS_REG_TOTAL_HEIGHT_H 0x320e
#define SC202CS_REG_TOTAL_HEIGHT_L 0x320f
#define SC202CS_REG_OUT_WIDTH_H  0x3208
#define SC202CS_REG_OUT_WIDTH_L  0x3209
#define SC202CS_REG_OUT_HEIGHT_H 0x320a
#define SC202CS_REG_OUT_HEIGHT_L 0x320b
#define SC202CS_REG_OUT_START_PIXEL_H 0x3210
#define SC202CS_REG_OUT_START_PIXEL_L 0x3211
#define SC202CS_REG_OUT_START_LINE_H  0x3212
#define SC202CS_REG_OUT_START_LINE_L  0x3213
#define SC202CS_REG_FLIP_MIRROR 0x3221

#define SC202CS_PID         0xeb52
#define SC202CS_SENSOR_NAME "SC202CS"
#define SC202CS_SCCB_ADDR 0x36

#ifndef portTICK_RATE_MS
#define portTICK_RATE_MS portTICK_PERIOD_MS
#endif
#define delay_ms(ms) vTaskDelay((ms > portTICK_PERIOD_MS ? ms / portTICK_PERIOD_MS : 1))

// Configuration 1280x720 RAW8
// cleaned_0x18_FT_SC2356_24Minput_576Mbps_1lane_8bit_1280x720_30fps
static const sc202cs_reginfo_t init_reglist_1280x720_30fps[] = {
    {0x0103, 0x01},
    {SC202CS_REG_SLEEP_MODE, 0x00},
    {0x36e9, 0x80},
    {0x36ea, 0x06},
    {0x36eb, 0x0a},
    {0x36ec, 0x01},
    {0x36ed, 0x18},
    {0x36e9, 0x24},
    {0x301f, 0x18},
    {0x3031, 0x08},
    {0x3037, 0x00},
    {0x3200, 0x00},
    {0x3201, 0xa0},
    {0x3202, 0x00},
    {0x3203, 0xf0},
    {0x3204, 0x07},
    {0x3205, 0xa7},
    {0x3206, 0x03},
    {0x3207, 0xc7},
    {0x3208, 0x05},
    {0x3209, 0x00},
    {0x320a, 0x02},
    {0x320b, 0xd0},
    {0x3210, 0x00},
    {0x3211, 0x04},
    {0x3212, 0x00},
    {0x3213, 0x04},
    {0x3301, 0xff},
    {0x3304, 0x68},
    {0x3306, 0x40},
    {0x3308, 0x08},
    {0x3309, 0xa8},
    {0x330b, 0xd0},
    {0x330c, 0x18},
    {0x330d, 0xff},
    {0x330e, 0x20},
    {0x331e, 0x59},
    {0x331f, 0x99},
    {0x3333, 0x10},
    {0x335e, 0x06},
    {0x335f, 0x08},
    {0x3364, 0x1f},
    {0x337c, 0x02},
    {0x337d, 0x00},  //{0x337d, 0x0a},
    {0x338f, 0xa0},
    {0x3390, 0x01},
    {0x3391, 0x03},
    {0x3392, 0x1f},
    {0x3393, 0xff},
    {0x3394, 0xff},
    {0x3395, 0xff},
    {0x33a2, 0x04},
    {0x33ad, 0x0c},
    {0x33b1, 0x20},
    {0x33b3, 0x38},
    {0x33f9, 0x40},
    {0x33fb, 0x48},
    {0x33fc, 0x0f},
    {0x33fd, 0x1f},
    {0x349f, 0x03},
    {0x34a6, 0x03},
    {0x34a7, 0x1f},
    {0x34a8, 0x38},
    {0x34a9, 0x30},
    {0x34ab, 0xd0},
    {0x34ad, 0xd8},
    {0x34f8, 0x1f},
    {0x34f9, 0x20},
    {0x3630, 0xa0},
    {0x3631, 0x92},
    {0x3632, 0x64},
    {0x3633, 0x43},
    {0x3637, 0x49},
    {0x363a, 0x85},
    {0x363c, 0x0f},
    {0x3650, 0x31},
    {0x3670, 0x0d},
    {0x3674, 0xc0},
    {0x3675, 0xa0},
    {0x3676, 0xa0},
    {0x3677, 0x92},
    {0x3678, 0x96},
    {0x3679, 0x9a},
    {0x367c, 0x03},
    {0x367d, 0x0f},
    {0x367e, 0x01},
    {0x367f, 0x0f},
    {0x3698, 0x83},
    {0x3699, 0x86},
    {0x369a, 0x8c},
    {0x369b, 0x94},
    {0x36a2, 0x01},
    {0x36a3, 0x03},
    {0x36a4, 0x07},
    {0x36ae, 0x0f},
    {0x36af, 0x1f},
    {0x36bd, 0x22},
    {0x36be, 0x22},
    {0x36bf, 0x22},
    {0x36d0, 0x01},
    {0x370f, 0x02},
    {0x3721, 0x6c},
    {0x3722, 0x8d},
    {0x3725, 0xc5},
    {0x3727, 0x14},
    {0x3728, 0x04},
    {0x37b7, 0x04},
    {0x37b8, 0x04},
    {0x37b9, 0x06},
    {0x37bd, 0x07},
    {0x37be, 0x0f},
    {0x3901, 0x02},
    {0x3903, 0x40},
    {0x3905, 0x8d},
    {0x3907, 0x00},
    {0x3908, 0x41},
    {0x391f, 0x41},
    {0x3933, 0x80},
    {0x3934, 0x02},
    {0x3937, 0x6f},
    {0x393a, 0x01},
    {0x393d, 0x01},
    {0x393e, 0xc0},
    {0x39dd, 0x41},
    {0x3e00, 0x00},
    {0x3e01, 0x4d},
    {0x3e02, 0xc0},
    {0x3e09, 0x03},
    {0x4509, 0x28},
    {0x450d, 0x61},
    {SC202CS_REG_END, 0x00},
};

// Configuration 640x480 RAW8 
static const sc202cs_reginfo_t init_reglist_640x480_30fps[] = {
    {0x0103, 0x01},          {SC202CS_REG_SLEEP_MODE, 0x00},
    {0x36e9, 0x80},          {0x36ea, 0x06},
    {0x36eb, 0x0a},          {0x36ec, 0x01},
    {0x36ed, 0x18},          {0x36e9, 0x24},
    {0x301f, 0x18},          {0x3031, 0x08},
    {0x3037, 0x00},          {0x3301, 0xff},
    {0x3304, 0x68},          {0x3306, 0x40},
    {0x3308, 0x08},          {0x3309, 0xa8},
    {0x330b, 0xd0},          {0x330c, 0x18},
    {0x330d, 0xff},          {0x330e, 0x20},
    {0x331e, 0x59},          {0x331f, 0x99},
    {0x3333, 0x10},          {0x335e, 0x06},
    {0x335f, 0x08},          {0x3364, 0x1f},
    {0x337c, 0x02},          {0x337d, 0x0a},
    {0x338f, 0xa0},          {0x3390, 0x01},
    {0x3391, 0x03},          {0x3392, 0x1f},
    {0x3393, 0xff},          {0x3394, 0xff},
    {0x3395, 0xff},          {0x33a2, 0x04},
    {0x33ad, 0x0c},          {0x33b1, 0x20},
    {0x33b3, 0x38},          {0x33f9, 0x40},
    {0x33fb, 0x48},          {0x33fc, 0x0f},
    {0x33fd, 0x1f},          {0x349f, 0x03},
    {0x34a6, 0x03},          {0x34a7, 0x1f},
    {0x34a8, 0x38},          {0x34a9, 0x30},
    {0x34ab, 0xd0},          {0x34ad, 0xd8},
    {0x34f8, 0x1f},          {0x34f9, 0x20},
    {0x3630, 0xa0},          {0x3631, 0x92},
    {0x3632, 0x64},          {0x3633, 0x43},
    {0x3637, 0x49},          {0x363a, 0x85},
    {0x363c, 0x0f},          {0x3650, 0x31},
    {0x3670, 0x0d},          {0x3674, 0xc0},
    {0x3675, 0xa0},          {0x3676, 0xa0},
    {0x3677, 0x92},          {0x3678, 0x96},
    {0x3679, 0x9a},          {0x367c, 0x03},
    {0x367d, 0x0f},          {0x367e, 0x01},
    {0x367f, 0x0f},          {0x3698, 0x83},
    {0x3699, 0x86},          {0x369a, 0x8c},
    {0x369b, 0x94},          {0x36a2, 0x01},
    {0x36a3, 0x03},          {0x36a4, 0x07},
    {0x36ae, 0x0f},          {0x36af, 0x1f},
    {0x36bd, 0x22},          {0x36be, 0x22},
    {0x36bf, 0x22},          {0x36d0, 0x01},
    {0x370f, 0x02},          {0x3721, 0x6c},
    {0x3722, 0x8d},          {0x3725, 0xc5},
    {0x3727, 0x14},          {0x3728, 0x04},
    {0x37b7, 0x04},          {0x37b8, 0x04},
    {0x37b9, 0x06},          {0x37bd, 0x07},
    {0x37be, 0x0f},          {0x3901, 0x02},
    {0x3903, 0x40},          {0x3905, 0x8d},
    {0x3907, 0x00},          {0x3908, 0x41},
    {0x391f, 0x41},          {0x3933, 0x80},
    {0x3934, 0x02},          {0x3937, 0x6f},
    {0x393a, 0x01},          {0x393d, 0x01},
    {0x393e, 0xc0},          {0x39dd, 0x41},
    {0x3e00, 0x00},          {0x3e01, 0x4d},
    {0x3e02, 0xc0},          {0x3e09, 0x00},
    {0x4509, 0x28},          {0x450d, 0x61},
    // Configuration directe sans windowing - résolution native
    {0x3200, 0x00},          {0x3201, 0x00},  // Start X = 0
    {0x3202, 0x00},          {0x3203, 0x00},  // Start Y = 0  
    {0x3204, 0x05},          {0x3205, 0x07},  // End X = 1287
    {0x3206, 0x02},          {0x3207, 0xd7},  // End Y = 727
    {0x3208, 0x02},          {0x3209, 0x80},  // Width = 640
    {0x320a, 0x01},          {0x320b, 0xe0},  // Height = 480
    {0x3210, 0x00},          {0x3211, 0x04},  // X offset = 4
    {0x3212, 0x00},          {0x3213, 0x02},  // Y offset = 2
    // VTS/HTS pour 30fps
    {0x320c, 0x07},          {0x320d, 0x80},  // HTS = 1920
    {0x320e, 0x04},          {0x320f, 0xe2},  // VTS = 1250
    
    {0x3221, 0x00}, 
    {SC202CS_REG_END, 0x00},
};




} // extern "C"

// ============================================================================
// IMPLÉMENTATIONS SCCB ET SENSOR
// ============================================================================

// Structure SCCB IO interne (utilise ESPHome I2C au lieu d'ESP-IDF)
struct esp_sccb_io_t {
    esphome::i2c::I2CDevice *i2c_device;
    uint32_t addr_bits_width;
    uint32_t val_bits_width;
};

// Forward declarations
esp_err_t sccb_new_i2c_io_esphome(esphome::i2c::I2CDevice *i2c_device,
                                   const sccb_i2c_config_t *config,
                                   esp_sccb_io_handle_t *io_handle);

esp_err_t esp_sccb_transmit_reg_a16v8(esp_sccb_io_handle_t handle, 
                                       uint16_t reg_addr, 
                                       uint8_t reg_val);

esp_err_t esp_sccb_transmit_receive_reg_a16v8(esp_sccb_io_handle_t handle, 
                                              uint16_t reg_addr, 
                                              uint8_t *reg_val);

// Implémentation de sccb_new_i2c_io pour ESPHome
esp_err_t sccb_new_i2c_io_esphome(esphome::i2c::I2CDevice *i2c_device,
                                   const sccb_i2c_config_t *config,
                                   esp_sccb_io_handle_t *io_handle) {
    if (!i2c_device || !config || !io_handle) {
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_sccb_io_t *sccb = (esp_sccb_io_t*)malloc(sizeof(esp_sccb_io_t));
    if (!sccb) {
        return ESP_ERR_NO_MEM;
    }
    
    sccb->i2c_device = i2c_device;
    sccb->addr_bits_width = config->addr_bits_width ? config->addr_bits_width : 8;
    sccb->val_bits_width = config->val_bits_width ? config->val_bits_width : 8;
    
    *io_handle = sccb;
    return ESP_OK;
}

// Implémentation de esp_cam_sensor_get_format
esp_err_t esp_cam_sensor_get_format(esp_cam_sensor_device_t *dev, 
                                    esp_cam_sensor_format_t *format) {
    if (!dev || !format || !dev->ops) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!dev->ops->get_format) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    return dev->ops->get_format(dev, format);
}

// Implémentation de esp_cam_sensor_ioctl
esp_err_t esp_cam_sensor_ioctl(esp_cam_sensor_device_t *dev, 
                               uint32_t cmd, 
                               void *arg) {
    if (!dev || !dev->ops) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!dev->ops->priv_ioctl) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    return dev->ops->priv_ioctl(dev, cmd, arg);
}

// Implémentations des fonctions SCCB pour communication I2C
esp_err_t esp_sccb_transmit_reg_a16v8(esp_sccb_io_handle_t handle, 
                                       uint16_t reg_addr, 
                                       uint8_t reg_val) {
    if (!handle || !handle->i2c_device) {
        return ESP_ERR_INVALID_ARG;
    }
    
    uint8_t data[3] = {
        (uint8_t)((reg_addr >> 8) & 0xFF),
        (uint8_t)(reg_addr & 0xFF),
        reg_val
    };
    
    esphome::i2c::ErrorCode err = handle->i2c_device->write(data, 3);
    return (err == esphome::i2c::ERROR_OK) ? ESP_OK : ESP_FAIL;
}

esp_err_t esp_sccb_transmit_receive_reg_a16v8(esp_sccb_io_handle_t handle, 
                                              uint16_t reg_addr, 
                                              uint8_t *reg_val) {
    if (!handle || !handle->i2c_device || !reg_val) {
        return ESP_ERR_INVALID_ARG;
    }
    
    uint8_t addr_buf[2] = {
        (uint8_t)((reg_addr >> 8) & 0xFF),
        (uint8_t)(reg_addr & 0xFF)
    };
    
    esphome::i2c::ErrorCode err = handle->i2c_device->write(addr_buf, 2, false);
    if (err != esphome::i2c::ERROR_OK) {
        return ESP_FAIL;
    }
    
    err = handle->i2c_device->read(reg_val, 1);
    return (err == esphome::i2c::ERROR_OK) ? ESP_OK : ESP_FAIL;
}

// Symboles faibles pour éviter les erreurs de linking
__attribute__((weak)) esp_cam_sensor_detect_fn_t __esp_cam_sensor_detect_fn_array_start = {};
__attribute__((weak)) esp_cam_sensor_detect_fn_t __esp_cam_sensor_detect_fn_array_end = {};

extern "C" {

static const char *SC202CS_TAG = "sc202cs";

// Fonctions SCCB de lecture/écriture
static esp_err_t sc202cs_write(esp_sccb_io_handle_t handle, uint16_t reg, uint8_t data) {
    return esp_sccb_transmit_reg_a16v8(handle, reg, data);
}

static esp_err_t sc202cs_read(esp_sccb_io_handle_t handle, uint16_t reg, uint8_t *data) {
    return esp_sccb_transmit_receive_reg_a16v8(handle, reg, data);
}

static esp_err_t sc202cs_write_array(esp_sccb_io_handle_t handle, sc202cs_reginfo_t *regarray) {
    int i = 0;
    esp_err_t ret = ESP_OK;
    while ((ret == ESP_OK) && regarray[i].reg != SC202CS_REG_END) {
        if (regarray[i].reg != SC202CS_REG_DELAY) {
            ret = sc202cs_write(handle, regarray[i].reg, regarray[i].val);
        } else {
            delay_ms(regarray[i].val);
        }
        i++;
    }
    return ret;
}

// Fonction pour définir des bits dans un registre
static esp_err_t sc202cs_set_reg_bits(esp_sccb_io_handle_t handle, 
                                       uint16_t reg, 
                                       uint8_t offset, 
                                       uint8_t length,
                                       uint8_t value) {
    esp_err_t ret = ESP_OK;
    uint8_t reg_data = 0;

    ret = sc202cs_read(handle, reg, &reg_data);
    if (ret != ESP_OK) {
        return ret;
    }
    
    uint8_t mask = ((1 << length) - 1) << offset;
    value = (reg_data & ~mask) | ((value << offset) & mask);
    ret = sc202cs_write(handle, reg, value);
    return ret;
}

// Fonction pour activer le mirror (flip horizontal)
static esp_err_t sc202cs_set_mirror(esp_cam_sensor_device_t *dev, int enable) {
    return sc202cs_set_reg_bits(dev->sccb_handle, 0x3221, 1, 2, enable ? 0x03 : 0x00);
}

// Fonction pour activer le vflip (flip vertical)
static esp_err_t sc202cs_set_vflip(esp_cam_sensor_device_t *dev, int enable) {
    return sc202cs_set_reg_bits(dev->sccb_handle, 0x3221, 5, 2, enable ? 0x03 : 0x00);
}

static esp_err_t sc202cs_get_sensor_id(esp_cam_sensor_device_t *dev, esp_cam_sensor_id_t *id) {
    uint8_t pid_h, pid_l;
    esp_err_t ret = sc202cs_read(dev->sccb_handle, SC202CS_REG_SENSOR_ID_H, &pid_h);
    if (ret != ESP_OK) return ret;
    
    ret = sc202cs_read(dev->sccb_handle, SC202CS_REG_SENSOR_ID_L, &pid_l);
    if (ret != ESP_OK) return ret;
    
    id->pid = (pid_h << 8) | pid_l;
    return ESP_OK;
}

static esp_err_t sc202cs_set_stream(esp_cam_sensor_device_t *dev, int enable) {
    esp_err_t ret = sc202cs_write(dev->sccb_handle, SC202CS_REG_SLEEP_MODE, enable ? 0x01 : 0x00);
    dev->stream_status = enable;
    ESP_LOGI(SC202CS_TAG, "Stream=%d", enable);
    return ret;
}

static esp_err_t sc202cs_set_format(esp_cam_sensor_device_t *dev, const void *format) {
    // Si format est NULL, utiliser la config par défaut depuis dev->priv
    const sc202cs_reginfo_t *reg_list = NULL;
    
    if (format != NULL) {
        // Format spécifié explicitement
        const esp_cam_sensor_format_t *fmt = (const esp_cam_sensor_format_t *)format;
        reg_list = (const sc202cs_reginfo_t *)fmt->regs;
    } else if (dev->priv != NULL) {
        // Utiliser la résolution stockée dans priv
        uint32_t resolution_index = *(uint32_t*)dev->priv;
        
        switch (resolution_index) {
            case 0: // VGA 640x480
                reg_list = init_reglist_640x480_30fps;
                ESP_LOGI(SC202CS_TAG, "Config: VGA 640x480@30fps");
                break;
            case 1: // 720P 1280x720
                reg_list = init_reglist_1280x720_30fps;
                ESP_LOGI(SC202CS_TAG, "Config: 720P 1280x720@30fps");
                break;
            case 2: // 1080P (si disponible)
            case 3: // QVGA (si disponible)
            default:
                // Par défaut, utiliser 720P
                reg_list = init_reglist_1280x720_30fps;
                ESP_LOGE(SC202CS_TAG, "Résolution non supportée, utilisation 720P");
                break;
        }
    } else {
        // Fallback absolu
        reg_list = init_reglist_1280x720_30fps;
        ESP_LOGW(SC202CS_TAG, "Format par défaut: 720P");
    }
    
    if (reg_list == NULL) {
        ESP_LOGE(SC202CS_TAG, "Liste de registres invalide");
        return ESP_FAIL;
    }
    
    esp_err_t ret = sc202cs_write_array(dev->sccb_handle, (sc202cs_reginfo_t*)reg_list);
    
    if (ret != ESP_OK) {
        ESP_LOGE(SC202CS_TAG, "Set format failed: %d", ret);
        return ret;
    }
    
    ESP_LOGI(SC202CS_TAG, "✓ Format configuré");
    return ESP_OK;
}

// CORRECTION: Support flip/mirror via IOCTL
static esp_err_t sc202cs_priv_ioctl(esp_cam_sensor_device_t *dev, uint32_t cmd, void *arg) {
    esp_err_t ret = ESP_OK;
    
    switch (cmd) {
        case 0x04000004: // ESP_CAM_SENSOR_IOC_S_STREAM
            ret = sc202cs_set_stream(dev, *(int*)arg);
            break;
            
        case 0x04000010: // ESP_CAM_SENSOR_IOC_S_VFLIP
            ret = sc202cs_set_vflip(dev, *(int*)arg);
            ESP_LOGI(SC202CS_TAG, "VFlip: %d", *(int*)arg);
            break;
            
        case 0x04000011: // ESP_CAM_SENSOR_IOC_S_HMIRROR
            ret = sc202cs_set_mirror(dev, *(int*)arg);
            ESP_LOGI(SC202CS_TAG, "HMirror: %d", *(int*)arg);
            break;
            
        default:
            ESP_LOGW(SC202CS_TAG, "IOCTL non supporté: 0x%08X", cmd);
            ret = ESP_ERR_NOT_SUPPORTED;
            break;
    }
    
    return ret;
}

static esp_err_t sc202cs_delete(esp_cam_sensor_device_t *dev) {
    if (dev) {
        if (dev->priv) free(dev->priv);
        free(dev);
    }
    return ESP_OK;
}

static const esp_cam_sensor_ops_t sc202cs_ops = {
    .set_format = (int (*)(esp_cam_sensor_device_t*, const void*))sc202cs_set_format,
    .priv_ioctl = (int (*)(esp_cam_sensor_device_t*, uint32_t, void*))sc202cs_priv_ioctl,
    .del = (int (*)(esp_cam_sensor_device_t*))sc202cs_delete,
};

esp_cam_sensor_device_t *sc202cs_detect(esp_cam_sensor_config_t *config) {
    if (!config) return NULL;
    
    esp_cam_sensor_device_t *dev = (esp_cam_sensor_device_t*)calloc(1, sizeof(esp_cam_sensor_device_t));
    if (!dev) {
        ESP_LOGE(SC202CS_TAG, "No memory");
        return NULL;
    }
    
    dev->name = (char*)SC202CS_SENSOR_NAME;
    dev->sccb_handle = config->sccb_handle;
    dev->xclk_pin = config->xclk_pin;
    dev->reset_pin = config->reset_pin;
    dev->pwdn_pin = config->pwdn_pin;
    dev->sensor_port = config->sensor_port;
    dev->ops = &sc202cs_ops;
    
    // Vérifier l'ID du capteur
    if (sc202cs_get_sensor_id(dev, &dev->id) != ESP_OK) {
        ESP_LOGE(SC202CS_TAG, "Get sensor ID failed");
        free(dev);
        return NULL;
    }
    
    if (dev->id.pid != SC202CS_PID) {
        ESP_LOGE(SC202CS_TAG, "Wrong PID: 0x%x", dev->id.pid);
        free(dev);
        return NULL;
    }
    
    ESP_LOGI(SC202CS_TAG, "SC202CS detected, PID=0x%x", dev->id.pid);
    
    return dev;
}

} // extern "C"

#endif  // USE_ESP32_VARIANT_ESP32P4

// ============================================================================
// CODE TAB5 CAMERA
// ============================================================================

namespace esphome {
namespace tab5_camera {

static const char *const TAG = "tab5_camera";

void Tab5Camera::setup() {
  ESP_LOGI(TAG, "🎥 Initialisation Tab5 Camera");
  
#ifdef USE_ESP32_VARIANT_ESP32P4
  // 1. Init pins
  if (this->reset_pin_ != nullptr) {
    this->reset_pin_->setup();
    this->reset_pin_->digital_write(false);
    delay(10);
    this->reset_pin_->digital_write(true);
    delay(20);
  }
  
  if (this->pwdn_pin_ != nullptr) {
    this->pwdn_pin_->setup();
    this->pwdn_pin_->digital_write(false);
  }
  
  // 2. Init capteur SC202CS
  if (!this->init_sensor_()) {
    ESP_LOGE(TAG, "❌ Échec init capteur");
    this->mark_failed();
    return;
  }
  
  // 3. Init LDO pour MIPI
  if (!this->init_ldo_()) {
    ESP_LOGE(TAG, "❌ Échec init LDO");
    this->mark_failed();
    return;
  }
  
  // 4. Init CSI
  if (!this->init_csi_()) {
    ESP_LOGE(TAG, "❌ Échec init CSI");
    this->mark_failed();
    return;
  }
  
  // 5. Init ISP
  if (!this->init_isp_()) {
    ESP_LOGE(TAG, "❌ Échec init ISP");
    this->mark_failed();
    return;
  }
  
  // 6. Allouer le buffer
  if (!this->allocate_buffer_()) {
    ESP_LOGE(TAG, "❌ Échec allocation buffer");
    this->mark_failed();
    return;
  }
  
  this->initialized_ = true;
  ESP_LOGI(TAG, "✅ Caméra prête");
  
#else
  ESP_LOGE(TAG, "❌ ESP32-P4 requis");
  this->mark_failed();
#endif
}

#ifdef USE_ESP32_VARIANT_ESP32P4

bool Tab5Camera::init_sensor_() {
  ESP_LOGI(TAG, "Init capteur SC202CS");
  
  // Configurer SCCB pour SC202CS
  sccb_i2c_config_t sccb_config = {};
  sccb_config.device_address = this->sensor_address_;
  sccb_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  sccb_config.scl_speed_hz = 400000;
  sccb_config.addr_bits_width = 16;
  sccb_config.val_bits_width = 8;
  
  esp_sccb_io_handle_t sccb_handle;
  esp_err_t ret = sccb_new_i2c_io_esphome(this, &sccb_config, &sccb_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "SCCB init failed: %d", ret);
    return false;
  }
  
  ESP_LOGI(TAG, "✓ SCCB initialisé via ESPHome I2C");
  
  // NOUVEAU: Allouer priv pour stocker la résolution
  uint32_t *resolution_ptr = (uint32_t*)malloc(sizeof(uint32_t));
  if (!resolution_ptr) {
    ESP_LOGE(TAG, "Erreur allocation mémoire pour résolution");
    return false;
  }
  *resolution_ptr = (uint32_t)this->resolution_;
  
  // Configurer le capteur
  esp_cam_sensor_config_t sensor_config = {};
  sensor_config.sccb_handle = sccb_handle;
  sensor_config.reset_pin = -1;
  sensor_config.pwdn_pin = -1;
  sensor_config.xclk_pin = (int8_t)this->external_clock_pin_;
  sensor_config.xclk_freq_hz = this->external_clock_frequency_;
  sensor_config.sensor_port = ESP_CAM_SENSOR_MIPI_CSI;
  
  // Détecter et initialiser le capteur
  this->sensor_device_ = sc202cs_detect(&sensor_config);
  
  if (this->sensor_device_ == nullptr) {
    free(resolution_ptr);
    ESP_LOGE(TAG, "SC202CS detection failed");
    return false;
  }
  
  // IMPORTANT: Stocker la résolution dans priv
  this->sensor_device_->priv = resolution_ptr;
  
  // Configurer le format avec la résolution
  if (sc202cs_set_format(this->sensor_device_, NULL) != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set format");
    free(resolution_ptr);
    free(this->sensor_device_);
    this->sensor_device_ = nullptr;
    return false;
  }
  
  // NOUVEAU: Appliquer flip/mirror si configuré
  if (this->flip_mirror_) {
    int enable = 1;
    esp_cam_sensor_ioctl(this->sensor_device_, 0x04000010, &enable); // VFlip
    esp_cam_sensor_ioctl(this->sensor_device_, 0x04000011, &enable); // HMirror
    ESP_LOGI(TAG, "✓ Flip/Mirror activé");
  }
  
  ESP_LOGI(TAG, "✓ SC202CS détecté (PID: 0x%04X)", this->sensor_device_->id.pid);
  return true;
}

bool Tab5Camera::init_ldo_() {
  ESP_LOGI(TAG, "Init LDO MIPI");
  
  esp_ldo_channel_config_t ldo_config = {
    .chan_id = 3,
    .voltage_mv = 2500,
  };
  
  esp_err_t ret = esp_ldo_acquire_channel(&ldo_config, &this->ldo_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "LDO failed: %d", ret);
    return false;
  }
  
  ESP_LOGI(TAG, "✓ LDO OK (2.5V)");
  return true;
}

bool Tab5Camera::init_csi_() {
  ESP_LOGI(TAG, "Init MIPI-CSI");
  
  CameraResolutionInfo res = this->get_resolution_info_();
  
  esp_cam_ctlr_csi_config_t csi_config = {};
  csi_config.ctlr_id = 0;
  csi_config.clk_src = MIPI_CSI_PHY_CLK_SRC_DEFAULT;
  csi_config.h_res = res.width;
  csi_config.v_res = res.height;
  csi_config.lane_bit_rate_mbps = 576;
  csi_config.input_data_color_type = CAM_CTLR_COLOR_RAW8;
  csi_config.output_data_color_type = CAM_CTLR_COLOR_RGB565;
  csi_config.data_lane_num = 1;
  csi_config.byte_swap_en = false;
  csi_config.queue_items = 3;
  
  esp_err_t ret = esp_cam_new_csi_ctlr(&csi_config, &this->csi_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "CSI failed: %d", ret);
    return false;
  }
  
  // Callbacks
  esp_cam_ctlr_evt_cbs_t callbacks = {
    .on_get_new_trans = Tab5Camera::on_csi_new_frame_,
    .on_trans_finished = Tab5Camera::on_csi_frame_done_,
  };
  
  ret = esp_cam_ctlr_register_event_callbacks(this->csi_handle_, &callbacks, this);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Callbacks failed: %d", ret);
    return false;
  }
  
  ret = esp_cam_ctlr_enable(this->csi_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Enable CSI failed: %d", ret);
    return false;
  }
  
  ESP_LOGI(TAG, "✓ CSI OK (%ux%u)", res.width, res.height);
  return true;
}

bool Tab5Camera::init_isp_() {
  ESP_LOGI(TAG, "Initialisation ISP");
  
  CameraResolutionInfo res = this->get_resolution_info_();
  
  // Ajuster la fréquence ISP selon la résolution
  uint32_t isp_clock_hz = 80000000;
  if (this->resolution_ == RESOLUTION_720P) {
    isp_clock_hz = 120000000;
  }
  
  esp_isp_processor_cfg_t isp_config = {};
  isp_config.clk_src = ISP_CLK_SRC_DEFAULT;
  isp_config.input_data_source = ISP_INPUT_DATA_SOURCE_CSI;
  isp_config.input_data_color_type = ISP_COLOR_RAW8;
  isp_config.output_data_color_type = ISP_COLOR_RGB565;
  isp_config.h_res = res.width;
  isp_config.v_res = res.height;
  isp_config.has_line_start_packet = false;
  isp_config.has_line_end_packet = false;
  isp_config.clk_hz = isp_clock_hz;
  
  // Configuration du pattern Bayer
  int bayer_pattern = 3;  // RGGB - testez 0, 1, 2, 3
  
  isp_config.bayer_order = (color_raw_element_order_t)bayer_pattern;
  
  const char* bayer_names[] = {"RGGB", "GRBG", "GBRG", "BGGR"};
  ESP_LOGI(TAG, "Pattern Bayer: %s (%d)", bayer_names[bayer_pattern], bayer_pattern);
  
  esp_err_t ret = esp_isp_new_processor(&isp_config, &this->isp_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Échec création ISP: 0x%x", ret);
    return false;
  }
  
  ret = esp_isp_enable(this->isp_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Échec activation ISP: 0x%x", ret);
    esp_isp_del_processor(this->isp_handle_);
    this->isp_handle_ = nullptr;
    return false;
  }
  
  ESP_LOGI(TAG, "✓ ISP initialisé (clock=%u MHz, bayer=%s)", 
           isp_clock_hz / 1000000, bayer_names[bayer_pattern]);
  
  // Configurer les corrections couleur
  this->configure_isp_color_correction_();
  
  return true;
}

void Tab5Camera::configure_isp_color_correction_() {
  ESP_LOGI(TAG, "Configuration corrections couleur");
  
  // Corrections couleur de base si supportées
#ifdef CONFIG_ISP_COLOR_ENABLED
  esp_isp_color_config_t color_config = {};
  color_config.color_contrast = {160, 160, 160};
  color_config.color_saturation = {140, 140, 140};
  color_config.color_hue = 0;
  color_config.color_brightness = 100;
  
  esp_err_t ret = esp_isp_color_configure(this->isp_handle_, &color_config);
  if (ret == ESP_OK) {
    ESP_LOGI(TAG, "✓ Corrections couleur configurées");
  }
#endif

  // Tentative d'activation AWB
  if (this->sensor_device_) {
    int awb_value = 1;
    
    // Utilisation de la valeur numérique pour AWB
    esp_err_t ret = esp_cam_sensor_ioctl(this->sensor_device_, 0x03010001, &awb_value);
    
    if (ret == ESP_OK) {
      ESP_LOGI(TAG, "✓ AWB activé");
    } else {
      ESP_LOGW(TAG, "AWB non supporté (erreur: 0x%x), utilisation balance manuelle", ret);
      this->apply_manual_white_balance_();
    }
  }
}

void Tab5Camera::apply_manual_white_balance_() {
  ESP_LOGI(TAG, "Application balance des blancs manuelle");
  
#ifdef CONFIG_ISP_COLOR_ENABLED
  esp_isp_color_config_t color_config = {};
  
  // Réglages manuels pour compensation
  color_config.color_contrast = {145, 145, 145};
  color_config.color_saturation = {135, 135, 135};
  color_config.color_hue = 0;
  color_config.color_brightness = 40;
  
  esp_err_t ret = esp_isp_color_configure(this->isp_handle_, &color_config);
  if (ret == ESP_OK) {
    ESP_LOGI(TAG, "✓ Balance manuelle appliquée");
  } else {
    ESP_LOGW(TAG, "Configuration couleur non supportée");
  }
#endif
}
bool Tab5Camera::allocate_buffer_() {
  CameraResolutionInfo res = this->get_resolution_info_();
  this->frame_buffer_size_ = res.width * res.height * 2;
  
  this->frame_buffers_[0] = (uint8_t*)heap_caps_aligned_alloc(
    64, this->frame_buffer_size_, MALLOC_CAP_SPIRAM
  );
  
  this->frame_buffers_[1] = (uint8_t*)heap_caps_aligned_alloc(
    64, this->frame_buffer_size_, MALLOC_CAP_SPIRAM
  );
  
  if (!this->frame_buffers_[0] || !this->frame_buffers_[1]) {
    ESP_LOGE(TAG, "Buffer alloc failed");
    return false;
  }
  
  this->current_frame_buffer_ = this->frame_buffers_[0];
  
  ESP_LOGI(TAG, "✓ Buffers: 2x%u bytes", this->frame_buffer_size_);
  return true;
}

bool IRAM_ATTR Tab5Camera::on_csi_new_frame_(
  esp_cam_ctlr_handle_t handle,
  esp_cam_ctlr_trans_t *trans,
  void *user_data
) {
  Tab5Camera *cam = (Tab5Camera*)user_data;
  
  trans->buffer = cam->frame_buffers_[cam->buffer_index_];
  trans->buflen = cam->frame_buffer_size_;
  
  return false;
}

bool IRAM_ATTR Tab5Camera::on_csi_frame_done_(
  esp_cam_ctlr_handle_t handle,
  esp_cam_ctlr_trans_t *trans,
  void *user_data
) {
  Tab5Camera *cam = (Tab5Camera*)user_data;
  
  if (trans->received_size > 0) {
    cam->frame_ready_ = true;
    cam->buffer_index_ = (cam->buffer_index_ + 1) % 2;
  }
  
  return false;
}

CameraResolutionInfo Tab5Camera::get_resolution_info_() const {
  switch (this->resolution_) {
    case RESOLUTION_720P: return {1280, 720};
    case RESOLUTION_VGA:
    default: return {640, 480};
  }
}

bool Tab5Camera::start_streaming() {
  if (!this->initialized_ || this->streaming_) {
    return false;
  }
  
  ESP_LOGI(TAG, "Démarrage streaming");
  
  // IMPORTANT: Vérifier la résolution configurée
  CameraResolutionInfo res = this->get_resolution_info_();
  ESP_LOGI(TAG, "Résolution active: %ux%u", res.width, res.height);
  
  // Démarrer le capteur
  if (this->sensor_device_) {
    int enable = 1;
    esp_err_t ret = esp_cam_sensor_ioctl(
      this->sensor_device_, 
      0x04000004,
      &enable
    );
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Failed to start sensor: %d", ret);
      return false;
    }
    
    // Délai pour que le capteur démarre complètement
    delay(100);
  }
  
  // Démarrer CSI
  esp_err_t ret = esp_cam_ctlr_start(this->csi_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Start CSI failed: %d", ret);
    return false;
  }
  
  this->streaming_ = true;
  ESP_LOGI(TAG, "✅ Streaming actif (%ux%u)", res.width, res.height);
  return true;
}

bool Tab5Camera::stop_streaming() {
  if (!this->streaming_) {
    return true;
  }
  
  esp_cam_ctlr_stop(this->csi_handle_);
  
  if (this->sensor_device_) {
    int enable = 0;
    esp_cam_sensor_ioctl(this->sensor_device_, 0x04000004, &enable);
  }
  
  this->streaming_ = false;
  ESP_LOGI(TAG, "⏹ Streaming arrêté");
  return true;
}

bool Tab5Camera::capture_frame() {
  if (!this->streaming_) {
    return false;
  }
  
  bool was_ready = this->frame_ready_;
  if (was_ready) {
    this->frame_ready_ = false;
    
    uint8_t last_complete_buffer = (this->buffer_index_ + 1) % 2;
    this->current_frame_buffer_ = this->frame_buffers_[last_complete_buffer];
  }
  
  return was_ready;
}

uint16_t Tab5Camera::get_image_width() const {
  return this->get_resolution_info_().width;
}

uint16_t Tab5Camera::get_image_height() const {
  return this->get_resolution_info_().height;
}

#endif  // USE_ESP32_VARIANT_ESP32P4

void Tab5Camera::loop() {
  // Tout est géré par les callbacks ISR
}

void Tab5Camera::dump_config() {
  ESP_LOGCONFIG(TAG, "Tab5 Camera:");
  ESP_LOGCONFIG(TAG, "  Capteur: SC202CS @ 0x%02X", this->sensor_address_);
  ESP_LOGCONFIG(TAG, "  Résolution: %ux%u", 
                this->get_image_width(), this->get_image_height());
  ESP_LOGCONFIG(TAG, "  Format: RGB565");
  ESP_LOGCONFIG(TAG, "  Flip/Mirror: %s", this->flip_mirror_ ? "OUI" : "NON");
  ESP_LOGCONFIG(TAG, "  Streaming: %s", this->streaming_ ? "OUI" : "NON");
  ESP_LOGCONFIG(TAG, "  Initialisé: %s", this->initialized_ ? "OUI" : "NON");
}

}  // namespace tab5_camera
}  // namespace esphome


