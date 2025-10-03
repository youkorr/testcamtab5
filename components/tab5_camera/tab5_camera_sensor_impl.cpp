// Implémentation des fonctions manquantes du sensor
// Ce fichier est inclus directement dans tab5_camera.cpp
// Les fonctions sont déclarées static pour éviter les conflits de linking

#ifdef USE_ESP32_VARIANT_ESP32P4

#include "tab5_camera_sensor.h"
#include <string.h>

// Définition de la structure SCCB IO
struct esp_sccb_io_t {
    i2c_master_dev_handle_t i2c_dev;
    uint32_t addr_bits_width;
    uint32_t val_bits_width;
};

// Implémentation de sccb_new_i2c_io (static pour éviter conflits)
static esp_err_t sccb_new_i2c_io_impl(i2c_master_bus_handle_t bus_handle, 
                                       const sccb_i2c_config_t *config,
                                       esp_sccb_io_handle_t *io_handle) {
    if (!bus_handle || !config || !io_handle) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Créer un device I2C sur le bus
    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = config->dev_addr_length;
    dev_cfg.device_address = config->device_address;
    dev_cfg.scl_speed_hz = config->scl_speed_hz;
    
    i2c_master_dev_handle_t dev_handle;
    esp_err_t ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Allouer la structure SCCB IO
    esp_sccb_io_t *sccb = (esp_sccb_io_t*)malloc(sizeof(esp_sccb_io_t));
    if (!sccb) {
        i2c_master_bus_rm_device(dev_handle);
        return ESP_ERR_NO_MEM;
    }
    
    sccb->i2c_dev = dev_handle;
    sccb->addr_bits_width = config->addr_bits_width ? config->addr_bits_width : 8;
    sccb->val_bits_width = config->val_bits_width ? config->val_bits_width : 8;
    
    *io_handle = sccb;
    return ESP_OK;
}

// Wrapper externe pour sccb_new_i2c_io
extern "C" esp_err_t sccb_new_i2c_io(i2c_master_bus_handle_t bus_handle, 
                                      const sccb_i2c_config_t *config,
                                      esp_sccb_io_handle_t *io_handle) {
    return sccb_new_i2c_io_impl(bus_handle, config, io_handle);
}

// Implémentation de esp_cam_sensor_get_format (static)
static esp_err_t esp_cam_sensor_get_format_impl(esp_cam_sensor_device_t *dev, 
                                                 esp_cam_sensor_format_t *format) {
    if (!dev || !format || !dev->ops) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!dev->ops->get_format) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    
    return dev->ops->get_format(dev, format);
}

// Wrapper externe pour esp_cam_sensor_get_format
extern "C" esp_err_t esp_cam_sensor_get_format(esp_cam_sensor_device_t *dev, 
                                                 esp_cam_sensor_format_t *format) {
    return esp_cam_sensor_get_format_impl(dev, format);
}

// Implémentation de esp_cam_sensor_ioctl (static)
static esp_err_t esp_cam_sensor_ioctl_impl(esp_cam_sensor_device_t *dev, 
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

// Wrapper externe pour esp_cam_sensor_ioctl
extern "C" esp_err_t esp_cam_sensor_ioctl(esp_cam_sensor_device_t *dev, 
                                           uint32_t cmd, 
                                           void *arg) {
    return esp_cam_sensor_ioctl_impl(dev, cmd, arg);
}

// ============================================================================
// Implémentations des fonctions SCCB pour communication I2C
// ============================================================================

extern "C" esp_err_t esp_sccb_transmit_reg_a16v8(esp_sccb_io_handle_t handle, 
                                                   uint16_t reg_addr, 
                                                   uint8_t reg_val) {
    if (!handle || !handle->i2c_dev) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Format: [REG_ADDR_H, REG_ADDR_L, VALUE]
    uint8_t write_buf[3];
    write_buf[0] = (reg_addr >> 8) & 0xFF;  // Adresse haute
    write_buf[1] = reg_addr & 0xFF;         // Adresse basse
    write_buf[2] = reg_val;                  // Valeur
    
    return i2c_master_transmit(handle->i2c_dev, write_buf, 3, -1);
}

extern "C" esp_err_t esp_sccb_transmit_receive_reg_a16v8(esp_sccb_io_handle_t handle, 
                                                          uint16_t reg_addr, 
                                                          uint8_t *reg_val) {
    if (!handle || !handle->i2c_dev || !reg_val) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Format pour écrire l'adresse: [REG_ADDR_H, REG_ADDR_L]
    uint8_t reg_addr_buf[2];
    reg_addr_buf[0] = (reg_addr >> 8) & 0xFF;
    reg_addr_buf[1] = reg_addr & 0xFF;
    
    // Écrire l'adresse puis lire la valeur
    return i2c_master_transmit_receive(handle->i2c_dev, 
                                       reg_addr_buf, 2, 
                                       reg_val, 1, 
                                       -1);
}

// Déclaration des symboles externes - ils seront définis par le linker
// Ces symboles sont créés par la macro ESP_CAM_SENSOR_DETECT_FN dans sc202cs.c
extern "C" {
    // Symboles faibles pour éviter les erreurs de linking si sc202cs n'est pas compilé
    __attribute__((weak)) esp_cam_sensor_detect_fn_t __esp_cam_sensor_detect_fn_array_start = {0};
    __attribute__((weak)) esp_cam_sensor_detect_fn_t __esp_cam_sensor_detect_fn_array_end = {0};
}

#endif // USE_ESP32_VARIANT_ESP32P4
