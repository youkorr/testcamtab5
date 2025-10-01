#pragma once

// Configuration des pins DVP (Digital Video Port) pour M5Stack Tab5
// Référence: M5Tab5-UserDemo/platforms/tab5/components/esp_cam_sensor

namespace esphome {
namespace tab5_camera {

// Pins DVP pour la capture d'image via ESP32-P4
struct Tab5DVPConfig {
  // Clock et sync
  static constexpr int PCLK_GPIO = 46;   // Pixel clock
  static constexpr int VSYNC_GPIO = 38;  // Vertical sync
  static constexpr int HSYNC_GPIO = 39;  // Horizontal sync
  
  // Pins de données (8 bits pour SC202CS)
  static constexpr int D0_GPIO = 40;
  static constexpr int D1_GPIO = 41;
  static constexpr int D2_GPIO = 42;
  static constexpr int D3_GPIO = 43;
  static constexpr int D4_GPIO = 44;
  static constexpr int D5_GPIO = 45;
  static constexpr int D6_GPIO = 33;
  static constexpr int D7_GPIO = 34;
  
  // Pins de contrôle caméra
  static constexpr int XCLK_GPIO = 47;   // Master clock (24MHz)
  static constexpr int RESET_GPIO = 48;  // Reset hardware
  static constexpr int PWDN_GPIO = -1;   // Power down (non utilisé sur Tab5)
  
  // Configuration I2C
  static constexpr int SDA_GPIO = 31;
  static constexpr int SCL_GPIO = 32;
  static constexpr int I2C_FREQ = 400000;  // 400 kHz
  
  // Configuration horloge
  static constexpr uint32_t XCLK_FREQ = 24000000;  // 24 MHz
};

// Configuration DVP pour ESP-IDF Camera Driver
struct DVPCameraConfig {
  int pin_pwdn;
  int pin_reset;
  int pin_xclk;
  int pin_sda;
  int pin_scl;
  
  int pin_d7;
  int pin_d6;
  int pin_d5;
  int pin_d4;
  int pin_d3;
  int pin_d2;
  int pin_d1;
  int pin_d0;
  
  int pin_vsync;
  int pin_href;
  int pin_pclk;
  
  uint32_t xclk_freq_hz;
  int ledc_timer;
  int ledc_channel;
  
  // Obtenir la configuration pour Tab5
  static DVPCameraConfig get_tab5_config() {
    return {
      .pin_pwdn = Tab5DVPConfig::PWDN_GPIO,
      .pin_reset = Tab5DVPConfig::RESET_GPIO,
      .pin_xclk = Tab5DVPConfig::XCLK_GPIO,
      .pin_sda = Tab5DVPConfig::SDA_GPIO,
      .pin_scl = Tab5DVPConfig::SCL_GPIO,
      
      .pin_d7 = Tab5DVPConfig::D7_GPIO,
      .pin_d6 = Tab5DVPConfig::D6_GPIO,
      .pin_d5 = Tab5DVPConfig::D5_GPIO,
      .pin_d4 = Tab5DVPConfig::D4_GPIO,
      .pin_d3 = Tab5DVPConfig::D3_GPIO,
      .pin_d2 = Tab5DVPConfig::D2_GPIO,
      .pin_d1 = Tab5DVPConfig::D1_GPIO,
      .pin_d0 = Tab5DVPConfig::D0_GPIO,
      
      .pin_vsync = Tab5DVPConfig::VSYNC_GPIO,
      .pin_href = Tab5DVPConfig::HSYNC_GPIO,
      .pin_pclk = Tab5DVPConfig::PCLK_GPIO,
      
      .xclk_freq_hz = Tab5DVPConfig::XCLK_FREQ,
      .ledc_timer = 0,
      .ledc_channel = 0,
    };
  }
};

// Notes sur les pins DVP:
// 
// Le SC202CS utilise une interface DVP 8 bits pour transférer les données d'image
// du capteur vers l'ESP32-P4. Les signaux sont:
//
// - PCLK (Pixel Clock): Horloge pour synchroniser le transfert de chaque pixel
// - VSYNC (Vertical Sync): Signal de début/fin de trame (frame)
// - HSYNC (Horizontal Sync): Signal de début/fin de ligne
// - D0-D7: 8 lignes de données parallèles (1 octet par cycle PCLK)
//
// Séquence de capture:
// 1. VSYNC passe à HIGH → Début d'une nouvelle frame
// 2. HSYNC passe à HIGH → Début d'une nouvelle ligne
// 3. À chaque front de PCLK, un octet est transféré sur D0-D7
// 4. HSYNC passe à LOW → Fin de ligne
// 5. Répéter étapes 2-4 pour chaque ligne
// 6. VSYNC passe à LOW → Fin de frame
//
// Pour une image VGA (640x480):
// - 480 lignes (HSYNC pulses)
// - 640 pixels par ligne
// - RGB565: 2 octets par pixel → 614,400 octets par frame

}  // namespace tab5_camera
}  // namespace esphome
