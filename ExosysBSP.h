// =====================================================================
// ExosysBSP.h — BOARD SUPPORT PACKAGE PARA MOTORES MODULARES EXOSYS
// Soporta:
//   1. BOARD_TWATCH_ULTRA  (410x502, QSPI AMOLED, Touch CST92xx, SD, GPS, DRV2605)
//   2. BOARD_TDISPLAY_S3   (170x320 o 320x170, LCD 8-bit, SD externa SPI, GPS/BLE)
//   3. BOARD_ESP32_GENERIC (240x320 / 320x240 SPI TFT, SD SPI)
// =====================================================================
#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SdFat.h>

// Si no se define la placa en platformio.ini, usa T-Watch Ultra por defecto
#if !defined(BOARD_TWATCH_ULTRA) && !defined(BOARD_TDISPLAY_S3) && !defined(BOARD_ESP32_GENERIC)
    #define BOARD_TWATCH_ULTRA
#endif

// ---------------------------------------------------------------------
// 1. PERFIL DE HARDWARE POR PLACA
// ---------------------------------------------------------------------
#if defined(BOARD_TWATCH_ULTRA)
    #include <LilyGoLib.h>
    #include <LV_Helper.h>
    #define EXO_SCREEN_W          410
    #define EXO_SCREEN_H          502
    #define EXO_HAS_DRV2605       1
    #define EXO_HAS_MULTITOUCH    1
    #define EXO_SD_CS_PIN         21
    #define EXO_GPS_RX_PIN        44
    #define EXO_GPS_TX_PIN        43
    #define EXO_GPS_BAUD          38400
    #define EXO_BTN_BACK_PIN      0

#elif defined(BOARD_TDISPLAY_S3)
    #define EXO_SCREEN_W          170
    #define EXO_SCREEN_H          320
    #define EXO_HAS_DRV2605       0
    #define EXO_HAS_MULTITOUCH    0
    // Pines SPI libres en LilyGo T-Display-S3 para lector MicroSD externo
    #define EXO_SD_CS_PIN         10
    #define EXO_SD_MOSI_PIN       11
    #define EXO_SD_SCLK_PIN       12
    #define EXO_SD_MISO_PIN       13
    // Pines libres para módulo GPS UART opcional (si no se conecta, usa BLE o Exploración)
    #define EXO_GPS_RX_PIN        44
    #define EXO_GPS_TX_PIN        43
    #define EXO_GPS_BAUD          9600
    #define EXO_BTN_BACK_PIN      0   // Botón izquierdo T-Display-S3
    #define EXO_BTN_AUX_PIN       14  // Botón derecho T-Display-S3

#else // BOARD_ESP32_GENERIC
    #ifndef EXO_SCREEN_W
        #define EXO_SCREEN_W      240
    #endif
    #ifndef EXO_SCREEN_H
        #define EXO_SCREEN_H      320
    #endif
    #define EXO_HAS_DRV2605       0
    #define EXO_HAS_MULTITOUCH    0
    #define EXO_SD_CS_PIN         5
    #define EXO_GPS_RX_PIN        16
    #define EXO_GPS_TX_PIN        17
    #define EXO_GPS_BAUD          9600
    #define EXO_BTN_BACK_PIN      0
#endif

// ---------------------------------------------------------------------
// 2. GEOMETRÍA UNIVERSAL DEL CONTENEDOR 4x4 (1024x1024) PARA CARTOENGINE
// ---------------------------------------------------------------------
// Alinea el píxel (512, 512) del contenedor de 16 teselas con el centro exacto de cualquier pantalla
#define EXO_MAP_ORIGIN_X      ((EXO_SCREEN_W / 2) - 512)
#define EXO_MAP_ORIGIN_Y      ((EXO_SCREEN_H / 2) - 512)
#define EXO_SCREEN_CX         (EXO_SCREEN_W / 2)
#define EXO_SCREEN_CY         (EXO_SCREEN_H / 2)

// Escalado automático de paneles modales para pantallas compactas (ej. T-Display 170x320)
#define EXO_MODAL_W(target_w) ((EXO_SCREEN_W - 12) < (target_w) ? (EXO_SCREEN_W - 12) : (target_w))
#define EXO_MODAL_H(target_h) ((EXO_SCREEN_H - 16) < (target_h) ? (EXO_SCREEN_H - 16) : (target_h))

// ---------------------------------------------------------------------
// 3. WRAPPERS DE HARDWARE DESACOPLADOS
// ---------------------------------------------------------------------
static inline void exo_haptic_init() {
#if EXO_HAS_DRV2605
    Wire.beginTransmission(0x5A); Wire.write(0x01); Wire.write(0x00); Wire.endTransmission();
#endif
}

static inline void exo_haptic_fire(uint8_t effect_id) {
#if EXO_HAS_DRV2605
    Wire.beginTransmission(0x5A); Wire.write(0x04); Wire.write(effect_id); Wire.endTransmission();
    Wire.beginTransmission(0x5A); Wire.write(0x05); Wire.write(0x00); Wire.endTransmission();
    Wire.beginTransmission(0x5A); Wire.write(0x0C); Wire.write(0x01); Wire.endTransmission();
#else
    (void)effect_id;
#endif
}

static inline uint8_t exo_touch_get_points(int16_t *x_arr, int16_t *y_arr, uint8_t max_pts) {
#if defined(BOARD_TWATCH_ULTRA)
    return instance.getPoint(x_arr, y_arr, max_pts);
#else
    (void)x_arr; (void)y_arr; (void)max_pts;
    return 0;
#endif
}