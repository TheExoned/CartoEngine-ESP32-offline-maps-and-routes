// =====================================================================
// main_standalone_ce.cpp — LANZADOR STANDALONE DE CARTOENGINE (EXOSYS)
// Arranca únicamente el Motor Cartográfico (.QOI) y Ruteador A* (.HWY)
// =====================================================================
#include <Arduino.h>
#include "ExosysBSP.h"
#include "CartoEngine.h"
#include <TinyGPS++.h>

SemaphoreHandle_t lvgl_mutex = NULL;
SemaphoreHandle_t i2c_mutex  = NULL;
SdFs sd;
HardwareSerial GPSSerial(1);
TinyGPSPlus gps;

extern void init_carto_engine();
extern void build_ui();
extern void carto_loop_handler();
extern bool hardware_back_router();
extern void actualizar_marcador_gps();

void setup() {
    Serial.begin(115200);
    pinMode(EXO_BTN_BACK_PIN, INPUT_PULLUP);

#if defined(BOARD_TWATCH_ULTRA)
    instance.setDisplayParams(true, false);
    if (!instance.begin()) {
        while (1) delay(100);
    }
    instance.setBrightness(255);
    beginLvglHelper(instance);
    if (!sd.begin(EXO_SD_CS_PIN, SD_SCK_MHZ(40))) {
        Serial.println("[!] Error montando MicroSD en pin CS=21");
    }
#elif defined(BOARD_TDISPLAY_S3)
    // En T-Display-S3 inicializa SPI en los pines libres para lector MicroSD externo
    SPI.begin(EXO_SD_SCLK_PIN, EXO_SD_MISO_PIN, EXO_SD_MOSI_PIN, EXO_SD_CS_PIN);
    if (!sd.begin(SdSpiConfig(EXO_SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(25)))) {
        Serial.println("[!] Inserta lector MicroSD SPI en pines 10/11/12/13");
    }
    // (Aquí se enlaza el driver LVGL de 170x320 del T-Display-S3)
#endif

    GPSSerial.setRxBufferSize(1024);
    GPSSerial.begin(EXO_GPS_BAUD, SERIAL_8N1, EXO_GPS_RX_PIN, EXO_GPS_TX_PIN);

    lvgl_mutex = xSemaphoreCreateRecursiveMutex();
    i2c_mutex  = xSemaphoreCreateMutex();

    init_carto_engine();

    if (xSemaphoreTakeRecursive(lvgl_mutex, portMAX_DELAY)) {
        build_ui(); // Carga directamente scr_mapa en pantalla
        xSemaphoreGiveRecursive(lvgl_mutex);
    }
}

void loop() {
    static bool last_btn = HIGH;
    bool cur_btn = digitalRead(EXO_BTN_BACK_PIN);
    if (last_btn == HIGH && cur_btn == LOW) {
        if (xSemaphoreTakeRecursive(lvgl_mutex, portMAX_DELAY)) {
            hardware_back_router();
            xSemaphoreGiveRecursive(lvgl_mutex);
        }
    }
    last_btn = cur_btn;

    static uint32_t backend_t = 0;
    if (millis() - backend_t > 40) {
        carto_loop_handler();
        if (xSemaphoreTakeRecursive(lvgl_mutex, portMAX_DELAY)) {
            actualizar_marcador_gps();
            xSemaphoreGiveRecursive(lvgl_mutex);
        }
        backend_t = millis();
    }

    if (xSemaphoreTakeRecursive(lvgl_mutex, portMAX_DELAY)) {
        lv_timer_handler();
        xSemaphoreGiveRecursive(lvgl_mutex);
    }
    delay(5);
}