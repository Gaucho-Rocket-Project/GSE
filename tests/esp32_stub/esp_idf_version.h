// -*- mode: c++ -*-
/*
 * esp_idf_version.h  --  stand-in for ESP-IDF's version header
 * ---------------------------------------------------------------------------
 * The real one is generated per IDF release.  The firmware's only use is
 * ESP_IDF_VERSION_MAJOR, to pick the ESP-NOW receive-callback signature and the
 * RSSI source, which genuinely differ between IDF 4.4 (arduino-esp32 2.x) and
 * IDF 5.x (3.x).
 *
 * tests/esp32_syntax_check.sh compiles every variant twice: once with
 * -D ARDUINO_ESP32_CORE2 and once without.  That flag selects 4 here, so the
 * 2.x run checks the 4.4 callback shape and the 3.x run checks the 5.x one.
 * Getting this wrong is exactly the class of bug the host check exists to catch.
 */
#ifndef ESP_IDF_VERSION_H_SYNTAX_STUB
#define ESP_IDF_VERSION_H_SYNTAX_STUB

#if defined(ARDUINO_ESP32_CORE2)
#  define ESP_IDF_VERSION_MAJOR 4
#  define ESP_IDF_VERSION_MINOR 4
#  define ESP_IDF_VERSION_PATCH 7
#else
#  define ESP_IDF_VERSION_MAJOR 5
#  define ESP_IDF_VERSION_MINOR 1
#  define ESP_IDF_VERSION_PATCH 4
#endif

#endif /* ESP_IDF_VERSION_H_SYNTAX_STUB */
