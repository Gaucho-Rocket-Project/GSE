// -*- mode: c++ -*-
/* Version stand-in: select IDF 4.4, 5.1, 5.4 or 5.5 in host checks.
 * IDF 5.0 changed the receive callback; 5.5 changed the send callback. */
#ifndef ESP_IDF_VERSION_H_SYNTAX_STUB
#define ESP_IDF_VERSION_H_SYNTAX_STUB

#define ESP_IDF_VERSION_VAL(major, minor, patch) (((major) << 16) | ((minor) << 8) | (patch))

#if defined(ARDUINO_ESP32_CORE2)
#  define ESP_IDF_VERSION_MAJOR 4
#  define ESP_IDF_VERSION_MINOR 4
#  define ESP_IDF_VERSION_PATCH 7
#elif defined(GSE_TEST_IDF_5_5)
#  define ESP_IDF_VERSION_MAJOR 5
#  define ESP_IDF_VERSION_MINOR 5
#  define ESP_IDF_VERSION_PATCH 0
#elif defined(GSE_TEST_IDF_5_4)
#  define ESP_IDF_VERSION_MAJOR 5
#  define ESP_IDF_VERSION_MINOR 4
#  define ESP_IDF_VERSION_PATCH 2
#else
#  define ESP_IDF_VERSION_MAJOR 5
#  define ESP_IDF_VERSION_MINOR 1
#  define ESP_IDF_VERSION_PATCH 4
#endif

#define ESP_IDF_VERSION ESP_IDF_VERSION_VAL(ESP_IDF_VERSION_MAJOR, ESP_IDF_VERSION_MINOR, ESP_IDF_VERSION_PATCH)

#endif /* ESP_IDF_VERSION_H_SYNTAX_STUB */
