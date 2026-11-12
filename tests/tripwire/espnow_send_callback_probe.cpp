/* IDF 5.5 must reject the old send callback shape. This file is expected to
 * fail compilation and guards against the syntax stub losing that API change. */
#include <esp_now.h>

static void old_send_callback(const uint8_t *, esp_now_send_status_t) {}

void register_old_send_callback()
{
    esp_now_register_send_cb(old_send_callback);
}
