/* Host integration test: real TX core + ESP-NOW adapter + real pad loop.
 * Radio, Arduino I/O and clock are stubs; this is not a hardware/range test. */
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <vector>

#define GSE_TEST_RUNTIME
#define GSE_ROLE_PAD
#ifndef GSE_TEST_PLACEHOLDER
#define GSE_NOW_PAD_MAC { 0x24, 0x11, 0x22, 0x33, 0x44, 0x02 }
#define GSE_NOW_TX_MAC  { 0x24, 0x11, 0x22, 0x33, 0x44, 0x01 }
#endif
#include "../firmware/src/pad/pad_node.cpp"
#include "gse_hal.h"
#include "gse_tx_core.h"

HardwareSerial Serial(0), Serial1(1), Serial2(2);
static uint32_t clock_ms = 1000;
static int pins[64];
static uint32_t duties[64];
static bool cut_air, fail_pmk;
static esp_now_recv_cb_t recv_callback;
static const uint8_t pad_mac[] = GSE_NOW_PAD_MAC;
static const uint8_t tx_mac[] = GSE_NOW_TX_MAC;
static GseLink tx_link;
static std::vector<uint8_t> host_in, host_out;
static size_t host_pos;
static int errors;

#define CHECK(x) do { if (!(x)) { printf("FAIL line %d: %s\n", __LINE__, #x); ++errors; } else printf("  ok %s\n", #x); } while (0)

unsigned long millis() { return clock_ms; }
unsigned long micros() { return clock_ms * 1000; }
void delay(unsigned long n) { clock_ms += n; }
void delayMicroseconds(unsigned int) {}
void pinMode(uint8_t, uint8_t) {}
void digitalWrite(uint8_t p, uint8_t v) { pins[p] = v; }
int digitalRead(uint8_t p) { return pins[p]; }
int analogRead(uint8_t) { return 0; }
void analogWrite(uint8_t, int) {}
bool ledcAttach(uint8_t, uint32_t, uint8_t) { return true; }
bool ledcWrite(uint8_t p, uint32_t d) { duties[p] = d; return true; }
bool ledcDetach(uint8_t) { return true; }
void rgbLedWrite(uint8_t, uint8_t, uint8_t, uint8_t) {}
void portENTER_CRITICAL(portMUX_TYPE *) {}
void portEXIT_CRITICAL(portMUX_TYPE *) {}
esp_err_t esp_task_wdt_reconfigure(const esp_task_wdt_config_t *) { return ESP_OK; }
esp_err_t esp_task_wdt_add(void *) { return ESP_OK; }
esp_err_t esp_task_wdt_reset() { return ESP_OK; }
esp_err_t esp_netif_init() { return ESP_OK; }
esp_err_t esp_event_loop_create_default() { return ESP_OK; }
esp_err_t esp_wifi_init(const wifi_init_config_t *) { return ESP_OK; }
esp_err_t esp_wifi_set_storage(wifi_storage_t) { return ESP_OK; }
esp_err_t esp_wifi_set_mode(wifi_mode_t) { return ESP_OK; }
esp_err_t esp_wifi_start() { return ESP_OK; }
esp_err_t esp_wifi_get_mac(wifi_interface_t, uint8_t *out) { memcpy(out, pad_mac, 6); return ESP_OK; }
esp_err_t esp_wifi_set_channel(uint8_t, wifi_second_chan_t) { return ESP_OK; }
esp_err_t esp_now_init() { return ESP_OK; }
esp_err_t esp_now_set_pmk(const uint8_t *) { return fail_pmk ? ESP_FAIL : ESP_OK; }
esp_err_t esp_now_register_recv_cb(esp_now_recv_cb_t cb) { recv_callback = cb; return ESP_OK; }
esp_err_t esp_now_register_send_cb(esp_now_send_cb_t) { return ESP_OK; }
esp_err_t esp_now_add_peer(const esp_now_peer_info_t *p) { return p->encrypt ? ESP_OK : ESP_FAIL; }
esp_err_t esp_now_send(const uint8_t *dst, const uint8_t *data, size_t n)
{
    if (!memcmp(dst, pad_mac, 6)) {
        if (!cut_air && recv_callback) {
            wifi_pkt_rx_ctrl_t rx = {}; rx.rssi = -50;
            esp_now_recv_info_t info = {};
            info.src_addr = const_cast<uint8_t *>(tx_mac);
            info.rx_ctrl = &rx;
            recv_callback(&info, data, (int)n);
        }
    } else if (!memcmp(dst, tx_mac, 6)) {
        tx_link.onDatagram(pad_mac, data, (int)n, -50);
    } else return ESP_FAIL;
    return ESP_OK;
}

extern "C" {
uint32_t gse_millis() { return clock_ms; }
void gse_tx_write(const uint8_t *b, size_t n) { tx_link.sendRaw(b, n); }
bool gse_tx_read_byte(uint8_t *out) { int c = tx_link.read(); if (c < 0) return false; *out = (uint8_t)c; return true; }
void gse_host_write(const uint8_t *b, size_t n) { host_out.insert(host_out.end(), b, b+n); }
bool gse_host_read_byte(uint8_t *out) { if (host_pos == host_in.size()) return false; *out = host_in[host_pos++]; return true; }
void gse_log(const char *, ...) {}
bool gse_radio_provision(uint8_t, uint8_t, uint16_t, uint16_t) { return tx_link.linkUp(); }
}

#ifndef GSE_TEST_PLACEHOLDER
static void advance(unsigned ms)
{
    for (unsigned i = 0; i < ms; ++i) { ++clock_ms; gse_tx_tick(); loop(); }
}
static void command(const gse_cmd_t &cmd, unsigned duration)
{
    static uint8_t seq;
    uint8_t buf[GSE_MAX_FRAME];
    for (unsigned i = 0; i < duration; i += 50) {
        size_t n = gse_encode_cmd(seq++, &cmd, buf);
        host_in.insert(host_in.end(), buf, buf+n);
        advance(50);
    }
}
static gse_state_t state;
static bool got_state;
static void on_host(uint8_t type, uint8_t, const uint8_t *payload, uint8_t len, void *)
{
    if (type == GSE_T_STATE && len == sizeof state) { memcpy(&state, payload, len); got_state = true; }
}
static void parse_host()
{
    gse_parser_t p; gse_parser_init(&p);
    for (uint8_t c : host_out) gse_parser_push(&p, c, on_host, nullptr);
    host_out.clear();
}
#endif
int main()
{
    setup();
#ifdef GSE_TEST_PLACEHOLDER
    CHECK(!gseLink.linkUp());
    CHECK(gseLink.provision(0, 0, 0, 0, 0).length() != 0);
    CHECK(strstr(gseLink.describe().c_str(), "MAC NOT CONFIGURED") != nullptr);
    CHECK(pins[PIN_RELAY_IGN] == LOW);
    CHECK(duties[PIN_SERVO_FILL] == US_TO_DUTY(1000));
    CHECK(duties[PIN_SERVO_VEH] == US_TO_DUTY(2000));
    printf(errors ? "%d FAILED\n" : "ALL PASS\n", errors);
    return errors ? 1 : 0;
#else
    CHECK(gseLink.linkUp());
    CHECK(recv_callback != nullptr);
    CHECK(!memcmp(gseLink.localMac, pad_mac, 6));
    CHECK(pins[PIN_RELAY_IGN] == LOW);
    CHECK(duties[PIN_SERVO_FILL] == US_TO_DUTY(1000));
    CHECK(duties[PIN_SERVO_VEH] == US_TO_DUTY(2000));
    memcpy(tx_link.peerMac, pad_mac, 6);
    tx_link.peerConfigured = tx_link.peerRegistered = tx_link.ready = true;
    gse_tx_init();
    gse_cmd_t cmd = {}; cmd.veh = 1; cmd.wdg_ds = 10; cmd.ign_max_ds = 5;
    command(cmd, 200);
    cmd.fill = 1; cmd.veh = 0; command(cmd, 150);
    CHECK(duties[PIN_SERVO_FILL] == US_TO_DUTY(2000));
    CHECK(duties[PIN_SERVO_VEH] == US_TO_DUTY(1000));
    parse_host(); CHECK(got_state && state.out_fill == 1 && state.out_veh == 0);
    cmd.fill = 0; cmd.veh = 1; command(cmd, 150);
    CHECK(duties[PIN_SERVO_FILL] == US_TO_DUTY(1000));
    CHECK(duties[PIN_SERVO_VEH] == US_TO_DUTY(2000));
    gse_cfgs_t config = {45, 45, 45, 10, 5, 0, 1, 0};
    uint8_t cfg_frame[GSE_MAX_FRAME];
    size_t cfg_len = gse_encode_cfg(1, &config, cfg_frame);
    host_in.insert(host_in.end(), cfg_frame, cfg_frame + cfg_len);
    advance(30);
    cmd.prg = 1; command(cmd, 150); parse_host();
    CHECK(duties[PIN_SERVO_PRG] == US_TO_DUTY(1000) && state.out_prg == 0);
    config.allow_purge = 1;
    cfg_len = gse_encode_cfg(2, &config, cfg_frame);
    host_in.insert(host_in.end(), cfg_frame, cfg_frame + cfg_len);
    advance(30); command(cmd, 150); parse_host();
    CHECK(duties[PIN_SERVO_PRG] == US_TO_DUTY(2000) && state.out_prg == 1);
    cmd.prg = 0;
    cmd.ign = 1; command(cmd, 150);
    CHECK(pins[PIN_RELAY_IGN] == LOW); /* unarmed */
    cmd.arm_code = GSE_ARM_CODE; command(cmd, 150);
    CHECK(pins[PIN_RELAY_IGN] == HIGH);
    command(cmd, 650); CHECK(pins[PIN_RELAY_IGN] == LOW); /* hard one-shot limit */
    parse_host(); CHECK(state.out_ign == 0 && !(state.flags & GSE_ST_IGN_ON));
    command(cmd, 150); CHECK(pins[PIN_RELAY_IGN] == LOW); /* no repeat while held */
    cmd.ign = 0; command(cmd, 100); cmd.ign = 1; command(cmd, 100);
    CHECK(pins[PIN_RELAY_IGN] == HIGH); /* release then reassert */
    cmd.arm_code = 0; command(cmd, 100); CHECK(pins[PIN_RELAY_IGN] == LOW);
    cmd.fill = 1; cmd.veh = 0; cmd.arm_code = GSE_ARM_CODE; cmd.ign = 1;
    command(cmd, 100); CHECK(pins[PIN_RELAY_IGN] == HIGH);
    cut_air = true; command(cmd, 1200); /* GUI alive but radio lost */
    CHECK(pins[PIN_RELAY_IGN] == LOW);
    CHECK(duties[PIN_SERVO_FILL] == US_TO_DUTY(1000));
    CHECK(duties[PIN_SERVO_VEH] == US_TO_DUTY(2000));
    parse_host(); CHECK(state.flags & GSE_ST_COMMS_LOSS);
    cut_air = false;
    snprintf(cfgFault, sizeof cfgFault, "test link fault");
    uint16_t rx_before_fault = rxCount;
    command(cmd, 150);
    CHECK(rxCount == rx_before_fault && pins[PIN_RELAY_IGN] == LOW);
    CHECK(duties[PIN_SERVO_FILL] == US_TO_DUTY(1000));
    CHECK(duties[PIN_SERVO_VEH] == US_TO_DUTY(2000));
    uint8_t packet[] = {1, 2, 3};
    unsigned long before = gseLink.foreignCount();
    gseLink.onDatagram(pad_mac, packet, sizeof packet, -50);
    gseLink.onDatagram(nullptr, packet, sizeof packet, -50);
    CHECK(gseLink.foreignCount() == before + 2 && gseLink.available() == 0);
    uint8_t big[250]; memset(big, 0x55, sizeof big);
    gseLink.onDatagram(tx_mac, big, sizeof big, -50);
    uint32_t dropped_before = gseLink.dropped;
    gseLink.onDatagram(tx_mac, packet, sizeof packet, -50);
    gseLink.onDatagram(tx_mac, packet, sizeof packet, -50);
    CHECK(gseLink.dropped == dropped_before + sizeof packet);
    CHECK(gseLink.available() == 253 && gseLink.read() == 0x55);
    GseLink failed;
    fail_pmk = true; failed.begin();
    CHECK(!failed.linkUp() && failed.provision(0, 0, 0, 0, 0).length() != 0);
    printf(errors ? "%d FAILED\n" : "ALL PASS\n", errors);
    return errors ? 1 : 0;
#endif
}
