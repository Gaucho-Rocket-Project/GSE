#include "config.h"
#if defined(EXPECT_NOW) && !defined(GSE_LINK_ESPNOW)
#error "ESP-NOW should be selected"
#endif
#if defined(EXPECT_LORA) && !defined(GSE_LINK_LORA)
#error "LoRa should be selected"
#endif
