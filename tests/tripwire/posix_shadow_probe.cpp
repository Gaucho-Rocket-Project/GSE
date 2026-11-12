/*
 * posix_shadow_probe.cpp  --  a deliberately broken file
 * ---------------------------------------------------------------------------
 * Part of tests/esp32_syntax_check.sh, which requires this file to FAIL
 * to compile.
 *
 * It reproduces, in eight lines, the one bug the host syntax check missed and
 * a real ESP-32S build caught:
 *
 *      error: 'GseLink link' redeclared as different kind of entity
 *      note:  previous declaration 'int link(const char *, const char *)'
 *
 * The real arduino-esp32 Arduino.h reaches <unistd.h> through IPAddress.h and
 * lwIP, so POSIX `link()` is visible at global scope in every firmware
 * translation unit.  A node object called `link` then collides with it, and
 * every `link.send()` fails.  tests/esp32_stub/Arduino.h now declares the same
 * POSIX names, which is what makes this probe fail -- and therefore what keeps
 * the check honest.
 *
 * If this file ever *compiles*, the stub has lost its POSIX declarations and
 * the host check is blind again.
 */
#include <Arduino.h>

static int link;      /* <- must collide with int link(const char*, const char*) */

int probe(void);
int probe(void) { return link; }
