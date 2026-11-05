# ESP32-S3 dual SG90 servo test firmware - main component
#
# esp_driver_ledc  : hardware PWM (50 Hz servo pulses)
# esp_driver_gpio  : GPIO numbering
# esp_driver_uart / esp_driver_usb_serial_jtag : console input
# esp_timer, freertos are pulled in by the build system as needed.

idf_component_register(
    SRCS "app_main.c" "servo.c" "console_io.c" "position.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES esp_driver_ledc esp_driver_gpio esp_driver_uart esp_driver_usb_serial_jtag
)

# ---------------------------------------------------------------- servo type
# Select the servo in use without editing any source file:
#   make build SERVO_TYPE=2                      # DS3225MG, 270 deg version
#   make build SERVO_TYPE=2 SERVO_TRAVEL_DEG=180 # DS3225MG, 180 deg version
#   1 = SG90 / micro analog (default), 2 = DS3225MG 25 kg digital
set(SERVO_TYPE       "1" CACHE STRING "1=SG90/micro analog, 2=DS3225MG digital")
set(SERVO_TRAVEL_DEG ""  CACHE STRING "mechanical travel in degrees (blank = type default)")

target_compile_definitions(${COMPONENT_LIB} PRIVATE SERVO_TYPE=${SERVO_TYPE})
if(NOT SERVO_TRAVEL_DEG STREQUAL "")
    target_compile_definitions(${COMPONENT_LIB} PRIVATE SERVO_TRAVEL_DEG=${SERVO_TRAVEL_DEG})
endif()
