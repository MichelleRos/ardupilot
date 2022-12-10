#!/bin/sh

EXTRA_HWDEF="test-extra.hwdef"
BOARD=MatekF405
# BOARD=revo-mini

cat >"$EXTRA_HWDEF" <<EOF
define GPS_BACKEND_DEFAULT_ENABLED 0
define GPS_BACKEND_UBLOX_ENABLED 1

undef HAL_PROBE_EXTERNAL_I2C_BAROS
define HAL_PROBE_EXTERNAL_I2C_BAROS 0

#undef AP_BARO_BMP085_ENABLED
#define AP_BARO_BMP085_ENABLED 0
#undef AP_BARO_BMP280_ENABLED
#define AP_BARO_BMP280_ENABLED 1

undef AP_BARO_MS56XX_ENABLED
define AP_BARO_MS6XX_ENABLED 0

undef AP_RCPROTOCOL_BACKEND_DEFAULT_ENABLED
define AP_RCPROTOCOL_BACKEND_DEFAULT_ENABLED 0
define AP_RCPROTOCOL_SBUS_ENABLED 1

define GPS_MAX_RECEIVERS 1

define HAL_BUTTON_ENABLED 0

define AP_RPM_ENABLED 0
define AP_ICENGINE_ENABLED 0

BOOTLOADER_EMBED 0

define AP_ADVANCEDFAILSAFE_ENABLED 0

define HAL_MAVLINK_INTERVALS_FROM_FILES_ENABLED 0

define HAL_SUPPORT_RCOUT_SERIAL 0

undef HAL_HOTT_TELEM_ENABLED
define HAL_HOTT_TELEM_ENABLED 0

undef HAL_SPEKTRUM_TELEM_ENABLED
define HAL_SPEKTRUM_TELEM_ENABLED 0

EOF

# ./Tools/autotest/test_build_options.py \
#     --no-run-with-defaults \
#     --no-disable-none \
#     --no-disable-in-turn \
#     --board="$BOARD" \
#     --extra-hwdef="$EXTRA_HWDEF"

./waf configure --board "$BOARD" --extra-hwdef="$EXTRA_HWDEF"
./waf copter