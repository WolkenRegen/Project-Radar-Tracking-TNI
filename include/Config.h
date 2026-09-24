#pragma once
#include <stdint.h>

namespace Config {
constexpr uint8_t TRIG_PIN = 18;
constexpr uint8_t ECHO_PIN = 19; // ECHO requires a 5V-to-3.3V divider.
constexpr uint8_t RADAR_OUT_PIN = 27; // LD2410S OT2.
constexpr uint8_t BUTTON_PIN = 33;
constexpr uint8_t SERVO_PIN = 25;
constexpr uint8_t BUZZER_PIN = 26;
constexpr uint8_t OLED_SDA_PIN = 21;
constexpr uint8_t OLED_SCL_PIN = 22;
constexpr uint8_t OLED_ADDRESS = 0x3C;

constexpr float MIN_VALID_CM = 2.0f;
constexpr float MAX_VALID_CM = 400.0f;
constexpr float DETECT_CM = 60.0f;
constexpr uint32_t PING_INTERVAL_MS = 65;
constexpr uint32_t ECHO_TIMEOUT_US = 25000;
constexpr uint32_t RADAR_DEBOUNCE_MS = 40;
constexpr uint32_t BUTTON_DEBOUNCE_MS = 35;

constexpr int SERVO_MIN_DEG = 0;
constexpr int SERVO_MAX_DEG = 180;
constexpr int SERVO_HOME_DEG = 0;
constexpr int SCAN_STEP_DEG = 3;
constexpr int TRACK_PROBE_DEG = 8;
constexpr int TRACK_STEP_DEG = 6;
constexpr unsigned TRACK_LOST_PROBE_CYCLES = 3;
constexpr int TRACK_RECOVERY_EXPAND_DEG = 8;
constexpr int TRACK_RECOVERY_MAX_DEG = 24;
constexpr float TRACK_IMPROVEMENT_CM = 2.0f;

constexpr uint32_t SERVO_MIN_US = 500;
constexpr uint32_t SERVO_MAX_US = 2400;
constexpr uint32_t SERVO_HOME_SETTLE_MS = 800;
constexpr uint32_t SERVO_SETTLE_BASE_MS = 55;
constexpr uint32_t SERVO_SETTLE_PER_DEG_MS = 5;
constexpr uint8_t SERVO_PWM_CHANNEL = 0;

constexpr uint8_t BUZZER_PWM_CHANNEL = 2; // Separate timer from servo channels 0/1.

#ifdef RADAR_PASSIVE_BUZZER
constexpr bool ACTIVE_BUZZER = false;
#else
constexpr bool ACTIVE_BUZZER = true;
#endif
constexpr bool BUZZER_ACTIVE_HIGH = true;
constexpr uint32_t BUZZER_FREQUENCY_HZ = 2200;
constexpr uint32_t BEEP_MS = 80;
constexpr uint32_t DOUBLE_BEEP_SECOND_MS = 180;
constexpr uint32_t NORMAL_BEEP_PERIOD_MS = 1000;
constexpr uint32_t FOUND_BEEP_PERIOD_MS = 500;
constexpr uint32_t SPLASH_MS = 3000;
constexpr uint32_t DISPLAY_INTERVAL_MS = 150;

constexpr uint32_t SEARCH_ECHO_TTL_MS = 1800;
constexpr uint32_t SEARCH_ECHO_FRESH_MS = 450;
constexpr uint32_t TRACK_ECHO_TTL_MS = 1200;

static_assert(SERVO_MIN_DEG <= SERVO_HOME_DEG && SERVO_HOME_DEG <= SERVO_MAX_DEG,
              "Home must be inside the servo range");
static_assert(SERVO_MIN_DEG >= 0 && SERVO_MAX_DEG <= 180, "Servo range must be 0..180");
static_assert(SCAN_STEP_DEG > 0 && TRACK_STEP_DEG > 0 && TRACK_PROBE_DEG > 0,
              "Angular steps must be positive");
static_assert(PING_INTERVAL_MS >= 60, "Allow ultrasonic echoes to decay between pings");
static_assert(TRACK_LOST_PROBE_CYCLES > 0 && TRACK_LOST_PROBE_CYCLES <= 10,
              "Tracking loss confirmation must remain bounded");
static_assert(TRACK_RECOVERY_EXPAND_DEG > 0 && TRACK_RECOVERY_MAX_DEG >= TRACK_PROBE_DEG &&
                  TRACK_RECOVERY_MAX_DEG <= 180,
              "Recovery must cover a bounded angle range");
}
