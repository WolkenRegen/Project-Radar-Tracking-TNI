#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "Config.h"
#include "RadarController.h"
#include "RadarScope.h"

namespace {
Radar::Controller controller;
RadarScope::History scopeHistory;
Radar::ButtonDebouncer button;
Adafruit_SSD1306 display(128, 64, &Wire, -1);
bool displayReady = false;

portMUX_TYPE interruptMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool buttonEdge = false;
volatile bool echoArmed = false;
volatile bool echoRisen = false;
volatile bool echoComplete = false;
volatile uint32_t echoRiseUs = 0;
volatile uint32_t echoWidthUs = 0;

bool pingInFlight = false;
uint32_t pingStartedUs = 0;
uint32_t lastPingMs = 0;
int pingAngle = 0;
int servoAngle = Config::SERVO_HOME_DEG;
uint32_t servoMovedAt = 0;
uint32_t servoSettleMs = Config::SERVO_HOME_SETTLE_MS;
bool radarRaw = false;
bool radarStable = false;
uint32_t radarChangedAt = 0;
uint32_t splashStartedAt = 0;
bool splashActive = true;
bool displayDirty = true;
uint32_t lastDisplayAt = 0;
Radar::Sound sound = Radar::Sound::Normal;
uint32_t soundStartedAt = 0;
bool buzzerHigh = false;

void IRAM_ATTR onButtonInterrupt() {
    portENTER_CRITICAL_ISR(&interruptMux);
    buttonEdge = true;
    portEXIT_CRITICAL_ISR(&interruptMux);
}

void IRAM_ATTR onEchoInterrupt() {
    const uint32_t now = micros();
    const bool high = digitalRead(Config::ECHO_PIN) == HIGH;
    portENTER_CRITICAL_ISR(&interruptMux);
    if (echoArmed) {
        if (high && !echoRisen) {
            echoRiseUs = now;
            echoRisen = true;
        } else if (!high && echoRisen) {
            echoWidthUs = uint32_t(now - echoRiseUs);
            echoComplete = true;
            echoArmed = false;
        }
    }
    portEXIT_CRITICAL_ISR(&interruptMux);
}

void cancelPing() {
    portENTER_CRITICAL(&interruptMux);
    echoArmed = false;
    echoRisen = false;
    echoComplete = false;
    portEXIT_CRITICAL(&interruptMux);
    pingInFlight = false;
}

void writeServoAngle(int angle) {
    const uint32_t pulseUs = Config::SERVO_MIN_US +
                             (Config::SERVO_MAX_US - Config::SERVO_MIN_US) * uint32_t(angle) / 180U;
    const uint32_t duty = uint32_t((uint64_t(pulseUs) * 65535U) / 20000U);
    ledcWrite(Config::SERVO_PWM_CHANNEL, duty);
}

void synchronizeServo(uint32_t now) {
    const int next = controller.angle();
    if (next == servoAngle) return;
    cancelPing();
    const int delta = abs(next - servoAngle);
    servoAngle = next;
    writeServoAngle(next);
    servoMovedAt = now;
    servoSettleMs = Config::SERVO_SETTLE_BASE_MS +
                    uint32_t(delta) * Config::SERVO_SETTLE_PER_DEG_MS;
}

void setMode(Radar::Mode mode, uint32_t now) {
    cancelPing();
    controller.setMode(mode);
    scopeHistory.clear();
    radarRaw = radarStable = false;
    radarChangedAt = now;
    splashStartedAt = now;
    splashActive = true;
    displayDirty = true;
}

void handleModeButton(uint32_t now) {
    portENTER_CRITICAL(&interruptMux);
    const bool edge = buttonEdge;
    buttonEdge = false;
    portEXIT_CRITICAL(&interruptMux);
    if (button.update(now, digitalRead(Config::BUTTON_PIN) == LOW, edge)) {
        setMode(controller.mode() == Radar::Mode::Search ? Radar::Mode::Tracking
                                                         : Radar::Mode::Search,
                now);
    }
}

void updateRadarSensor(uint32_t now) {
    if (controller.mode() == Radar::Mode::Search) {
        controller.setRadarPresence(false);
        return;
    }
    const bool reading = digitalRead(Config::RADAR_OUT_PIN) == HIGH;
    if (reading != radarRaw) {
        radarRaw = reading;
        radarChangedAt = now;
    }
    if (uint32_t(now - radarChangedAt) >= Config::RADAR_DEBOUNCE_MS) radarStable = radarRaw;
    controller.setRadarPresence(radarStable);
}

void acceptDistance(float cm, int measuredAngle, uint32_t now) {
    controller.onDistance(cm);
    scopeHistory.record(controller.mode(), controller.state(), controller.radarPresence(),
                        measuredAngle, cm, now);
}

void startPing(uint32_t now) {
    if (digitalRead(Config::ECHO_PIN) == HIGH) {
        lastPingMs = now;
        acceptDistance(NAN, servoAngle, now);
        return;
    }
    portENTER_CRITICAL(&interruptMux);
    echoRisen = false;
    echoComplete = false;
    echoArmed = true;
    portEXIT_CRITICAL(&interruptMux);
    pingAngle = servoAngle;
    lastPingMs = now;
    pingStartedUs = micros();
    pingInFlight = true;
    digitalWrite(Config::TRIG_PIN, HIGH);
    delayMicroseconds(12);
    digitalWrite(Config::TRIG_PIN, LOW);
}

void updateUltrasonic(uint32_t now) {
    if (pingInFlight) {
        const bool expired = uint32_t(micros() - pingStartedUs) >= Config::ECHO_TIMEOUT_US;
        portENTER_CRITICAL(&interruptMux);
        const bool complete = echoComplete;
        const uint32_t width = echoWidthUs;
        const uint32_t rise = echoRiseUs;
        if (complete || expired) {
            echoArmed = false;
            echoComplete = false;
        }
        portEXIT_CRITICAL(&interruptMux);
        if (complete || expired) {
            pingInFlight = false;
            const float cm = complete && width > 0 &&
                                     uint32_t(rise + width - pingStartedUs) <= Config::ECHO_TIMEOUT_US
                                 ? float(width) / 58.0f
                                 : NAN;
            if (pingAngle == servoAngle) {
                acceptDistance(cm, pingAngle, now);
            }
        }
        return;
    }
    if (uint32_t(now - servoMovedAt) >= servoSettleMs &&
        uint32_t(now - lastPingMs) >= Config::PING_INTERVAL_MS)
        startPing(now);
}

void setBuzzerOutput(bool on) {
    if (on == buzzerHigh) return;
    buzzerHigh = on;
    if (Config::ACTIVE_BUZZER) {
        digitalWrite(Config::BUZZER_PIN, (on == Config::BUZZER_ACTIVE_HIGH) ? HIGH : LOW);
    } else {
        ledcWriteTone(Config::BUZZER_PWM_CHANNEL, on ? Config::BUZZER_FREQUENCY_HZ : 0);
    }
}

void updateBuzzer(uint32_t now) {
    const Radar::Sound next = controller.sound();
    if (next != sound) {
        sound = next;
        soundStartedAt = now;
    }
    setBuzzerOutput(Radar::buzzerOn(sound, uint32_t(now - soundStartedAt)));
}

void showModeSplash() {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(2);
    const char *title = controller.mode() == Radar::Mode::Search ? "SEARCH" : "TRACKING";
    const int width = int(strlen(title)) * 12;
    display.setCursor((128 - width) / 2, 24);
    display.print(title);
    display.display();
}

void updateDisplay(uint32_t now) {
    if (!displayReady || pingInFlight) return;
    if (splashActive) {
        if (uint32_t(now - splashStartedAt) < Config::SPLASH_MS) {
            if (displayDirty) {
                showModeSplash();
                displayDirty = false;
            }
            return;
        }
        splashActive = false;
        displayDirty = true;
    }
    if (!displayDirty && uint32_t(now - lastDisplayAt) < Config::DISPLAY_INTERVAL_MS) return;
    lastDisplayAt = now;
    displayDirty = false;
    RadarScope::draw(display, controller, scopeHistory, servoAngle, now);
    display.display();
}

void initializeDisplay() {
    if (!Wire.begin(Config::OLED_SDA_PIN, Config::OLED_SCL_PIN, 400000)) return;
    Wire.setTimeOut(20);
    const uint8_t addresses[] = {Config::OLED_ADDRESS, 0x3D};
    for (uint8_t address : addresses) {
        Wire.beginTransmission(address);
        if (Wire.endTransmission() != 0) continue;
        displayReady = display.begin(SSD1306_SWITCHCAPVCC, address, false, false);
        if (displayReady) {
            display.setTextWrap(false);
            return;
        }
    }
}

}

void setup() {

    pinMode(Config::TRIG_PIN, OUTPUT);
    digitalWrite(Config::TRIG_PIN, LOW);
    pinMode(Config::ECHO_PIN, INPUT);
    pinMode(Config::RADAR_OUT_PIN, INPUT_PULLDOWN);
    pinMode(Config::BUTTON_PIN, INPUT_PULLUP);
    pinMode(Config::BUZZER_PIN, OUTPUT);
    digitalWrite(Config::BUZZER_PIN, Config::BUZZER_ACTIVE_HIGH ? LOW : HIGH);

    if (!Config::ACTIVE_BUZZER) {
        ledcSetup(Config::BUZZER_PWM_CHANNEL, Config::BUZZER_FREQUENCY_HZ, 8);
        ledcAttachPin(Config::BUZZER_PIN, Config::BUZZER_PWM_CHANNEL);
        ledcWrite(Config::BUZZER_PWM_CHANNEL, 0);
    }

    ledcSetup(Config::SERVO_PWM_CHANNEL, 50, 16);
    ledcAttachPin(Config::SERVO_PIN, Config::SERVO_PWM_CHANNEL);
    controller.begin();
    servoAngle = controller.angle();
    writeServoAngle(servoAngle);

    attachInterrupt(digitalPinToInterrupt(Config::ECHO_PIN), onEchoInterrupt, CHANGE);
    attachInterrupt(digitalPinToInterrupt(Config::BUTTON_PIN), onButtonInterrupt, FALLING);

    initializeDisplay();
    const uint32_t now = millis();
    servoMovedAt = now;
    servoSettleMs = Config::SERVO_HOME_SETTLE_MS;
    splashStartedAt = soundStartedAt = now;
    lastPingMs = now;

    updateDisplay(now);
}

void loop() {
    const uint32_t now = millis();
    handleModeButton(now);
    updateRadarSensor(now);
    updateUltrasonic(now);
    synchronizeServo(now);
    updateBuzzer(now);
    updateDisplay(now);

    if (!pingInFlight) delay(1);
}
