#pragma once
#include <math.h>
#include <stdint.h>
#include "Config.h"

namespace Radar {
enum class Mode { Search, Tracking };
enum class State { Scan, Track, Waiting };
enum class Sound { Normal, Found, Waiting };

inline bool validDistance(float cm) {
    return isfinite(cm) && cm >= Config::MIN_VALID_CM && cm <= Config::MAX_VALID_CM;
}
inline bool detected(float cm) {
    return validDistance(cm) && cm <= Config::DETECT_CM;
}
inline int clampAngle(int angle) {
    if (angle < Config::SERVO_MIN_DEG) return Config::SERVO_MIN_DEG;
    if (angle > Config::SERVO_MAX_DEG) return Config::SERVO_MAX_DEG;
    return angle;
}
inline bool buzzerOn(Sound sound, uint32_t elapsedMs) {
    const uint32_t period = sound == Sound::Found ? Config::FOUND_BEEP_PERIOD_MS
                                                  : Config::NORMAL_BEEP_PERIOD_MS;
    const uint32_t phase = elapsedMs % period;
    return phase < Config::BEEP_MS ||
           (sound == Sound::Waiting && phase >= Config::DOUBLE_BEEP_SECOND_MS &&
            phase < Config::DOUBLE_BEEP_SECOND_MS + Config::BEEP_MS);
}

class ButtonDebouncer {
public:
    bool update(uint32_t now, bool pressed, bool edge) {
        if (edge) edgeSeen_ = true;
        if (pressed != raw_) {
            raw_ = pressed;
            changedAt_ = now;
        }
        if (uint32_t(now - changedAt_) < Config::BUTTON_DEBOUNCE_MS) return false;
        if (!pressed) {
            armed_ = true;
            edgeSeen_ = false;
            return false;
        }
        if (armed_ && edgeSeen_) {
            armed_ = false;
            edgeSeen_ = false;
            return true;
        }
        return false;
    }
private:
    bool raw_ = false;
    bool armed_ = true;
    bool edgeSeen_ = false;
    uint32_t changedAt_ = 0;
};

class Controller {
public:
    void begin() {
        angle_ = Config::SERVO_HOME_DEG;
        direction_ = 1;
        setMode(Mode::Search);
    }
    void setMode(Mode mode) {
        mode_ = mode;
        state_ = State::Scan;
        radar_ = false;
        ultrasonicFound_ = false;
        distance_ = NAN;
        phase_ = Probe::Left;
        left_ = right_ = NAN;
        center_ = angle_;
        missedProbeCycles_ = 0;
    }
    void setRadarPresence(bool presence) {
        radar_ = mode_ == Mode::Tracking && presence;
        if (mode_ == Mode::Tracking && !radar_ && state_ != State::Scan) {
            state_ = State::Scan;
            left_ = right_ = NAN;
            missedProbeCycles_ = 0;
        }
    }
    void onDistance(float cm) {
        distance_ = validDistance(cm) ? cm : NAN;
        ultrasonicFound_ = detected(cm);
        if (mode_ == Mode::Search || !radar_) {
            state_ = State::Scan;
            advanceScan();
            return;
        }
        if (state_ != State::Track) {
            if (ultrasonicFound_) {
                state_ = State::Track;
                center_ = angle_;
                missedProbeCycles_ = 0;
                startProbe();
            } else {
                state_ = State::Waiting;
            }
            return;
        }
        if (phase_ == Probe::Left) {
            left_ = distance_;
            phase_ = Probe::Right;
            angle_ = clampAngle(center_ + trackingProbeDegrees());
        } else if (phase_ == Probe::Right) {
            right_ = distance_;
            phase_ = Probe::Center;
            angle_ = center_;
        } else {
            finishProbe(distance_);
        }
    }
    Mode mode() const { return mode_; }
    State state() const { return state_; }
    int angle() const { return angle_; }
    int trackingCenter() const { return center_; }
    unsigned trackingMissCycles() const { return missedProbeCycles_; }
    int trackingProbeDegrees() const {
        const int radius = Config::TRACK_PROBE_DEG +
                           int(missedProbeCycles_) * Config::TRACK_RECOVERY_EXPAND_DEG;
        return radius > Config::TRACK_RECOVERY_MAX_DEG ? Config::TRACK_RECOVERY_MAX_DEG : radius;
    }
    float distance() const { return distance_; }
    bool radarPresence() const { return radar_; }
    bool targetFound() const {
        return mode_ == Mode::Search ? ultrasonicFound_ : state_ == State::Track;
    }
    Sound sound() const {
        if (mode_ == Mode::Tracking && state_ == State::Waiting) return Sound::Waiting;
        return targetFound() ? Sound::Found : Sound::Normal;
    }
private:
    enum class Probe { Left, Right, Center };
    void advanceScan() {
        if (angle_ >= Config::SERVO_MAX_DEG) direction_ = -1;
        else if (angle_ <= Config::SERVO_MIN_DEG) direction_ = 1;
        angle_ = clampAngle(angle_ + direction_ * Config::SCAN_STEP_DEG);
    }
    void startProbe() {
        left_ = right_ = NAN;
        phase_ = Probe::Left;
        angle_ = clampAngle(center_ - trackingProbeDegrees());
    }
    void finishProbe(float centerDistance) {
        const bool centerFound = detected(centerDistance);
        float best = centerFound ? centerDistance : INFINITY;
        int bestAngle = center_;
        if (detected(left_) && left_ + Config::TRACK_IMPROVEMENT_CM < best) {
            best = left_;
            bestAngle = clampAngle(center_ - trackingProbeDegrees());
        }
        if (detected(right_) && right_ + Config::TRACK_IMPROVEMENT_CM < best) {
            best = right_;
            bestAngle = clampAngle(center_ + trackingProbeDegrees());
        }
        if (!isfinite(best)) {
            ++missedProbeCycles_;
            if (missedProbeCycles_ < Config::TRACK_LOST_PROBE_CYCLES) {
                startProbe();
            } else {
                state_ = State::Waiting;
                angle_ = center_;
            }
            return;
        }
        missedProbeCycles_ = 0;
        int offset = bestAngle - center_;
        if (centerFound) {
            if (offset > Config::TRACK_STEP_DEG) offset = Config::TRACK_STEP_DEG;
            if (offset < -Config::TRACK_STEP_DEG) offset = -Config::TRACK_STEP_DEG;
        }
        center_ = clampAngle(center_ + offset);
        startProbe();
    }
    Mode mode_ = Mode::Search;
    State state_ = State::Scan;
    Probe phase_ = Probe::Left;
    int angle_ = Config::SERVO_HOME_DEG;
    int direction_ = 1;
    int center_ = Config::SERVO_HOME_DEG;
    unsigned missedProbeCycles_ = 0;
    bool radar_ = false;
    bool ultrasonicFound_ = false;
    float distance_ = NAN;
    float left_ = NAN;
    float right_ = NAN;
};
}
