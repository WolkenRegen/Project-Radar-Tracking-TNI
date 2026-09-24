#pragma once
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "RadarController.h"

namespace RadarScope {
constexpr int WIDTH = 128;
constexpr int CENTER_X = 64;
constexpr int CENTER_Y = 53;
constexpr int RADIUS = 42;
constexpr uint16_t PIXEL_OFF = 0;
constexpr uint16_t PIXEL_ON = 1;
constexpr unsigned HISTORY_CAPACITY = 32;

struct Position {
    int x;
    int y;
};
struct Echo {
    int angle = 0;
    float cm = NAN;
    uint32_t measuredAt = 0;
    bool valid = false;
};

inline Position position(int angle, float cm) {
    const float radians = float(Radar::clampAngle(angle)) * 0.017453292519943295f;
    float distance = isfinite(cm) ? cm : 0.0f;
    if (distance < 0) distance = 0;
    if (distance > Config::DETECT_CM) distance = Config::DETECT_CM;
    const float radius = distance * float(RADIUS) / Config::DETECT_CM;
    return {CENTER_X + int(lroundf(radius * cosf(radians))),
            CENTER_Y - int(lroundf(radius * sinf(radians)))};
}

inline bool alive(const Echo &echo, uint32_t now, uint32_t ttl) {
    return echo.valid && uint32_t(now - echo.measuredAt) < ttl;
}

class History {
public:
    void clear() {
        for (auto &echo : echoes_)
            echo.valid = false;
        locked_.valid = false;
        next_ = 0;
    }
    void record(Radar::Mode mode, Radar::State state, bool radarPresence,
                int measuredAngle, float cm, uint32_t now) {
        if (mode != mode_) {
            clear();
            mode_ = mode;
        }
        if (mode == Radar::Mode::Search) {
            for (auto &echo : echoes_) {
                if (echo.valid && echo.angle == measuredAngle) echo.valid = false;
            }
            if (Radar::detected(cm)) {
                set(echoes_[next_], measuredAngle, cm, now);
                next_ = (next_ + 1) % HISTORY_CAPACITY;
            }
        } else if (state != Radar::State::Track || !radarPresence) {
            locked_.valid = false;
        } else if (Radar::detected(cm)) {
            set(locked_, measuredAngle, cm, now);
        }
    }
    const Echo &echo(unsigned index) const { return echoes_[index]; }
    const Echo &locked() const { return locked_; }
private:
    static void set(Echo &echo, int angle, float cm, uint32_t now) {
        echo.angle = angle;
        echo.cm = cm;
        echo.measuredAt = now;
        echo.valid = true;
    }
    Echo echoes_[HISTORY_CAPACITY];
    Echo locked_;
    unsigned next_ = 0;
    Radar::Mode mode_ = Radar::Mode::Search;
};

template <class Display>
void semiCircle(Display &display, int radius, bool dotted) {
    int x = 0, y = radius, error = 1 - radius;
    unsigned step = 0;
    while (x <= y) {
        if (!dotted || step % 2 == 0) {
            display.drawPixel(CENTER_X + x, CENTER_Y - y, PIXEL_ON);
            display.drawPixel(CENTER_X - x, CENTER_Y - y, PIXEL_ON);
            display.drawPixel(CENTER_X + y, CENTER_Y - x, PIXEL_ON);
            display.drawPixel(CENTER_X - y, CENTER_Y - x, PIXEL_ON);
        }
        ++step;
        ++x;
        if (error < 0) error += 2 * x + 1;
        else {
            --y;
            error += 2 * (x - y) + 1;
        }
    }
}

template <class Display>
void target(Display &display, const Echo &echo, bool locked, bool fresh) {
    const Position p = position(echo.angle, echo.cm);
    if (!fresh && !locked) {
        display.drawPixel(p.x, p.y, PIXEL_ON);
        return;
    }
    const int top = p.y - 3 < 8 ? 8 : p.y - 3;
    const int bottom = p.y + 4 > 56 ? 56 : p.y + 4;
    display.fillRect(p.x - 3, top, 7, bottom - top, PIXEL_OFF);
    if (locked) {
        display.drawRect(p.x - 2, p.y - 2, 5, 5, PIXEL_ON);
        display.drawPixel(p.x, p.y, PIXEL_ON);
    } else {
        display.fillRect(p.x - 1, p.y - 1, 3, 3, PIXEL_ON);
    }
}

template <class Display>
void draw(Display &display, const Radar::Controller &controller, const History &history,
          int servoAngle, uint32_t now) {
    display.clearDisplay();
    display.setTextColor(PIXEL_ON);
    display.setTextSize(1);
    display.setTextWrap(false);
    const bool search = controller.mode() == Radar::Mode::Search;
    const bool waiting = controller.state() == Radar::State::Waiting;
    const bool tracking = !search && controller.state() == Radar::State::Track &&
                          controller.radarPresence();
    const char *status = waiting ? "WAIT" : tracking ? "LOCK" :
                         (search && controller.targetFound()) ? "FOUND" : "SCAN";
    display.setCursor(0, 0);
    display.print(search ? "SEARCH" : "TRACK");
    display.setCursor(WIDTH - int(strlen(status)) * 6, 0);
    display.print(status);
    semiCircle(display, RADIUS / 3, true);
    semiCircle(display, RADIUS * 2 / 3, true);
    semiCircle(display, RADIUS, false);
    display.drawLine(CENTER_X - RADIUS, CENTER_Y, CENTER_X + RADIUS, CENTER_Y, PIXEL_ON);
    const Position sweep = position(servoAngle, Config::DETECT_CM);
    display.drawLine(CENTER_X, CENTER_Y, sweep.x, sweep.y, PIXEL_ON);
    char label[24];
    snprintf(label, sizeof(label), "%.0fcm", double(Config::DETECT_CM));
    display.setCursor(0, 12);
    display.print(label);
    if (search) {
        for (unsigned i = 0; i < HISTORY_CAPACITY; ++i) {
            const Echo &echo = history.echo(i);
            if (alive(echo, now, Config::SEARCH_ECHO_TTL_MS)) {
                target(display, echo, false,
                       uint32_t(now - echo.measuredAt) < Config::SEARCH_ECHO_FRESH_MS);
            }
        }
    } else if (tracking && alive(history.locked(), now, Config::TRACK_ECHO_TTL_MS)) {
        target(display, history.locked(), true, true);
    }
    snprintf(label, sizeof(label), "A:%03d", servoAngle);
    display.setCursor(0, 56);
    display.print(label);
    const float distance = controller.distance();
    if (!waiting && Radar::validDistance(distance)) snprintf(label, sizeof(label), "D:%.0fcm", double(distance));
    else
        snprintf(label, sizeof(label), "D:--cm");
    display.setCursor(WIDTH - int(strlen(label)) * 6, 56);
    display.print(label);
}
}
