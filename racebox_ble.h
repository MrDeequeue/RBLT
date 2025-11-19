#pragma once
#include <Arduino.h>

// Snapshot of current RaceBox telemetry.
// You can extend this later if you need more fields.
struct RaceboxSnapshot {
    bool     connected;    // BLE link up
    bool     hasFix;       // GPS fix OK (per fixStatus/fixFlags)
    double   latDeg;       // degrees, WGS84 (TODO: fill in from payload)
    double   lonDeg;       // degrees, WGS84 (TODO: fill in from payload)
    float    speedKPH;     // ground speed in km/h
    float    gX;           // accel X, g
    float    gY;           // accel Y, g
    float    gZ;           // accel Z, g
    float    leanDeg;      // derived lean angle from gY/gZ
    float    yawDeg;       // raw yaw if you ever wire it up
    float    pitchDeg;     // raw pitch if you ever wire it up
    uint32_t lastUpdateMs; // millis() of last FF/01 update
};

// Initialise RaceBox BLE (creates BLE client/task on Core 0)
void racebox_init();

// Lightweight periodic hook you *can* call from loop().
// Right now it's a no-op, but kept for future extension.
void racebox_tick();

// Simple state helpers
bool racebox_is_connected();
bool racebox_has_valid_fix();

// Atomically copy latest RaceBox data into 'out'.
void racebox_get_snapshot(RaceboxSnapshot &out);
