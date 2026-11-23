#pragma once

#include <Arduino.h>
#include <pico/mutex.h>

/**
 * @brief Raw telemetry acquired asynchronously on Core 0.
 */
struct RawTelemetry {
    uint16_t rpm = 0;
    float rawSpeedMs = 0.0f;
    uint32_t lastUpdateMs = 0;
};

/**
 * @brief Computed vehicle dynamics metrics generated at 20 Hz on Core 1.
 */
struct ComputedDynamics {
    float filteredSpeedMs = 0.0f;
    float accelerationMs2 = 0.0f;
    float jerkMs3 = 0.0f;
    float distanceM = 0.0f;
    float powerCv = 0.0f;
    uint32_t sampleCount = 0;
};

/**
 * @brief Thread-safe shared state bridging Core 0 (I/O) and Core 1 (Math).
 */
class SharedVehicleState {
public:
    SharedVehicleState() {
        mutex_init(&mtx_);
    }

    void updateRawTelemetry(uint16_t rpm, float speedMs, uint32_t timestampMs) {
        mutex_enter_blocking(&mtx_);
        raw_.rpm = rpm;
        raw_.rawSpeedMs = speedMs;
        raw_.lastUpdateMs = timestampMs;
        mutex_exit(&mtx_);
    }

    RawTelemetry getRawTelemetry() {
        mutex_enter_blocking(&mtx_);
        RawTelemetry copy = raw_;
        mutex_exit(&mtx_);
        return copy;
    }

    void updateComputedDynamics(const ComputedDynamics& dynamics) {
        mutex_enter_blocking(&mtx_);
        computed_ = dynamics;
        mutex_exit(&mtx_);
    }

    ComputedDynamics getComputedDynamics() {
        mutex_enter_blocking(&mtx_);
        ComputedDynamics copy = computed_;
        mutex_exit(&mtx_);
        return copy;
    }

private:
    mutex_t mtx_;
    RawTelemetry raw_;
    ComputedDynamics computed_;
};