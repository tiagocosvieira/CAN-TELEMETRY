#pragma once

#include <Arduino.h>
#include "VehicleState.h"

/**
 * @brief Deterministic numerical calculus and longitudinal vehicle dynamics engine.
 * Executed strictly at 20 Hz (dt = 0.05s) on RP2040 Core 1.
 */
class MathEngine {
public:
    MathEngine() = default;

    /**
     * @brief Advances the kinematics and dynamics estimation by one time step (dt = 0.05s).
     * @param rawSpeedMs Unfiltered velocity in m/s from OBD-II.
     * @return Updated ComputedDynamics snapshot.
     */
    ComputedDynamics step(float rawSpeedMs);

    /**
     * @brief Resets all internal filter states and distance accumulators.
     */
    void reset();

private:
    bool isInitialized_ = false;
    float prevFilteredSpeedMs_ = 0.0f;
    float prevAccelerationMs2_ = 0.0f;
    float totalDistanceM_ = 0.0f;
    uint32_t sampleCount_ = 0;

    float computeTractivePowerCv(float speedMs, float accelMs2) const;
};