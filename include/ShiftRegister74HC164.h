#pragma once

#include <Arduino.h>

/**
 * @brief Driver for the SN74HC164N 8-bit serial-in, parallel-out shift register.
 * Controls an 8-LED bar graph for real-time power/shift visualization.
 */
class ShiftRegister74HC164 {
public:
    ShiftRegister74HC164(uint8_t dataPin, uint8_t clockPin);

    /**
     * @brief Configures GPIO pins and clears all LEDs.
     */
    void begin();

    /**
     * @brief Shifts out a raw 8-bit LED bitmask (MSB first).
     */
    void writeMask(uint8_t bitmask);

    /**
     * @brief Maps a value in [0.0, maxValue] proportionally across 0 to 8 LEDs.
     */
    void displayBarLevel(float currentValue, float maxValue);

    /**
     * @brief Runs a quick startup sweep animation across all 8 LEDs.
     */
    void runStartupSweep(uint16_t stepDelayMs = 40);

private:
    uint8_t dataPin_;
    uint8_t clockPin_;
};