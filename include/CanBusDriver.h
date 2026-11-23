#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>

/**
 * @brief Driver wrapping MCP2515 SPI operations and OBD-II PID decoding.
 */
class CanBusDriver {
public:
    explicit CanBusDriver(uint8_t csPin);

    /**
     * @brief Initializes SPI0 and configures the MCP2515 controller.
     * @param loopbackMode True for hardware-in-the-loop testing, false for live CAN bus.
     * @return True if configuration succeeded.
     */
    bool begin(bool loopbackMode);

    /**
     * @brief Sends an OBD-II Mode 01 PID request (0x0C for RPM, 0x0D for Speed).
     */
    bool requestObdPid(uint8_t pid);

    /**
     * @brief Injects a synthetic OBD-II response frame (used in Loopback mode).
     */
    bool injectSimulatedObdFrame(uint16_t rpm, uint8_t speedKmh);

    /**
     * @brief Polls incoming CAN frames and updates internal RPM and Speed states.
     * @return True if a valid OBD-II frame was decoded.
     */
    bool poll();

    uint16_t getRpm() const { return currentRpm_; }
    float getSpeedMs() const { return currentSpeedMs_; }

private:
    MCP2515 mcp2515_;
    uint16_t currentRpm_ = 0;
    float currentSpeedMs_ = 0.0f;

    static constexpr uint32_t OBD_REQUEST_ID = 0x7DF;
    static constexpr uint32_t OBD_RESPONSE_ID = 0x7E8;
    static constexpr uint8_t  PID_ENGINE_RPM = 0x0C;
    static constexpr uint8_t  PID_VEHICLE_SPD = 0x0D;
};