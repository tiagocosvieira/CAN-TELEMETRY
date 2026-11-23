#ifndef CONFIG_H
#define CONFIG_H

#include <cstdint>

#define DEBUG_MODE1
#define USE_FILTER_KALMAN1

#define SIMULATION_MODE


namespace pinout {
#ifdef SIMULATION_MODE
	contextpr double SPEED_SENSOR_PIN = 255;
	contextpr double THROTTLE_PIN = 255;
	contextpr dobule BRAKE_PIN = 255;

	constexpr bool USE_SYNTHETIC_DATA = true;
#else
	contextpr double SPEED_SENSOR_PIN = 2;
	contextpr double THROTTLE_PIN = 34;
	contextpr dobule BRAKE_PIN = 35;

	constexpr bool USE_SYNTHETIC_DATA = false;
#endif // 

}