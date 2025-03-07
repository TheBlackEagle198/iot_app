#include <stdint.h>

#define MESH_MAX_NODE_ID 255 ///< The maximum node ID

#define RADIO_MIN_CHANNEL 100 ///< The minimum radio channel
#define RADIO_MAX_CHANNEL 125 ///< The maximum radio channel

/// @brief environment setup
#define PRODUCTION 0
#define DEVELOPMENT 1
#define ENVIRONMENT DEVELOPMENT

typedef uint32_t GLOBAL_ID_T;

typedef uint16_t POTENTIOMETER_T;
typedef float TEMPERATURE_T;
typedef float HUMIDITY_T;
typedef bool BOOLEAN_T;

typedef int16_t NODE_ID_T;
typedef uint32_t DELAY_T;