#include <stdint.h>

/// @brief environment setup
#define PRODUCTION 0
#define DEVELOPMENT 1
#define ENVIRONMENT DEVELOPMENT

typedef uint32_t GLOBAL_ID_T;

typedef uint16_t POTENTIOMETER_T;
typedef float TEMPERATURE_T;
typedef float HUMIDITY_T;
typedef bool BOOLEAN_T;