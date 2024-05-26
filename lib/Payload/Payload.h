#pragma once

#include <stdint.h>
#include "GlobalConfig.h"

struct Payload {
    GLOBAL_ID_T global_id;
    uint32_t data;
};