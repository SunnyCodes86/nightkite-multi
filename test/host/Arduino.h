#pragma once
#include <stdint.h>
extern uint32_t hostMillis;
inline uint32_t millis() { return hostMillis; }
