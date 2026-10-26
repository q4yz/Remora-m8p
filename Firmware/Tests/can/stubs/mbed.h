#pragma once
#include <stdio.h>
#include <stdint.h>
#include <string.h>
extern uint32_t sim_uid[3];
static inline uint32_t HAL_GetUIDw0(void) { return sim_uid[0]; }
static inline uint32_t HAL_GetUIDw1(void) { return sim_uid[1]; }
static inline uint32_t HAL_GetUIDw2(void) { return sim_uid[2]; }
