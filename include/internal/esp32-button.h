#pragma once

#include <stdint.h>

typedef struct {
  uint8_t pin;
  bool inverted;
  uint16_t history;
  uint32_t down_time;
  uint32_t next_long_time;
} debounce_t;

