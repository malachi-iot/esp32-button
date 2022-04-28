#pragma once

#include <stdint.h>

typedef struct {
  uint8_t pin;
  bool inverted;

  // TODO: Put inverted in here too, resulting in
  // overall structure size the same as before ISR mod
  struct {
    uint8_t up_isr_triggerred : 1;
    uint8_t down_isr_triggerred : 1;
  } flags;
  uint16_t history;
  uint32_t down_time;
  uint32_t next_long_time;

  uint32_t start_window_bouncey;
} debounce_t;