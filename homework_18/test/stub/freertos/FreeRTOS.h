#pragma once
#include <cstdint>
#include <cstddef>
typedef int BaseType_t;
typedef unsigned int UBaseType_t;
typedef uint32_t TickType_t;
#define pdTRUE 1
#define pdFALSE 0
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))
#define portMAX_DELAY ((TickType_t)0xFFFFFFFF)
#define configMAX_PRIORITIES 25
typedef struct { int owner; } portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED {0}
void portENTER_CRITICAL(portMUX_TYPE*);
void portEXIT_CRITICAL(portMUX_TYPE*);
void portENTER_CRITICAL_ISR(portMUX_TYPE*);
void portEXIT_CRITICAL_ISR(portMUX_TYPE*);
