#pragma once
#include "freertos/FreeRTOS.h"
typedef void* TaskHandle_t;
typedef void (*TaskFunction_t)(void*);
TaskHandle_t xTaskGetCurrentTaskHandle(void);
void vTaskDelay(TickType_t);
void vTaskNotifyGiveFromISR(TaskHandle_t, BaseType_t*);
uint32_t ulTaskNotifyTake(BaseType_t clear, TickType_t wait);
BaseType_t xTaskCreatePinnedToCore(TaskFunction_t, const char*, uint32_t, void*, UBaseType_t, TaskHandle_t*, BaseType_t);
