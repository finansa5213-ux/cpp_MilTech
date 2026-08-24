#pragma once
#include <cstdio>
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL -1
#define ESP_ERR_INVALID_ARG 0x102
const char* esp_err_to_name(esp_err_t e);
#define ESP_ERROR_CHECK(x) do { esp_err_t rc__ = (x); (void)rc__; } while (0)
