#pragma once
#define ESP_IDF_VERSION_VAL(a,b,c) (((a)<<16)|((b)<<8)|(c))
#ifndef IDF_MAJOR
#define IDF_MAJOR 5
#define IDF_MINOR 4
#endif
#define ESP_IDF_VERSION ESP_IDF_VERSION_VAL(IDF_MAJOR, IDF_MINOR, 0)
