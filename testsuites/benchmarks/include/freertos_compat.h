#ifndef FREERTOS_COMPAT_H
#define FREERTOS_COMPAT_H
#include <stdint.h>
typedef int SemaphoreHandle_t;
#define portMAX_DELAY ((uint32_t)0xFFFFFFFFUL)
#define TIMEOUT_QUEUES portMAX_DELAY
#define xSemaphoreTake(sem, timeout) (1)
#define xSemaphoreGive(sem) (1)
#endif
