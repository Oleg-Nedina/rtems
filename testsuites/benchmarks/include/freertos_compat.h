#ifndef FREERTOS_COMPAT_H
#define FREERTOS_COMPAT_H

#include <rtems.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>

typedef rtems_id SemaphoreHandle_t;
typedef rtems_id QueueHandle_t;
typedef rtems_id TaskHandle_t;
typedef rtems_interval TickType_t;
typedef int32_t BaseType_t;
typedef uint32_t UBaseType_t;

typedef uint32_t StackType_t;
typedef uint32_t StaticTask_t;
typedef uint32_t StaticQueue_t;
typedef uint32_t StaticSemaphore_t;

typedef uint8_t u8;
typedef uint32_t u32;
typedef uint64_t XTime;

#define pdFALSE ((BaseType_t)0)
#define pdTRUE ((BaseType_t)1)
#define portMAX_DELAY ((TickType_t)RTEMS_NO_TIMEOUT)
#define TIMEOUT_QUEUES portMAX_DELAY
#define tskIDLE_PRIORITY ((rtems_task_priority)10)
#define configSUPPORT_STATIC_ALLOCATION 0
#define COUNTS_PER_SECOND 1000000000.0f

#define xil_printf printf
#define pdMS_TO_TICKS(ms) RTEMS_MILLISECONDS_TO_TICKS(ms)
#define vTaskDelay(ticks) (void)rtems_task_wake_after(ticks)
#define uxTaskGetStackHighWaterMark2(task) (0U)

static inline void XTime_GetTime(XTime *t)
{
    *t = rtems_clock_get_uptime_nanoseconds();
}

static inline SemaphoreHandle_t xSemaphoreCreateBinaryStatic(StaticSemaphore_t *buf)
{
    (void)buf;
    static uint8_t sem_idx = 0;
    rtems_id id = 0;
    char c = (char)('0' + (sem_idx++ % 10));
    (void)rtems_semaphore_create(
        rtems_build_name('S', 'E', 'M', c),
        0,
        RTEMS_SIMPLE_BINARY_SEMAPHORE | RTEMS_FIFO,
        0,
        &id
    );
    return id;
}

static inline BaseType_t xSemaphoreTake(SemaphoreHandle_t sem, TickType_t timeout)
{
    rtems_status_code sc = rtems_semaphore_obtain(sem, RTEMS_WAIT, timeout);
    return (sc == RTEMS_SUCCESSFUL) ? pdTRUE : pdFALSE;
}

static inline BaseType_t xSemaphoreGive(SemaphoreHandle_t sem)
{
    rtems_status_code sc = rtems_semaphore_release(sem);
    return (sc == RTEMS_SUCCESSFUL) ? pdTRUE : pdFALSE;
}

static inline QueueHandle_t xQueueCreateStatic(
    UBaseType_t queue_length,
    UBaseType_t item_size,
    uint8_t *storage_buf,
    StaticQueue_t *queue_buf
)
{
    (void)storage_buf;
    (void)queue_buf;
    static uint8_t q_idx = 0;
    rtems_id id = 0;
    char c1 = (char)('0' + ((q_idx / 10) % 10));
    char c2 = (char)('0' + (q_idx % 10));
    q_idx++;
    (void)rtems_message_queue_create(
        rtems_build_name('Q', 'U', c1, c2),
        queue_length,
        item_size,
        RTEMS_DEFAULT_ATTRIBUTES,
        &id
    );
    return id;
}

static inline BaseType_t freertos_queue_send_impl(
    QueueHandle_t q,
    const void *item,
    size_t item_size
)
{
    rtems_status_code sc = rtems_message_queue_send(q, item, item_size);
    return (sc == RTEMS_SUCCESSFUL) ? pdTRUE : pdFALSE;
}

static inline BaseType_t freertos_queue_receive_impl(
    QueueHandle_t q,
    void *item,
    TickType_t timeout
)
{
    size_t actual_size = 0;
    rtems_status_code sc = rtems_message_queue_receive(
        q,
        item,
        &actual_size,
        RTEMS_WAIT,
        timeout
    );
    return (sc == RTEMS_SUCCESSFUL) ? pdTRUE : pdFALSE;
}

#define xQueueSend(q, item, timeout) \
    freertos_queue_send_impl((q), (item), sizeof(*(item)))

#define xQueueReceive(q, item, timeout) \
    freertos_queue_receive_impl((q), (item), (timeout))

static inline TaskHandle_t xTaskCreateStatic(
    void (*task_code)(void *),
    const char *name,
    uint32_t stack_depth,
    void *parameters,
    rtems_task_priority priority,
    StackType_t *stack_buf,
    StaticTask_t *task_buf
)
{
    (void)stack_depth;
    (void)stack_buf;
    (void)task_buf;
    rtems_id id = 0;
    char n0 = (name && name[0]) ? name[0] : 'T';
    char n1 = (name && name[1]) ? name[1] : 'S';
    char n2 = (name && name[2]) ? name[2] : 'K';
    char n3 = (name && name[3]) ? name[3] : '0';
    size_t stack_bytes = 4096U;
    rtems_status_code sc = rtems_task_create(
        rtems_build_name(n0, n1, n2, n3),
        priority,
        stack_bytes,
        RTEMS_PREEMPT | RTEMS_TIMESLICE,
        RTEMS_FLOATING_POINT,
        &id
    );
    if (sc == RTEMS_SUCCESSFUL) {
        (void)rtems_task_start(
            id,
            (rtems_task_entry)task_code,
            (rtems_task_argument)parameters
        );
    }
    return id;
}

#define vTaskStartScheduler() rtems_task_delete(RTEMS_SELF)

#endif
