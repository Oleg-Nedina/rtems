#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <rtems.h>
#include <tmacros.h>
#include <stdio.h>
#include "freertos_compat.h"
#include "user.h"
#include "crc.h"
#include "simple_random.h"

const char rtems_test_name[] = "BENCH IMG SCALING";

extern void init_img_scaling(void);
extern crc img_scaling(SemaphoreHandle_t xMutexRandom);

static rtems_task Init(rtems_task_argument argument)
{
    (void) argument;
    TEST_BEGIN();
    SemaphoreHandle_t mutex = 0;
    random_set_seed(42);
    init_img_scaling();
    crc result = img_scaling(mutex);
    printf("IMG_SCALING CRC: 0x%08lx\n", (unsigned long)result);
    TEST_END();
    rtems_test_exit(0);
}

#define CONFIGURE_APPLICATION_NEEDS_CLOCK_DRIVER
#define CONFIGURE_APPLICATION_NEEDS_SIMPLE_CONSOLE_DRIVER
#define CONFIGURE_MAXIMUM_TASKS 1
#define CONFIGURE_RTEMS_INIT_TASKS_TABLE
#define CONFIGURE_INIT_TASK_ATTRIBUTES RTEMS_FLOATING_POINT
#define CONFIGURE_INIT_TASK_STACK_SIZE (16 * 1024)
#define CONFIGURE_UNIFIED_WORK_AREAS
#define CONFIGURE_INITIAL_EXTENSIONS RTEMS_TEST_INITIAL_EXTENSION
#define CONFIGURE_INIT
#include <rtems/confdefs.h>
