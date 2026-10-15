#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <rtems.h>
#include <tmacros.h>
#include <stdio.h>
#include <stdlib.h>
#include <rtems/timespec.h>
#include <rtems/score/timespec.h>

const char rtems_test_name[] = "BENCH EDDI STRESS";

static rtems_task Init(rtems_task_argument argument)
{
  (void) argument;
  struct timespec ts1;
  struct timespec ts2;
  struct timespec diff;
  uint32_t ival;
  uint32_t fval;
  uint32_t ticks;
  void *ptrs[8];
  int i;
  int k;

  TEST_BEGIN();

  ts1.tv_sec = 100;
  ts1.tv_nsec = 500000;
  ts2.tv_sec = 25;
  ts2.tv_nsec = 250000;

  for (i = 0; i < 10; i++) {
    rtems_timespec_add_to(&ts1, &ts2);
    rtems_timespec_subtract(&ts1, &ts2, &diff);
    rtems_timespec_divide(&ts1, &ts2, &ival, &fval);
    rtems_timespec_divide_by_integer(&ts1, 2, &diff);
    (void) rtems_timespec_is_valid(&ts1);
    (void) rtems_timespec_less_than(&ts2, &ts1);
    ticks = rtems_timespec_to_ticks(&diff);
    rtems_timespec_from_ticks(ticks + 1, &ts2);
    (void) rtems_clock_get_ticks_since_boot();

    for (k = 0; k < 8; k++) {
      ptrs[k] = malloc(64 + k * 16);
    }
    for (k = 0; k < 8; k++) {
      free(ptrs[k]);
    }
  }

  printf("EDDI STRESS VALIDATION COMPLETE: ticks=%lu\n", (unsigned long)ticks);
  TEST_END();
  rtems_test_exit(0);
}

#define CONFIGURE_APPLICATION_NEEDS_CLOCK_DRIVER
#define CONFIGURE_APPLICATION_NEEDS_SIMPLE_CONSOLE_DRIVER
#define CONFIGURE_MAXIMUM_TASKS 1
#define CONFIGURE_RTEMS_INIT_TASKS_TABLE
#define CONFIGURE_INIT_TASK_ATTRIBUTES RTEMS_DEFAULT_ATTRIBUTES
#define CONFIGURE_INIT_TASK_STACK_SIZE (8 * 1024)
#define CONFIGURE_UNIFIED_WORK_AREAS
#define CONFIGURE_INITIAL_EXTENSIONS RTEMS_TEST_INITIAL_EXTENSION
#define CONFIGURE_INIT
#include <rtems/confdefs.h>
