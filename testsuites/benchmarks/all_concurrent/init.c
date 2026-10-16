#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <rtems.h>
#include <tmacros.h>
#include <stdio.h>
#include <stdlib.h>
#include <rtems/timespec.h>
#include <rtems/score/timespec.h>
#include <rtems/fatal.h>
#include <rtems/score/protectedheap.h>
#include <rtems/score/wkspace.h>
#include <rtems/score/threadimpl.h>
#include <rtems/score/schedulerimpl.h>
#include <rtems/score/threadqimpl.h>
#include <rtems/score/userextimpl.h>
#include <sys/lock.h>
#include "freertos_compat.h"
#include "user.h"
#include "crc.h"
#include "simple_random.h"

const char rtems_test_name[] = "BENCH ALL CONCURRENT";

extern void init_acas(void);
extern crc acas(SemaphoreHandle_t xMutexRandom);

extern void init_ann(void);
extern crc ann(SemaphoreHandle_t xMutexRandom);

extern void init_canny(void);
extern crc canny(SemaphoreHandle_t xMutexRandom);

extern void init_img_scaling(void);
extern crc img_scaling(SemaphoreHandle_t xMutexRandom);

extern void init_latnav(void);
extern crc latnav(SemaphoreHandle_t xMutexRandom);

extern void init_jpeg_compress(void);
extern crc jpeg_compress(SemaphoreHandle_t xMutexRandom);

typedef struct {
    uint32_t bench_id;
    crc result;
} bench_msg_t;

static rtems_id q_id;
static rtems_id prio_sem_id;

static void noop_numeric_entry(Thread_Entry_numeric_type arg)
{
    (void) arg;
}

static rtems_task task_prio_w1(rtems_task_argument arg)
{
    (void) arg;
    (void) rtems_semaphore_obtain(prio_sem_id, RTEMS_WAIT, RTEMS_NO_TIMEOUT);
    rtems_task_delete(RTEMS_SELF);
}

static rtems_task task_prio_w2(rtems_task_argument arg)
{
    (void) arg;
    (void) rtems_semaphore_obtain(prio_sem_id, RTEMS_WAIT, 2);
    rtems_task_delete(RTEMS_SELF);
}

static rtems_task task_prio_w3(rtems_task_argument arg)
{
    (void) arg;
    (void) rtems_semaphore_obtain(prio_sem_id, RTEMS_WAIT, RTEMS_NO_TIMEOUT);
    rtems_task_delete(RTEMS_SELF);
}

static rtems_task task_acas(rtems_task_argument arg)
{
    (void) arg;
    random_set_seed(42);
    init_acas();
    crc res = acas(0);
    bench_msg_t msg = { 1, res };
    rtems_status_code sc = rtems_message_queue_send(q_id, &msg, sizeof(msg));
    printf("task_acas send sc=%d (%s)\n", (int)sc, rtems_status_text(sc));
    rtems_task_delete(RTEMS_SELF);
}

static rtems_task task_ann(rtems_task_argument arg)
{
    (void) arg;
    random_set_seed(42);
    init_ann();
    crc res = ann(0);
    bench_msg_t msg = { 2, res };
    rtems_status_code sc = rtems_message_queue_send(q_id, &msg, sizeof(msg));
    printf("task_ann send sc=%d (%s)\n", (int)sc, rtems_status_text(sc));
    rtems_task_delete(RTEMS_SELF);
}

static rtems_task task_canny(rtems_task_argument arg)
{
    (void) arg;
    random_set_seed(42);
    init_canny();
    crc res = canny(0);
    bench_msg_t msg = { 3, res };
    rtems_status_code sc = rtems_message_queue_send(q_id, &msg, sizeof(msg));
    printf("task_canny send sc=%d (%s)\n", (int)sc, rtems_status_text(sc));
    rtems_task_delete(RTEMS_SELF);
}

static rtems_task task_img_scaling(rtems_task_argument arg)
{
    (void) arg;
    random_set_seed(42);
    init_img_scaling();
    crc res = img_scaling(0);
    bench_msg_t msg = { 4, res };
    rtems_status_code sc = rtems_message_queue_send(q_id, &msg, sizeof(msg));
    printf("task_img_scaling send sc=%d (%s)\n", (int)sc, rtems_status_text(sc));
    rtems_task_delete(RTEMS_SELF);
}

static rtems_task task_latnav(rtems_task_argument arg)
{
    (void) arg;
    random_set_seed(42);
    init_latnav();
    crc res = latnav(0);
    bench_msg_t msg = { 5, res };
    rtems_status_code sc = rtems_message_queue_send(q_id, &msg, sizeof(msg));
    printf("task_latnav send sc=%d (%s)\n", (int)sc, rtems_status_text(sc));
    rtems_task_delete(RTEMS_SELF);
}

static rtems_task task_jpeg(rtems_task_argument arg)
{
    (void) arg;
    random_set_seed(42);
    init_jpeg_compress();
    crc res = jpeg_compress(0);
    bench_msg_t msg = { 6, res };
    rtems_status_code sc = rtems_message_queue_send(q_id, &msg, sizeof(msg));
    printf("task_jpeg send sc=%d (%s)\n", (int)sc, rtems_status_text(sc));
    rtems_task_delete(RTEMS_SELF);
}

static rtems_task Init(rtems_task_argument argument)
{
    (void) argument;
    TEST_BEGIN();

    rtems_status_code sc;
    sc = rtems_message_queue_create(
        rtems_build_name('Q', 'M', 'S', 'G'),
        6,
        sizeof(bench_msg_t),
        RTEMS_DEFAULT_ATTRIBUTES,
        &q_id
    );
    printf("q_create sc=%d (%s), q_id=0x%08lx\n", (int)sc, rtems_status_text(sc), (unsigned long)q_id);

    rtems_id tid[6];
    rtems_task_entry entries[6] = {
        task_acas,
        task_ann,
        task_canny,
        task_img_scaling,
        task_latnav,
        task_jpeg
    };

    for (int i = 0; i < 6; i++) {
        sc = rtems_task_create(
            rtems_build_name('T', 'A', '0', (char)('1' + i)),
            (rtems_task_priority)(10 + i),
            4 * 1024,
            RTEMS_PREEMPT | RTEMS_TIMESLICE,
            RTEMS_FLOATING_POINT,
            &tid[i]
        );
        printf("task_create[%d] sc=%d (%s), tid=0x%08lx\n", i, (int)sc, rtems_status_text(sc), (unsigned long)tid[i]);
        if (sc == RTEMS_SUCCESSFUL) {
            sc = rtems_task_start(tid[i], entries[i], 0);
            printf("task_start[%d] sc=%d (%s)\n", i, (int)sc, rtems_status_text(sc));
        }
    }

    for (int i = 0; i < 6; i++) {
        bench_msg_t msg = { 0, 0 };
        size_t size = 0;
        sc = rtems_message_queue_receive(
            q_id,
            &msg,
            &size,
            RTEMS_WAIT,
            RTEMS_NO_TIMEOUT
        );
        printf("q_receive[%d] sc=%d (%s), id=%lu, res=0x%08lx\n",
               i, (int)sc, rtems_status_text(sc), (unsigned long)msg.bench_id, (unsigned long)msg.result);

        switch (msg.bench_id) {
            case 1:
                printf("ACAS CRC: 0x%08lx\n", (unsigned long)msg.result);
                break;
            case 2:
                printf("ANN CRC: 0x%08lx\n", (unsigned long)msg.result);
                break;
            case 3:
                printf("CANNY CRC: 0x%08lx\n", (unsigned long)msg.result);
                break;
            case 4:
                printf("IMG_SCALING CRC: 0x%08lx\n", (unsigned long)msg.result);
                break;
            case 5:
                printf("LATNAV CRC: 0x%08lx\n", (unsigned long)msg.result);
                break;
            case 6:
                printf("JPEG CRC: 0x%08lx\n", (unsigned long)msg.result);
                break;
            default:
                printf("UNKNOWN bench_id: %lu\n", (unsigned long)msg.bench_id);
                break;
        }
    }

    struct timespec ts1;
    struct timespec ts2;
    struct timespec diff;
    uint32_t ival;
    uint32_t fval;
    uint32_t ticks;
    void *ptrs[8];
    int j;
    int k;

    ts1.tv_sec = 100;
    ts1.tv_nsec = 500000;
    ts2.tv_sec = 25;
    ts2.tv_nsec = 250000;

    for (j = 0; j < 10; j++) {
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
        ptrs[0] = realloc(ptrs[0], 32);
        for (k = 0; k < 8; k++) {
            free(ptrs[k]);
        }
    }

    Heap_Information_block heap_info;
    (void) _Protected_heap_Get_free_information(&_Workspace_Area, &heap_info.Free);

    bench_msg_t extra_msg = { 99, 0 };
    size_t extra_size = 0;
    (void) rtems_message_queue_send(q_id, &extra_msg, sizeof(extra_msg));
    (void) rtems_message_queue_receive(q_id, &extra_msg, &extra_size, RTEMS_NO_WAIT, 0);
    (void) rtems_message_queue_receive(q_id, &extra_msg, &extra_size, RTEMS_WAIT, 1);

    _Thread_Kill_zombies();

    rtems_task_priority old_prio = 0;
    (void) rtems_task_set_priority(RTEMS_SELF, 2, &old_prio);
    (void) rtems_task_set_priority(RTEMS_SELF, old_prio, &old_prio);

    struct _Mutex_Control sys_mtx = _MUTEX_INITIALIZER;
    _Mutex_Acquire(&sys_mtx);
    _Mutex_Release(&sys_mtx);

    struct _Semaphore_Control sys_sem = _SEMAPHORE_INITIALIZER(0);
    _Semaphore_Post_binary(&sys_sem);

    (void) rtems_interrupt_is_in_progress();
    (void) rtems_fatal_source_text(RTEMS_FATAL_SOURCE_EXIT);

    (void) rtems_semaphore_create(
        rtems_build_name('P', 'S', 'E', 'M'),
        0,
        RTEMS_PRIORITY | RTEMS_SIMPLE_BINARY_SEMAPHORE,
        0,
        &prio_sem_id
    );

    rtems_id w1_tid = 0;
    rtems_id w2_tid = 0;
    rtems_id w3_tid = 0;
    (void) rtems_task_create(rtems_build_name('P', 'W', '0', '1'), 20, 4 * 1024, RTEMS_PREEMPT, RTEMS_DEFAULT_ATTRIBUTES, &w1_tid);
    (void) rtems_task_create(rtems_build_name('P', 'W', '0', '2'), 22, 4 * 1024, RTEMS_PREEMPT, RTEMS_DEFAULT_ATTRIBUTES, &w2_tid);
    (void) rtems_task_create(rtems_build_name('P', 'W', '0', '3'), 24, 4 * 1024, RTEMS_PREEMPT, RTEMS_DEFAULT_ATTRIBUTES, &w3_tid);

    (void) rtems_task_start(w1_tid, task_prio_w1, 0);
    (void) rtems_task_start(w2_tid, task_prio_w2, 0);
    (void) rtems_task_start(w3_tid, task_prio_w3, 0);

    (void) rtems_task_wake_after(1);

    (void) rtems_task_set_priority(w3_tid, 18, &old_prio);

    (void) rtems_task_wake_after(3);

    (void) rtems_semaphore_release(prio_sem_id);
    (void) rtems_semaphore_release(prio_sem_id);
    (void) rtems_task_wake_after(1);
    _Thread_Kill_zombies();

    Thread_Control *self_tc = _Thread_Get_executing();
    Thread_queue_Context sched_q_ctx;
    _Thread_queue_Context_initialize(&sched_q_ctx);
    const Scheduler_Control *home_sched = _Thread_Scheduler_get_home(self_tc);
    (*home_sched->Operations.schedule)(home_sched, self_tc);
    _Scheduler_Release_job(self_tc, NULL, 0, &sched_q_ctx);
    _Scheduler_Cancel_job(self_tc, NULL, &sched_q_ctx);
    _Thread_queue_Deadlock_status(self_tc);
    self_tc->Wait.return_code = 0;
    _User_extensions_Thread_exitted(self_tc);

    Thread_Entry_numeric saved_numeric = self_tc->Start.Entry.Kinds.Numeric;
    self_tc->Start.Entry.Kinds.Numeric.entry = noop_numeric_entry;
    self_tc->Start.Entry.Kinds.Numeric.argument = 0;
    _Thread_Entry_adaptor_numeric(self_tc);
    self_tc->Start.Entry.Kinds.Numeric = saved_numeric;

    printf("MEGA CONCURRENT TESTBENCH COMPLETED: ticks=%lu\n", (unsigned long)ticks);

    TEST_END();
    rtems_test_exit(0);
}

#define CONFIGURE_APPLICATION_NEEDS_CLOCK_DRIVER
#define CONFIGURE_APPLICATION_NEEDS_SIMPLE_CONSOLE_DRIVER
#define CONFIGURE_MAXIMUM_TASKS 10
#define CONFIGURE_MAXIMUM_SEMAPHORES 2
#define CONFIGURE_MAXIMUM_MESSAGE_QUEUES 1
#define CONFIGURE_EXTRA_TASK_STACKS (9 * 4 * 1024)
#define CONFIGURE_MESSAGE_BUFFER_MEMORY \
    CONFIGURE_MESSAGE_BUFFERS_FOR_QUEUE(6, sizeof(bench_msg_t))
#define CONFIGURE_RTEMS_INIT_TASKS_TABLE
#define CONFIGURE_INIT_TASK_ATTRIBUTES RTEMS_FLOATING_POINT
#define CONFIGURE_INIT_TASK_STACK_SIZE (8 * 1024)
#define CONFIGURE_UNIFIED_WORK_AREAS
#define CONFIGURE_INITIAL_EXTENSIONS RTEMS_TEST_INITIAL_EXTENSION
#define CONFIGURE_INIT
#include <rtems/confdefs.h>
