#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <rtems.h>
#include <rtems/bspIo.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "user.h"
#include "crc.h"
#include "simple_random.h"
#include "freertos_compat.h"

#define lenStackInputGeneration 0x500
#define lenStackUARTOutput      0x800
#define lenStackTemp            0x300

#define lenStackAcas            0x2000
#define lenStackAnn             0x500
#define lenStackCanny           0x500
#define lenStackImgScaling      0x800
#define lenStackJpeg            0x1000
#define lenStackLatnav          0x800

#define DELAY_1_SECOND          1000UL
#define QUEUE_LENGTH            1
#define RX_STRING_SIZE          16
#define MAX_BENCH_ITERATIONS    10

uint32_t acas(SemaphoreHandle_t xMutexRandom);
uint32_t ann(SemaphoreHandle_t xMutexRandom);
uint32_t canny(SemaphoreHandle_t xMutexRandom);
uint32_t img_scaling(SemaphoreHandle_t xMutexRandom);
uint32_t jpeg_compress(SemaphoreHandle_t xMutexRandom);
uint32_t latnav(SemaphoreHandle_t xMutexRandom);

static void InputGeneration(void *pvParameters);
static void UARTOutput(void *pvParameters);

void init_acas(void);
void init_ann(void);
void init_canny(void);
void init_img_scaling(void);
void init_jpeg_compress(void);
void init_latnav(void);

static void AcasBench(void *pvParameters);
static void AnnBench(void *pvParameters);
static void CannyBench(void *pvParameters);
static void ImgScalingBench(void *pvParameters);
static void JpegBench(void *pvParameters);
static void LatnavBench(void *pvParameters);

typedef struct {
    uint32_t crc;
    float tDelta;
} benchmark_result_t;

static TaskHandle_t xInputGeneration_Task;
static TaskHandle_t xUARTOutput_Task;

static TaskHandle_t xAcasTask;
static TaskHandle_t xAnnTask;
static TaskHandle_t xCannyTask;
static TaskHandle_t xImgScalingTask;
static TaskHandle_t xJpegTask;
static TaskHandle_t xLatnavTask;

static uint8_t xRxQueueBuffer[RX_STRING_SIZE * sizeof(char)];
static uint8_t xSeedAcasQueueBuffer[QUEUE_LENGTH * sizeof(uint32_t)];
static uint8_t xSeedAnnQueueBuffer[QUEUE_LENGTH * sizeof(uint32_t)];
static uint8_t xSeedCannyQueueBuffer[QUEUE_LENGTH * sizeof(uint32_t)];
static uint8_t xSeedImgScalingQueueBuffer[QUEUE_LENGTH * sizeof(uint32_t)];
static uint8_t xSeedJpegQueueBuffer[QUEUE_LENGTH * sizeof(uint32_t)];
static uint8_t xSeedLatnavQueueBuffer[QUEUE_LENGTH * sizeof(uint32_t)];
static uint8_t xCRCAcasQueueBuffer[QUEUE_LENGTH * sizeof(benchmark_result_t)];
static uint8_t xCRCAnnQueueBuffer[QUEUE_LENGTH * sizeof(benchmark_result_t)];
static uint8_t xCRCCannyQueueBuffer[QUEUE_LENGTH * sizeof(benchmark_result_t)];
static uint8_t xCRCImgScalingQueueBuffer[QUEUE_LENGTH * sizeof(benchmark_result_t)];
static uint8_t xCRCJpegQueueBuffer[QUEUE_LENGTH * sizeof(benchmark_result_t)];
static uint8_t xCRCLatnavQueueBuffer[QUEUE_LENGTH * sizeof(benchmark_result_t)];

static StaticQueue_t qRxQueue;
static StaticQueue_t qSeedAcasQueue;
static StaticQueue_t qSeedAnnQueue;
static StaticQueue_t qSeedCannyQueue;
static StaticQueue_t qSeedImgScalingQueue;
static StaticQueue_t qSeedJpegQueue;
static StaticQueue_t qSeedLatnavQueue;
static StaticQueue_t qCRCAcasQueue;
static StaticQueue_t qCRCAnnQueue;
static StaticQueue_t qCRCCannyQueue;
static StaticQueue_t qCRCImgScalingQueue;
static StaticQueue_t qCRCJpegQueue;
static StaticQueue_t qCRCLatnavQueue;

static QueueHandle_t xRxQueue;
static QueueHandle_t xSeedAcasQueue;
static QueueHandle_t xSeedAnnQueue;
static QueueHandle_t xSeedCannyQueue;
static QueueHandle_t xSeedImgScalingQueue;
static QueueHandle_t xSeedJpegQueue;
static QueueHandle_t xSeedLatnavQueue;
static QueueHandle_t xCRCAcasQueue;
static QueueHandle_t xCRCAnnQueue;
static QueueHandle_t xCRCCannyQueue;
static QueueHandle_t xCRCImgScalingQueue;
static QueueHandle_t xCRCJpegQueue;
static QueueHandle_t xCRCLatnavQueue;

static StaticSemaphore_t xIterationDoneBuffer;
static SemaphoreHandle_t xIterationDone;

static StaticSemaphore_t xMutexRandomBuffer;
static SemaphoreHandle_t xMutexRandom;

float T = 0.0f;
float minT = 0.0f;
float maxT = 0.0f;
static float frequency = 16.0f;

#if (configSUPPORT_STATIC_ALLOCATION == 1)
StackType_t xInputGenerationStack[lenStackInputGeneration];
StackType_t xUARTOutputStack[lenStackUARTOutput];
StackType_t xTempStack[lenStackTemp];

StackType_t xAcasStack[lenStackAcas];
StackType_t xAnnStack[lenStackAnn];
StackType_t xCannyStack[lenStackCanny];
StackType_t xImgScalingStack[lenStackImgScaling];
StackType_t xJpegStack[lenStackJpeg];
StackType_t xLatnavStack[lenStackLatnav];

StaticTask_t xInputGenerationBuffer, xUARTOutputBuffer, xTempBuffer;
StaticTask_t xAcasBuffer, xAnnBuffer, xCannyBuffer, xImgScalingBuffer, xJpegBuffer, xLatnavBuffer;
#endif

XTime tStartAcas;
XTime tEndAcas;
XTime tStartAnn;
XTime tEndAnn;
XTime tStartCanny;
XTime tEndCanny;
XTime tStartImgScaling;
XTime tEndImgScaling;
XTime tStartJpeg;
XTime tEndJpeg;
XTime tStartLatnav;
XTime tEndLatnav;

int main(void)
{
    xil_printf("Using static memory for tasks, queue and timer creations. \r\n");

    xIterationDone = xSemaphoreCreateBinaryStatic(&xIterationDoneBuffer);
    xMutexRandom = xSemaphoreCreateBinaryStatic(&xMutexRandomBuffer);
    xSemaphoreGive(xMutexRandom);

    xil_printf("Creating the minimal queues statically. \r\n");

    xRxQueue = xQueueCreateStatic(RX_STRING_SIZE, sizeof(char), xRxQueueBuffer, &qRxQueue);

    xSeedAcasQueue = xQueueCreateStatic(QUEUE_LENGTH, sizeof(uint32_t), xSeedAcasQueueBuffer, &qSeedAcasQueue);
    xSeedAnnQueue = xQueueCreateStatic(QUEUE_LENGTH, sizeof(uint32_t), xSeedAnnQueueBuffer, &qSeedAnnQueue);
    xSeedCannyQueue = xQueueCreateStatic(QUEUE_LENGTH, sizeof(uint32_t), xSeedCannyQueueBuffer, &qSeedCannyQueue);
    xSeedImgScalingQueue = xQueueCreateStatic(QUEUE_LENGTH, sizeof(uint32_t), xSeedImgScalingQueueBuffer, &qSeedImgScalingQueue);
    xSeedJpegQueue = xQueueCreateStatic(QUEUE_LENGTH, sizeof(uint32_t), xSeedJpegQueueBuffer, &qSeedJpegQueue);
    xSeedLatnavQueue = xQueueCreateStatic(QUEUE_LENGTH, sizeof(uint32_t), xSeedLatnavQueueBuffer, &qSeedLatnavQueue);

    xCRCAcasQueue = xQueueCreateStatic(QUEUE_LENGTH, sizeof(benchmark_result_t), xCRCAcasQueueBuffer, &qCRCAcasQueue);
    xCRCAnnQueue = xQueueCreateStatic(QUEUE_LENGTH, sizeof(benchmark_result_t), xCRCAnnQueueBuffer, &qCRCAnnQueue);
    xCRCCannyQueue = xQueueCreateStatic(QUEUE_LENGTH, sizeof(benchmark_result_t), xCRCCannyQueueBuffer, &qCRCCannyQueue);
    xCRCImgScalingQueue = xQueueCreateStatic(QUEUE_LENGTH, sizeof(benchmark_result_t), xCRCImgScalingQueueBuffer, &qCRCImgScalingQueue);
    xCRCJpegQueue = xQueueCreateStatic(QUEUE_LENGTH, sizeof(benchmark_result_t), xCRCJpegQueueBuffer, &qCRCJpegQueue);
    xCRCLatnavQueue = xQueueCreateStatic(QUEUE_LENGTH, sizeof(benchmark_result_t), xCRCLatnavQueueBuffer, &qCRCLatnavQueue);

    xil_printf("Queues created. \r\n");

    xInputGeneration_Task = xTaskCreateStatic(
        InputGeneration,
        (const char *)"Input",
        lenStackInputGeneration,
        NULL,
        tskIDLE_PRIORITY + 1,
        NULL,
        NULL
    );
    xUARTOutput_Task = xTaskCreateStatic(
        UARTOutput,
        (const char *)"UARTOut",
        lenStackUARTOutput,
        NULL,
        tskIDLE_PRIORITY + 1,
        NULL,
        NULL
    );

    xAcasTask = xTaskCreateStatic(
        AcasBench,
        (const char *)"Acas",
        lenStackAcas,
        NULL,
        tskIDLE_PRIORITY + 1,
        NULL,
        NULL
    );
    xAnnTask = xTaskCreateStatic(
        AnnBench,
        (const char *)"Ann",
        lenStackAnn,
        NULL,
        tskIDLE_PRIORITY + 1,
        NULL,
        NULL
    );
    xCannyTask = xTaskCreateStatic(
        CannyBench,
        (const char *)"Canny",
        lenStackCanny,
        NULL,
        tskIDLE_PRIORITY + 1,
        NULL,
        NULL
    );
    xImgScalingTask = xTaskCreateStatic(
        ImgScalingBench,
        (const char *)"ImgScale",
        lenStackImgScaling,
        NULL,
        tskIDLE_PRIORITY + 1,
        NULL,
        NULL
    );
    xJpegTask = xTaskCreateStatic(
        JpegBench,
        (const char *)"Jpeg",
        lenStackJpeg,
        NULL,
        tskIDLE_PRIORITY + 1,
        NULL,
        NULL
    );
    xLatnavTask = xTaskCreateStatic(
        LatnavBench,
        (const char *)"Latnav",
        lenStackLatnav,
        NULL,
        tskIDLE_PRIORITY + 1,
        NULL,
        NULL
    );

    xil_printf("Tasks created. \r\n");
    xil_printf("Starting the scheduler.\n");

    vTaskStartScheduler();

    for (;;) {
    }
}

static void InputGeneration(void *pvParameters)
{
    (void)pvParameters;
    u32 seed = 42;

    for (uint32_t iter = 0; iter < MAX_BENCH_ITERATIONS; iter++) {
        xil_printf("\nRunning iteration %lu with fixed seed %lu\n", (unsigned long)(iter + 1), (unsigned long)seed);

        xSemaphoreTake(xMutexRandom, TIMEOUT_QUEUES);
        srand(seed);
        random_set_seed(seed);
        xSemaphoreGive(xMutexRandom);

        xQueueSend(xSeedAcasQueue, &seed, portMAX_DELAY);

        xSemaphoreTake(xIterationDone, TIMEOUT_QUEUES);
    }

    xil_printf("Completed %d iterations. Exiting.\n", MAX_BENCH_ITERATIONS);
    exit(0);
}

static void UARTOutput(void *pvParameters)
{
    (void)pvParameters;
    xil_printf("Starting UARTOutput task.\n");

    uint32_t counter = 0;

    benchmark_result_t ResultAcas;
    benchmark_result_t ResultAnn;
    benchmark_result_t ResultCanny;
    benchmark_result_t ResultImgScaling;
    benchmark_result_t ResultJpeg;
    benchmark_result_t ResultLatnav;

    while (1) {
        xQueueReceive(xCRCAcasQueue, &ResultAcas, TIMEOUT_QUEUES);
        xQueueReceive(xCRCAnnQueue, &ResultAnn, TIMEOUT_QUEUES);
        xQueueReceive(xCRCCannyQueue, &ResultCanny, TIMEOUT_QUEUES);
        xQueueReceive(xCRCImgScalingQueue, &ResultImgScaling, TIMEOUT_QUEUES);
        xQueueReceive(xCRCJpegQueue, &ResultJpeg, TIMEOUT_QUEUES);
        xQueueReceive(xCRCLatnavQueue, &ResultLatnav, TIMEOUT_QUEUES);

        printf(
            "i:%lu\tcrcAcas:0x%08lx crcAnn:0x%08lx crcCanny:0x%08lx crcImgScaling:0x%08lx crcJpeg:0x%08lx crcLatnav:0x%08lx\n",
            (unsigned long)(++counter),
            (unsigned long)ResultAcas.crc,
            (unsigned long)ResultAnn.crc,
            (unsigned long)ResultCanny.crc,
            (unsigned long)ResultImgScaling.crc,
            (unsigned long)ResultJpeg.crc,
            (unsigned long)ResultLatnav.crc
        );

        xSemaphoreGive(xIterationDone);
    }
}

static void AcasBench(void *pvParameters)
{
    (void)pvParameters;

    uint32_t seed;
    benchmark_result_t ResultAcas;
    while (1) {
        xQueueReceive(xSeedAcasQueue, &seed, TIMEOUT_QUEUES);
        XTime_GetTime(&tStartAcas);
        srand(seed);
        random_set_seed(seed);
        init_acas();
        ResultAcas.crc = acas(xMutexRandom);
        XTime_GetTime(&tEndAcas);
        ResultAcas.tDelta = (float)(tEndAcas - tStartAcas);
        xQueueSend(xCRCAcasQueue, &ResultAcas, TIMEOUT_QUEUES);

        xQueueSend(xSeedAnnQueue, &seed, portMAX_DELAY);
    }
}

static void AnnBench(void *pvParameters)
{
    (void)pvParameters;

    uint32_t seed;
    benchmark_result_t ResultAnn;
    while (1) {
        xQueueReceive(xSeedAnnQueue, &seed, TIMEOUT_QUEUES);
        XTime_GetTime(&tStartAnn);
        srand(seed);
        random_set_seed(seed);
        init_ann();
        ResultAnn.crc = ann(xMutexRandom);
        XTime_GetTime(&tEndAnn);
        ResultAnn.tDelta = (float)(tEndAnn - tStartAnn);
        xQueueSend(xCRCAnnQueue, &ResultAnn, TIMEOUT_QUEUES);

        xQueueSend(xSeedCannyQueue, &seed, portMAX_DELAY);
    }
}

static void CannyBench(void *pvParameters)
{
    (void)pvParameters;

    uint32_t seed;
    benchmark_result_t ResultCanny;
    while (1) {
        xQueueReceive(xSeedCannyQueue, &seed, TIMEOUT_QUEUES);
        XTime_GetTime(&tStartCanny);
        srand(seed);
        random_set_seed(seed);
        init_canny();
        ResultCanny.crc = canny(xMutexRandom);
        XTime_GetTime(&tEndCanny);
        ResultCanny.tDelta = (float)(tEndCanny - tStartCanny);
        xQueueSend(xCRCCannyQueue, &ResultCanny, TIMEOUT_QUEUES);

        xQueueSend(xSeedImgScalingQueue, &seed, portMAX_DELAY);
    }
}

static void ImgScalingBench(void *pvParameters)
{
    (void)pvParameters;

    uint32_t seed;
    benchmark_result_t ResultImgScaling;
    while (1) {
        xQueueReceive(xSeedImgScalingQueue, &seed, TIMEOUT_QUEUES);
        XTime_GetTime(&tStartImgScaling);
        srand(seed);
        random_set_seed(seed);
        init_img_scaling();
        ResultImgScaling.crc = img_scaling(xMutexRandom);
        XTime_GetTime(&tEndImgScaling);
        ResultImgScaling.tDelta = (float)(tEndImgScaling - tStartImgScaling);
        xQueueSend(xCRCImgScalingQueue, &ResultImgScaling, TIMEOUT_QUEUES);

        xQueueSend(xSeedJpegQueue, &seed, portMAX_DELAY);
    }
}

static void JpegBench(void *pvParameters)
{
    (void)pvParameters;

    uint32_t seed;
    benchmark_result_t ResultJpeg;
    while (1) {
        xQueueReceive(xSeedJpegQueue, &seed, TIMEOUT_QUEUES);
        XTime_GetTime(&tStartJpeg);
        srand(seed);
        random_set_seed(seed);
        init_jpeg_compress();
        ResultJpeg.crc = jpeg_compress(xMutexRandom);
        XTime_GetTime(&tEndJpeg);
        ResultJpeg.tDelta = (float)(tEndJpeg - tStartJpeg);
        xQueueSend(xCRCJpegQueue, &ResultJpeg, TIMEOUT_QUEUES);

        xQueueSend(xSeedLatnavQueue, &seed, portMAX_DELAY);
    }
}

static void LatnavBench(void *pvParameters)
{
    (void)pvParameters;

    uint32_t seed;
    benchmark_result_t ResultLatnav;
    while (1) {
        xQueueReceive(xSeedLatnavQueue, &seed, TIMEOUT_QUEUES);
        XTime_GetTime(&tStartLatnav);
        srand(seed);
        random_set_seed(seed);
        init_latnav();
        ResultLatnav.crc = latnav(xMutexRandom);
        XTime_GetTime(&tEndLatnav);
        ResultLatnav.tDelta = (float)(tEndLatnav - tStartLatnav);
        xQueueSend(xCRCLatnavQueue, &ResultLatnav, TIMEOUT_QUEUES);
    }
}

static rtems_task Init(rtems_task_argument argument)
{
    (void)argument;
    (void)main();
}

#define CONFIGURE_APPLICATION_NEEDS_CLOCK_DRIVER
#define CONFIGURE_APPLICATION_NEEDS_SIMPLE_CONSOLE_DRIVER
#define CONFIGURE_MAXIMUM_TASKS 10
#define CONFIGURE_MAXIMUM_SEMAPHORES 4
#define CONFIGURE_MAXIMUM_MESSAGE_QUEUES 14
#define CONFIGURE_EXTRA_TASK_STACKS (9 * 4 * 1024)
#define CONFIGURE_MESSAGE_BUFFER_MEMORY ( \
    CONFIGURE_MESSAGE_BUFFERS_FOR_QUEUE(RX_STRING_SIZE, sizeof(char)) + \
    6 * CONFIGURE_MESSAGE_BUFFERS_FOR_QUEUE(QUEUE_LENGTH, sizeof(uint32_t)) + \
    6 * CONFIGURE_MESSAGE_BUFFERS_FOR_QUEUE(QUEUE_LENGTH, sizeof(benchmark_result_t)) \
)
#define CONFIGURE_RTEMS_INIT_TASKS_TABLE
#define CONFIGURE_INIT_TASK_ATTRIBUTES RTEMS_FLOATING_POINT
#define CONFIGURE_INIT_TASK_STACK_SIZE (4 * 1024)
#define CONFIGURE_UNIFIED_WORK_AREAS
#define CONFIGURE_INIT
#include <rtems/confdefs.h>
