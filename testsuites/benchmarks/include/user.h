#ifndef USER_H_
#define USER_H_

#define USE_ITERATIONS_LOOP 1

#include "dev_classes.h"
#include "crc.h"
#include "freertos_compat.h"

#define UINT32_T unsigned int

#define BILLION 1000000000L

#define MEASURE_GLOBAL_VARIABLES()
#define MEASURE_START(x)
#define MEASURE_STOP(x)
#define CHECK_RESULT(x) while(!(x)){};

#define ANN_LOOP_BOUND(iters)
#define ANN_VAR(min,max)
#define ANN_VAR_NOBOUNDS()
#define ANN_ARRAY(min,max)
#define ANN_ARRAY_NOBOUNDS()

#endif
