
#ifdef CLASS_A
#define CLASS_PRECISION 0.5
#define ARRAY_LENGTH 10
#define MATRIX_SIZE 10
#define BOGO_LENGTH 5
#define FFT_LENGTH 8
#define IMG_HEIGHT 8
#define IMG_WIDTH 8
#define NR_PLANES 2 /* acas */
#define PARALLEL_SORT_LENGTH 50
#if USE_ITERATIONS_LOOP
    #define ITERATIONS 10
#else
    #define ITERATIONS 1
#endif
#endif

#ifdef CLASS_B
#define CLASS_PRECISION 0.5
#define ARRAY_LENGTH 100
#define MATRIX_SIZE 100
#define BOGO_LENGTH 6
#define FFT_LENGTH 128
#define IMG_HEIGHT 16
#define IMG_WIDTH 16
#define NR_PLANES 4 /* acas */
#define PARALLEL_SORT_LENGTH 500
#if USE_ITERATIONS_LOOP
    #define ITERATIONS 100
#else
    #define ITERATIONS 1
#endif
#endif

#ifdef CLASS_C
#define CLASS_PRECISION 0.1
#define ARRAY_LENGTH 1000
#define MATRIX_SIZE 300
#define BOGO_LENGTH 8
#define FFT_LENGTH 512
#define IMG_HEIGHT 128
#define IMG_WIDTH 128
#define NR_PLANES 8 /* acas */
#define PARALLEL_SORT_LENGTH 5000
#if USE_ITERATIONS_LOOP
    #define ITERATIONS 1000
#else
    #define ITERATIONS 1
#endif
#endif

#ifdef CLASS_D
#define CLASS_PRECISION 0.05
#define ARRAY_LENGTH 5000
#define MATRIX_SIZE 400
#define BOGO_LENGTH 9
#define FFT_LENGTH 1024
#define IMG_HEIGHT 256
#define IMG_WIDTH 256
#define NR_PLANES 16 /* acas */
#define PARALLEL_SORT_LENGTH 25000
#if USE_ITERATIONS_LOOP
    #define ITERATIONS 10000
#else
    #define ITERATIONS 1
#endif
#endif

#ifdef CLASS_E
#define CLASS_PRECISION 0.01
#define ARRAY_LENGTH 10000
#define MATRIX_SIZE 500
#define BOGO_LENGTH 10
#define FFT_LENGTH 2048
#define IMG_HEIGHT 512
#define IMG_WIDTH 512
#define NR_PLANES 32 /* acas */
#define PARALLEL_SORT_LENGTH 50000
#if USE_ITERATIONS_LOOP
    #define ITERATIONS 50000
#else
    #define ITERATIONS 1
#endif
#endif

