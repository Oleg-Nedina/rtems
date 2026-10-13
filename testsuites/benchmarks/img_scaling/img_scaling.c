/******************************************************************************
*   Copyright 2021 Politecnico di Milano
*
*   Licensed under the Apache License, Version 2.0 (the "License");
*   you may not use this file except in compliance with the License.
*   You may obtain a copy of the License at
*
*      http://www.apache.org/licenses/LICENSE-2.0
*
*   Unless required by applicable law or agreed to in writing, software
*   distributed under the License is distributed on an "AS IS" BASIS,
*   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*   See the License for the specific language governing permissions and
*   limitations under the License.
*******************************************************************************/
#include "user.h"

#include "simple_random.h"


#define SCALING_FACTOR 2

#ifdef USER_IMAGE_SCALING
#include "data/scaling_image.h"
#endif

#define CLAMP(v, min, max) if (v < min) { v = min; } else if (v > max) { v = max; }

#if defined(IMG_SCALING_TO_HARDEN) && !defined(TO_HARDEN_FUNC)
static unsigned char __attribute__((annotate("to_harden"))) mat_out[SCALING_FACTOR*IMG_HEIGHT][SCALING_FACTOR*IMG_WIDTH];
crc __attribute__((annotate("to_harden"))) crc_img_scaling;
#else
static unsigned char mat_out[SCALING_FACTOR*IMG_HEIGHT][SCALING_FACTOR*IMG_WIDTH];
crc crc_img_scaling;
#endif

MEASURE_GLOBAL_VARIABLES()

#ifndef USER_IMAGE_SCALING
static unsigned char mat_in[IMG_HEIGHT][IMG_WIDTH];
#endif

static double hermit_poly(double p_0, double p_1, double p_2, double p_3, double x){
    double a,b,c,d;

    a = -p_0/2 + (3 *p_1)/2 - (3 *p_2)/2 + p_3/2;
    b = p_0 - (5 *p_1)/2 + 2*p_2 - p_3/2;
    c = - p_0/2 +p_2/2;
    d = p_1;

    return  a*x*x*x + b*x*x +c*x+d;
}
static unsigned int get_pixel(int x, int y){
    CLAMP(x,0,(IMG_WIDTH-1));
    CLAMP(y,0,(IMG_HEIGHT-1));
    return mat_in[x][y];
}
static double bicubic_interpolation(int x,int y, double dx, double dy){
    double col0,col1,col2,col3,res;
    col0=hermit_poly(get_pixel(x-1,y-1),get_pixel(x,y-1),get_pixel(x+1,y-1),get_pixel(x+2,y-1),dx);
    col1=hermit_poly(get_pixel(x-1,y),get_pixel(x,y),get_pixel(x+1,y),get_pixel(x+2,y),dx);
    col2=hermit_poly(get_pixel(x-1,y+1),get_pixel(x,y+1),get_pixel(x+1,y+1),get_pixel(x+2,y+1),dx);
    col3=hermit_poly(get_pixel(x-1,y+2),get_pixel(x,y+2),get_pixel(x+1,y+2),get_pixel(x+2,y+2),dx);

    res=hermit_poly(col0,col1,col2,col3,dy);
    CLAMP(res,0,255);

    return res;
}
/**
 * @brief Actual image scaling implementation
 * 
 */
static void img_scaling_routine(){
    int i,j;
    for(i=0;i<SCALING_FACTOR*IMG_HEIGHT;i++){
       for(j=0;j<SCALING_FACTOR*IMG_WIDTH;j++){
           int x,y;
           double dx,dy;

           x = i/SCALING_FACTOR;
           y = j/SCALING_FACTOR;
           dx=(double)i/SCALING_FACTOR-x;
           dy=(double)j/SCALING_FACTOR-y;

           mat_out[i][j]=bicubic_interpolation(x,y,dx,dy);
       }
   }
    
}

void init_img_scaling() {
    #ifndef USER_IMAGE_SCALING
    int i;
    int j;
    
    for (i = 0; i < IMG_HEIGHT; i++){
        for (j = 0; j < IMG_WIDTH; j++){
            mat_in[i][j]=random_get()*256;
            mat_out[i][j] = 0;
        }
    }
    #endif
}


void img_scaling_update_init_conditions() {
    for (int i = 0; i < IMG_HEIGHT; i++){
        for (int j = 0; j < IMG_WIDTH; j++){
            double rnd = random_get();
            mat_in[i][j] = (unsigned short)(rnd* (256 - mat_in[i][j] * rnd));
        }
    }
}

/**
 * @brief It performs image scaling on a random RGB image using bicubic interpolation. The execution time is measured through user defined MEASURE_START()/MEASURE_STOP() macros. 
 */
#if defined(IMG_SCALING_TO_HARDEN) && defined(TO_HARDEN_FUNC)
crc __attribute__((annotate("to_harden"))) img_scaling(){
#else
crc img_scaling(SemaphoreHandle_t xMutexRandom){
#endif
    xSemaphoreTake(xMutexRandom, TIMEOUT_QUEUES);
    img_scaling_update_init_conditions();
    xSemaphoreGive(xMutexRandom);

    MEASURE_START(img_scaling_result);
    
    img_scaling_routine();
    
    MEASURE_STOP(img_scaling_result);

    crc_img_scaling = crc32(mat_out, sizeof(mat_out));
    return crc_img_scaling;
}