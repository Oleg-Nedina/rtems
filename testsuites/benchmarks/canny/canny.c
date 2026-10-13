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

#include <math.h>

#ifdef USER_IMAGE_CANNY
#include "data/canny_image.h"
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


#define SIGMA 1.0
#define KERNEL_SIZE 5
#define CLAMP(v, min, max) if (v < min) { v = min; } else if (v > max) { v = max; }

static unsigned char mat_out[IMG_HEIGHT][IMG_WIDTH];
crc crc_canny;

MEASURE_GLOBAL_VARIABLES()


#ifndef USER_IMAGE_CANNY
static unsigned char mat_in[IMG_HEIGHT][IMG_WIDTH];
#endif
static unsigned char mat_blured[IMG_HEIGHT][IMG_WIDTH];
static double grad_orientation[IMG_HEIGHT][IMG_WIDTH];
static unsigned char mat_edges[IMG_HEIGHT][IMG_WIDTH];

static unsigned char top_bottom[IMG_HEIGHT][IMG_WIDTH];
static unsigned char bottom_top[IMG_HEIGHT][IMG_WIDTH];
static unsigned char left_right[IMG_HEIGHT][IMG_WIDTH];
static unsigned char right_left[IMG_HEIGHT][IMG_WIDTH];

/* KERNEL_SIZExKERNEL_SIZE gaussian filter with origin in (1,1) */
static double kernel_gauss[KERNEL_SIZE][KERNEL_SIZE];

/* Sobel operators */
static double sobel_x[3][3]={{-1,0,1},
                          {-2,0,2},
                          {-1,0,1}};
static double sobel_y[3][3]={{-1,-2,-1},
                          {0,0,0},
                          {1,2,1}};

/**
 * @brief It generates a KERNEL_SIZE x KERNEL_SIZE gaussian kernel
 * 
 */
static void gaussian_kernel_init(){
    int i;
    double sum;
    
    sum=0;
    for (i = 0; i < KERNEL_SIZE; i++) {
        int j;
        for (j = 0; j < KERNEL_SIZE; j++) {
            double x,y;
            x = i - (KERNEL_SIZE - 1) / 2.0;
            y = j - (KERNEL_SIZE - 1) / 2.0;
            kernel_gauss[i][j] =  exp(((pow(x, 2) + pow(y, 2)) / ((2 * pow(SIGMA, 2)))) * (-1));
            sum += kernel_gauss[i][j];
        }
    }

    for (i = 0; i < KERNEL_SIZE; i++) {
        int j;
        for (j = 0; j < KERNEL_SIZE; j++) {
            kernel_gauss[i][j] /= sum;
        }
    }

}

/**
 * @brief Performs 2D convolution of 3x3 kernel on src image
 * @param src source image
 * @param p_x  center point x coordinate
 * @param p_y center point y coordinate
 * @param k 3x3 kernel
 * @return int 
 */
static int convolution2D_3x3(unsigned char src[IMG_HEIGHT][IMG_WIDTH], int p_x, int p_y,double k[3][3]){
    int k_r,offset_x,offset_y,i;
    double temp;
    k_r=1;
    /*kernel can be superimposed? if not we are on borders, then we keep the values unchanged*/
    if(p_x-k_r<0 || p_y-k_r<0 || p_x+k_r>=IMG_HEIGHT || p_y+k_r>=IMG_WIDTH){
        return src[p_x][p_y];
    }
    /*offset between kernel's indexes and array's ones */
    offset_x=p_x-k_r;
    offset_y=p_y-k_r;

    temp=0;
    for(i=p_x-k_r;i<=p_x+k_r;i++){
        int j;
        for(j=p_y-k_r;j<=p_y+k_r;j++){
            temp+=k[i-offset_x][j-offset_y] * src[i][j];
        }
    }
    return temp;
}
/**
 * @brief Performs 2D convolution of 3x3 kernel on src image
 * @param src source image
 * @param p_x  center point x coordinate
 * @param p_y center point y coordinate
 * @param k 5x5 kernel
 * @return int 
 */
static int convolution2D_5x5(unsigned char src[IMG_HEIGHT][IMG_WIDTH], int p_x, int p_y,double k[5][5]){
    int k_r,offset_x,offset_y,i;
    double temp;
    k_r=2;
    /*kernel can be superimposed? if not we are on borders, then we keep the values unchanged*/
    if(p_x-k_r<0 || p_y-k_r<0 || p_x+k_r>=IMG_HEIGHT || p_y+k_r>=IMG_WIDTH){
        return src[p_x][p_y];
    }
    /*offset between kernel's indexes and array's ones */
    offset_x=p_x-k_r;
    offset_y=p_y-k_r;

    temp=0;
    for(i=p_x-k_r;i<=p_x+k_r;i++){
        int j;
        for(j=p_y-k_r;j<=p_y+k_r;j++){
            temp+=k[i-offset_x][j-offset_y] * src[i][j];
        }
    }
    return temp;
}
/**
 * @brief It applies sobel operator on input image
 * 
 */
static void sobel(){
    int i;
    for(i=0;i<IMG_HEIGHT;i++){
       int j;
       for(j=0;j<IMG_WIDTH;j++){
            int mag_x,mag_y,mag;
            
            mag_x=convolution2D_3x3(mat_blured,i,j,sobel_x);
            mag_y=convolution2D_3x3(mat_blured,i,j,sobel_y);
            mag=sqrt(mag_x*mag_x+mag_y*mag_y);

            CLAMP(mag,0,255)

            mat_edges[i][j]=mag;
            grad_orientation[i][j]=atan2(mag_y,mag_x);
        }
    }
    
}
/**
 * @brief It performs gauss filtering on input image
 * 
 */
static void gauss_filter(){
   int i,j;

   for(i=0;i<IMG_HEIGHT;i++){
       for(j=0;j<IMG_WIDTH;j++){
           mat_blured[i][j]=convolution2D_5x5(mat_in,i,j,kernel_gauss);
       }
    }
    
}
/**
 * @brief It performs non-maximum suppression
 * 
 */
static void nms(){
    int i,j;
    for(i=1;i<IMG_HEIGHT-1;i++){
       for(j=1;j<IMG_WIDTH-1;j++){
           double direction,curr_p,next_p,prev_p;

           direction=grad_orientation[i][j];
           curr_p=mat_edges[i][j];
           
           if((0 <= direction && direction < M_PI/8) || ((15*M_PI)/8 <= direction && direction < 2*M_PI)){
               next_p=mat_edges[i][j+1];
               prev_p=mat_edges[i][j-1];
           }
           else if ((M_PI/8 <= direction && direction < (3*M_PI)/8) || ((9*M_PI)/8 <= direction && direction < (11*M_PI)/8)){
               next_p=mat_edges[i-1][j+1];
               prev_p=mat_edges[i+1][j-1];
           }
           else if (((3*M_PI)/8 <= direction && direction < (5*M_PI)/8) || ((11*M_PI)/8 <= direction && direction < (13*M_PI)/8)){
               next_p=mat_edges[i+1][j];
               prev_p=mat_edges[i-1][j];
           }
           else{
               next_p=mat_edges[i+1][j+1];
               prev_p=mat_edges[i-1][j-1];
           }
           
           if(curr_p>= prev_p && curr_p>=next_p){
               mat_out[i][j]=curr_p;
           }
           
       }
    }
}
/**
 * @brief It scans point's neighbours to check if the point at (x,y) is part of an edge 
 * 
 * @param x x coordinate
 * @param y y coordinate
 * @return unsigned char 
 */
static unsigned char check_neighbours(int x, int y){
    int i,j;

    for(i=x;x-1>0 && i<IMG_HEIGHT;i++){
       for(j=y;y-1>0 &&j<IMG_WIDTH;j++){
           if(mat_out[i][j]==255) return 255;
       }
    }
    return 0;
}
/**
 * @brief It performs hysteresis thresholding
 * 
 * @param low_thresh low threshold
 * @param high_thresh high threshold
 */
static void hyst_tresh(int low_thresh, int high_thresh){
    int i,j;

    for(i=0;i<IMG_HEIGHT;i++){
       for(j=0;j<IMG_WIDTH;j++){
           int elem;
           
           elem= mat_out[i][j];
           if(elem>=high_thresh) top_bottom[i][j]=255;
           else if (elem<=low_thresh) top_bottom[i][j]=0;
           else top_bottom[i][j]=check_neighbours(i,j);
        }
    }
    
    
    for(i=IMG_HEIGHT-1;i>=0;i--){
       for(j=IMG_WIDTH-1;j>=0;j--){
           int elem;
           
           elem= mat_out[i][j];
           if(elem>=high_thresh) bottom_top[i][j]=255;
           else if (elem<=low_thresh) bottom_top[i][j]=0;
           else bottom_top[i][j]=check_neighbours(i,j);
        }
    }
   
    
    for(i=0;i<IMG_HEIGHT;i++){
       for(j=IMG_WIDTH-1;j>=0;j--){
           int elem;
           
           elem= mat_out[i][j];
           if(elem>=high_thresh) right_left[i][j]=255;
           else if (elem<=low_thresh) right_left[i][j]=0;
           else right_left[i][j]=check_neighbours(i,j);
        }
    }
    
    for(i=IMG_HEIGHT-1;i>=0;i--){
       for(j=0;j<IMG_WIDTH;j++){
           int elem;
           
           elem= mat_out[i][j];
           if(elem>=high_thresh) left_right[i][j]=255;
           else if (elem<=low_thresh) left_right[i][j]=0;
           else left_right[i][j]=check_neighbours(i,j);
        }
    }

    for(i=0;i<IMG_HEIGHT;i++){
       for(j=0;j<IMG_WIDTH;j++){
            int sum;

            sum=top_bottom[i][j]+bottom_top[i][j]+left_right[i][j]+right_left[i][j];
            CLAMP(sum,0,255)
            mat_out[i][j]=sum;
    }}
}
/**
 * @brief Actual Canny algorithm implementation
 * 
 */
static void canny_routine(){
    /*kernel initialization*/
    gaussian_kernel_init();
    gauss_filter();
    sobel();
    nms();
    hyst_tresh(100,200);

}

void init_canny() {
#ifndef USER_IMAGE_CANNY
    for (int i = 0; i < IMG_HEIGHT; i++){
        for (int j = 0; j < IMG_WIDTH; j++){
            mat_in[i][j]=random_get()*256;
        }
    }
#endif
}

void canny_update_init_conditions() {
#ifndef USER_IMAGE_CANNY
    for (int i = 0; i < IMG_HEIGHT; i++){
        for (int j = 0; j < IMG_WIDTH; j++){
            double rnd = random_get();
            mat_in[i][j] = rnd* (256 - mat_in[i][j] * rnd);
            mat_out[i][j]=0;
        }
    }
#endif
}

/**
 * @brief It performs canny algorithm on a random grayscale image . The execution time is measured through user defined MEASURE_START()/MEASURE_STOP() macros. 
 */
crc canny(SemaphoreHandle_t xMutexRandom){
    xSemaphoreTake(xMutexRandom, TIMEOUT_QUEUES);
    canny_update_init_conditions();
    xSemaphoreGive(xMutexRandom);

    MEASURE_START(canny_result);
    
    canny_routine();
    
    MEASURE_STOP(canny_result);

    crc_canny = crc32(mat_out, IMG_HEIGHT*IMG_WIDTH*sizeof(mat_in[0][0]));
    return crc_canny;
}