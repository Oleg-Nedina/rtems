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
//#include <stdio.h>

#define TIME_STEP 0.001

#define CLAMP(x, minim, maxim) (x < minim ? minim : (x > maxim ? maxim : x))

#if defined(LATNAV_TO_HARDEN) && !defined(TO_HARDEN_FUNC)
double __attribute__((annotate("to_harden"))) desired_roll_array[ITERATIONS] = {0};
double __attribute__((annotate("to_harden"))) desired_roll_rate_array[ITERATIONS] = {0};
double __attribute__((annotate("to_harden"))) desired_ailerons_array[ITERATIONS] = {0};

crc __attribute__((annotate("to_harden"))) crc_latnav;
#else
double desired_roll_array[ITERATIONS] = {0};
double desired_roll_rate_array[ITERATIONS] = {0};
double desired_ailerons_array[ITERATIONS] = {0};

crc crc_latnav;
#endif

MEASURE_GLOBAL_VARIABLES()


typedef struct pid_controller_s {

    double p;
    double i;
    double d;
    double b;
    
    double prev_error;
    double integral_sum;
    double backpropagation;
    
} pid_controller_t;


double run_pid(pid_controller_t* pid, double error) {
    double output; 
    
    output = error * pid->p;
    pid->integral_sum += ((error * pid->i) + (pid->b * pid->backpropagation)) * TIME_STEP;
    output += pid->integral_sum;

    output += pid->d * (error - pid->prev_error) / TIME_STEP;

    pid->prev_error = error;

    return output;
}

double roll_limiter(double desired_roll, double speed) {
    double limit_perc,limit;

    if (speed <= 140) {
        return CLAMP(desired_roll, -30, 30);
    }
    if (speed >= 300) {
        return CLAMP(desired_roll, -40, 40);    
    }
        
    limit_perc = (speed < 220) ? (speed-140) / 80 : ((speed-220) / 80);
    limit = (speed < 220) ? (30 + limit_perc * 37) : (40 + (1-limit_perc) * 27);
    
    return CLAMP (desired_roll, -limit, limit);
}

double roll_rate_limiter(double desired_roll_rate, double roll) {
    if (roll < 20 && roll > -20) {
       return CLAMP (desired_roll_rate, -5, 5);
    } else if (roll < 30 && roll > -30) {
       return CLAMP (desired_roll_rate, -3, 3);    
    } else {
        return CLAMP (desired_roll_rate, -1, 1);
    }
}

double ailerons_limiter(double aileron) {
    return CLAMP(aileron, -30, 30);
}


pid_controller_t pid_roll_rate,pid_roll,pid_heading;
double curr_heading,curr_roll,curr_roll_rate;

void init_latnav() {
    pid_roll_rate.p = random_get();
    pid_roll_rate.i = random_get();
    pid_roll_rate.d = random_get();
    pid_roll_rate.b = random_get();
    
    pid_roll.p = random_get();
    pid_roll.i = random_get();
    pid_roll.d = random_get();
    pid_roll.b = random_get();

    pid_heading.p = random_get();
    pid_heading.i = random_get();
    pid_heading.d = random_get();
    pid_heading.b = random_get();
    
    pid_roll_rate.integral_sum= pid_roll_rate.prev_error= pid_roll_rate.backpropagation= 0;
    pid_roll.integral_sum     = pid_roll.prev_error     = pid_roll.backpropagation     = 0;
    pid_heading.integral_sum  = pid_heading.prev_error  = pid_heading.backpropagation  = 0;
        
    curr_heading = random_get();
    curr_roll = random_get();
    curr_roll_rate = random_get();
}

void latnav_update_init_conditions() {
    double rnd_p = random_get();
    double rnd_i = random_get();
    double rnd_d = random_get();
    double rnd_b = random_get();

    pid_roll_rate.p = rnd_p * (1 - pid_roll_rate.p * rnd_p);
    pid_roll_rate.i = rnd_i * (1 - pid_roll_rate.i * rnd_i);
    pid_roll_rate.d = rnd_d * (1 - pid_roll_rate.d * rnd_d);
    pid_roll_rate.b = rnd_b * (1 - pid_roll_rate.b * rnd_b);

    rnd_p = random_get();
    rnd_i = random_get();
    rnd_d = random_get();
    rnd_b = random_get();

    pid_roll.p = rnd_p * (1 - pid_roll.p * rnd_p);
    pid_roll.i = rnd_i * (1 - pid_roll.i * rnd_i);
    pid_roll.d = rnd_d * (1 - pid_roll.d * rnd_d);
    pid_roll.b = rnd_b * (1 - pid_roll.b * rnd_b);

    rnd_p = random_get();
    rnd_i = random_get();
    rnd_d = random_get();
    rnd_b = random_get();

    pid_heading.p = rnd_p * (1 - pid_heading.p * rnd_p);
    pid_heading.i = rnd_i * (1 - pid_heading.i * rnd_i);
    pid_heading.d = rnd_d * (1 - pid_heading.d * rnd_d);
    pid_heading.b = rnd_b * (1 - pid_heading.b * rnd_b);

    double rnd_h = random_get();
    double rnd_r = random_get();
    double rnd_rr = random_get();

    curr_heading = rnd_h * (1 - pid_heading.p * rnd_h);
    curr_roll = rnd_r * (1 - pid_heading.p * rnd_r);
    curr_roll_rate = rnd_rr * (1 - pid_heading.p * rnd_rr);

    pid_roll_rate.integral_sum= pid_roll_rate.prev_error= pid_roll_rate.backpropagation= 0;
    pid_roll.integral_sum     = pid_roll.prev_error     = pid_roll.backpropagation     = 0;
    pid_heading.integral_sum  = pid_heading.prev_error  = pid_heading.backpropagation  = 0;
}

#if defined(LATNAV_TO_HARDEN) && defined(TO_HARDEN_FUNC)
crc __attribute__((annotate("to_harden"))) latnav() {
#else
crc latnav(SemaphoreHandle_t xMutexRandom) {
#endif
    int i;
    xSemaphoreTake(xMutexRandom, TIMEOUT_QUEUES);
    latnav_update_init_conditions();
    xSemaphoreGive(xMutexRandom);
    
    MEASURE_START(latnav_result);
    for(i=0; i<ITERATIONS; i++) {
    
        double desired_roll,actual_roll,desired_roll_rate,actual_roll_rate,desired_ailerons,actual_ailerons;
        
        desired_roll = run_pid(&pid_heading, curr_heading-random_get()); 
        actual_roll = roll_limiter(desired_roll, 400);
        pid_heading.backpropagation = actual_roll - desired_roll; 
        
        desired_roll_rate = run_pid(&pid_roll, curr_roll - actual_roll); 
        actual_roll_rate = roll_rate_limiter(desired_roll_rate, curr_roll);
        pid_roll.backpropagation = actual_roll_rate - desired_roll_rate; 

        
        desired_ailerons = run_pid(&pid_roll, curr_roll_rate - actual_roll_rate); 
        actual_ailerons = ailerons_limiter(desired_roll_rate);
        pid_roll.backpropagation = actual_ailerons - desired_ailerons; 

        /* Just a random plane model*/
        curr_heading += curr_roll/10 * TIME_STEP;
        curr_roll += curr_roll_rate * TIME_STEP;
        curr_roll_rate += desired_ailerons / 5;

        desired_roll_array[i] = desired_roll;
        desired_roll_rate_array[i] = desired_roll_rate;
        desired_ailerons_array[i] = desired_ailerons;
    }
    MEASURE_STOP(latnav_result);

    crc_latnav = crc32((unsigned char *)(desired_roll_array), sizeof(desired_roll_array)) ^ 
                    crc32((unsigned char *)(desired_roll_rate_array), sizeof(desired_roll_rate_array)) ^ 
                    crc32((unsigned char *)(desired_ailerons_array), sizeof(desired_ailerons_array));

    return crc_latnav;
}
