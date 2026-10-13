#pragma once

#include "user.h"
#include "freertos_compat.h"

void random_set_seed(UINT32_T seed);

double random_get(void);

void random_get_array(double a[], int len);

void random_get_sarray(double a[], int len);

void random_get_iarray(UINT32_T a[], int len);

void random_get_barray(int a[], int len);
