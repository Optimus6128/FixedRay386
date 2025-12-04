#ifndef MATHUTIL_H
#define MATHUTIL_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "types.h"

#define CLAMP(value,min,max) if (value<min) value=min; if (value>max) value=max;
#define CLAMP_LEFT(value,min) if (value<min) value=min;
#define CLAMP_RIGHT(value,max) if (value>max) value=max;

#define PI 3.14159265359f

int isqrt(int x);
int getRand(int from, int to);

#endif
