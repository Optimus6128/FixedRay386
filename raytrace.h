#ifndef RAYTRACE_H
#define RAYTRACE_H

#include "types.h"
#include "demo.h"
#define COL_BITS 8
#define COL_BITS_OBJ 6

#define COLS_RANGE ((1<<COL_BITS)-1)
#define COLS_OBJ_RANGE ((1<<COL_BITS_OBJ)-1)

enum { OBJ_NOTHING, OBJ_PLANE, OBJ_SPHERE, OBJ_TYPES_COUNT };

void raytraceInit0(Screen *buffer);
void raytraceRun0(Screen *buffer, int ticks);

void raytraceInit1(Screen* buffer);
void raytraceRun1(Screen *buffer, int ticks);

#endif
