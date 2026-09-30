#ifndef RAYTRACE_H
#define RAYTRACE_H

#include "types.h"
#include "demo.h"
#define COL_BITS 8

#define COL_BITS_OBJ 6
#define COLS_OBJ_RANGE (1<<COL_BITS_OBJ)

#define SPEC_COLS (COLS_OBJ_RANGE >> 2)
#define DIFFUSE_COLS (COLS_OBJ_RANGE - SPEC_COLS)


enum { OBJ_NOTHING, OBJ_PLANE, OBJ_SPHERE, OBJ_TYPES_COUNT };

void raytraceInit0(Screen *buffer);
void raytraceRun0(Screen *buffer, int ticks);

void raytraceInit1();
void raytraceRun1(Screen *buffer, int ticks, int subdiv);

#endif
