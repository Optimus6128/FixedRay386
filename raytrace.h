#ifndef RAYTRACE_H
#define RAYTRACE_H

#include "types.h"
#include "demo.h"

enum { OBJ_NOTHING, OBJ_PLANE, OBJ_SPHERE };

void raytraceInit0(Screen *buffer);
void raytraceRun0(Screen *buffer, int ticks);

void raytraceInit1(Screen* buffer);
void raytraceRun1(Screen *buffer, int ticks);

#endif
