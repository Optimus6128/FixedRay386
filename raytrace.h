#ifndef RAYTRACE_H
#define RAYTRACE_H

#include "types.h"
#include "demo.h"

#define CLAMP_LEFT(value, min) { if (value < min) value = min; }
#define CLAMP_RIGHT(value, max) { if (value > max) value = max; }
#define CLAMP(value, min, max) { CLAMP_LEFT(value,min) CLAMP_RIGHT(value, max) }

enum { OBJ_NOTHING, OBJ_PLANE, OBJ_SPHERE };

void raytraceInit0(Screen *buffer);
void raytraceRun0(Screen *buffer, int ticks);

void raytraceInit1(Screen* buffer);
void raytraceRun1(Screen *buffer, int ticks);

#endif
