#ifndef FX_PLASM_H
#define FX_PLASM_H

#include "types.h"
#include "demo.h"
#include "video.h"

void fxRayInit(Screen *screen, bool useFloat);
void fxRayRun(Screen *screen, uint32 ticks, bool useFloat, int subdiv);

#endif
