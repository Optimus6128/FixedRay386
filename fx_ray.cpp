#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "fx_ray.h"

#include "gfxtools.h"
#include "raytrace.h"


void fxRayInit(Screen *screen)
{
	makeAndSetPal(0, 255, 0, 0, 0, 63, 63, 63);

	raytraceInit0(screen);
	raytraceInit1(screen);
}

void fxRayRun(Screen *screen, uint32 ticks)
{
	raytraceRun0(screen, ticks);
	//raytraceRun1(screen, ticks);
}
