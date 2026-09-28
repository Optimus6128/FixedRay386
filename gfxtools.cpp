#include <math.h>

#include "gfxtools.h"
#include "mathutil.h"
#include "video.h"


#define RANGE_SHR 12
#define RANGE 4096
#define RANGE_COL 6
#define RANGE_TO_COL (RANGE_SHR - RANGE_COL)

#define FIX_MUL(a,b) ((a * b) >> RANGE_SHR)
#define RANDV_NUM 256

static bool genRandValues = false;

static int hx, hy, hz;
static int shift, iters;

static int randV[RANDV_NUM];

static int dotGradDist(int x, int y, int sx, int sy)
{
	const unsigned int randVindex = (((x >> RANGE_SHR) + hx) * ((y >> RANGE_SHR) + hy) * ((x >> RANGE_SHR) * (y >> RANGE_SHR) + hz)) & (RANDV_NUM - 1);
	const int vx = randV[randVindex];
	const int vy = randV[(randVindex + 1) & (RANDV_NUM - 1)];

	return (vx * (sx - x) + vy * (sy - y)) >> RANGE_SHR;
}

static int smooth(int t) {
	return FIX_MUL(FIX_MUL(FIX_MUL(t, t), t), (FIX_MUL(t, (FIX_MUL(t, (6 << RANGE_SHR)) - (15 << RANGE_SHR))) + (10 << RANGE_SHR)));         // 6t^5 - 15t^4 + 10t^3
}

static int lerp(int n0, int n1, int t)
{
	t = smooth(t);
	return n0 * (RANGE - t - 1) + n1 * t;
}

static int perlin(int x, int y)
{
	const int px = x & (RANGE - 1);
	const int py = y & (RANGE - 1);

	const int x0 = x - px;
	const int y0 = y - py;
	const int x1 = x0 + RANGE;
	const int y1 = y0 + RANGE;

	const int sx = x0 + px;
	const int sy = y0 + py;

	const int n00 = dotGradDist(x0, y0, sx, sy);
	const int n10 = dotGradDist(x1, y0, sx, sy);
	const int n01 = dotGradDist(x0, y1, sx, sy);
	const int n11 = dotGradDist(x1, y1, sx, sy);

	const int ix0 = lerp(n00, n10, px) >> RANGE_SHR;
	const int ix1 = lerp(n01, n11, px) >> RANGE_SHR;

	int c = lerp(ix0, ix1, py) >> RANGE_SHR;

	return c;
}

static int perlinOctave(int x, int y)
{
	int i;
	int c = 0;
	int d = 0;
	int s = shift;
	for (i = 0; i < iters; ++i) {
		c += perlin(x << s, y << s) >> d;
		++d;
		++s;
	}

	c += (RANGE >> 1);
	if (c < 0) c = 0;
	if (c > RANGE - 1) c = RANGE - 1;
	return c;
}

void genCloudTexture(int width, int height, int rangeMin, int rangeMax, ImggenParams *params, unsigned char *imgData)
{
	if (!genRandValues) {
		for (int i = 0; i < RANDV_NUM; ++i) {
			float ang = (float)(getRand(0, 32767)) / 32.0f;
			randV[i] = ((int)(sin(ang) * 32768.0f) * ((RANGE << 0) - 1)) >> 16;
		}
		genRandValues = true;
	}

	int x, y;

	hx = params->hashX;
	hy = params->hashY;
	hz = params->hashZ;

	shift = params->shrStart;
	iters = params->iterations;

	for (y = 0; y < height; ++y) {
		for (x = 0; x < width; ++x) {
			int c = perlinOctave(x, y) >> RANGE_TO_COL;
			c -= 8;
			CLAMP(c, rangeMin, rangeMax)
			*imgData++ = c;
		}
	}
}

void genGridTexture(int width, int height, unsigned char *imgData)
{
	int x, y;
	int xc, yc;
	int c;

	for (y=0; y<height; y++) {
		yc = y - (height >> 1);
		for (x=0; x<width; x++) {
			xc = x - (width >> 1);
			c = (xc * xc * xc * xc + yc * yc * yc * yc) >> 16;
			if (c > 31) c = 31;
			*imgData++ = c;
		}
	}
}

ImggenParams generateImageParamsCloud(int width, int height, int rangeMin, int rangeMax, int hashX, int hashY, int hashZ, int shrStart, int iterations)
{
	ImggenParams params;

	params.width = width;
	params.height = height;
	params.rangeMin = rangeMin;
	params.rangeMax = rangeMax;

	params.hashX = hashX;
	params.hashY = hashY;
	params.hashZ = hashZ;
	params.shrStart = shrStart;
	params.iterations = iterations;

	return params;
}


void makePalTab(int numCols, int r0, int g0, int b0, int r1, int g1, int b1, unsigned char *myPalTab)
{
    float dr = (float)(r1 - r0) / (float)(numCols - 1);
    float dg = (float)(g1 - g0) / (float)(numCols - 1);
    float db = (float)(b1 - b0) / (float)(numCols - 1);
    float cr = (float)r0;
    float cg = (float)g0;
    float cb = (float)b0;
    for (int i=0; i<numCols; i++)
    {
        *myPalTab++ = (unsigned char)cr;
        *myPalTab++ = (unsigned char)cg;
        *myPalTab++ = (unsigned char)cb;
        cr += dr;
        cg += dg;
        cb += db;
    }
}

void makeAndSetPal(int colStart, int colEnd, int r0, int g0, int b0, int r1, int g1, int b1)
{
    int numCols = colEnd - colStart + 1;
    unsigned char *tempPal = new unsigned char[3 * numCols];

    makePalTab(numCols, r0, g0, b0, r1, g1, b1, tempPal);
    setPalFromTab(colStart, tempPal, numCols);

    delete tempPal;
}
