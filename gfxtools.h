#ifndef GFXTOOLS_H
#define GFXTOOLS_H

typedef struct ImggenParams
{
	int width, height;
	int rangeMin, rangeMax;

	int hashX, hashY, hashZ;
	int shrStart;
	int iterations;
} ImggenParams;

ImggenParams generateImageParamsCloud(int width, int height, int rangeMin, int rangeMax, int hashX, int hashY, int hashZ, int shrStart, int iterations);

void genGridTexture(int width, int height, unsigned char *imgData);
void genCloudTexture(int width, int height, int rangeMin, int rangeMax, ImggenParams *params, unsigned char *imgData);

void makePalTab(int numCols, int r0, int g0, int b0, int r1, int g1, int b1, unsigned char *myPalTab);
void makeAndSetPal(int colStart, int colEnd, int r0, int g0, int b0, int r1, int g1, int b1);

#endif
