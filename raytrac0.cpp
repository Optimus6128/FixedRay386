#include <math.h>
#include <string.h>
#include <stdio.h>
#include <malloc.h>

#include "types.h"
#include "raytrace.h"
#include "vector.h"
#include "mathutil.h"


#define FAR_F 1000000000.0f

typedef struct Object
{
	int type;
	float reflectivity;
	Vec3 pos, dir1, dir2;
	float radius;
	float a, b, c, d;
	int index;
} Object;

typedef struct Hit
{
	Object* obj;
	Vec3 pos, normal;
} Hit;

typedef struct PerPixelPrecs
{
	Vec3* viewDir;
	float* dirDot;
} PerPixelPrecs;

#define CYBER_TEX_W 64
#define CYBER_TEX_H 64

#define CLOUD_TEX_W 128
#define CLOUD_TEX_H 128

static uint8 floorTex[CYBER_TEX_W * CYBER_TEX_H];
static uint8 ceilingTex[CLOUD_TEX_W * CLOUD_TEX_H];

static Object plane;
static Object sphere[3];
static Vec3 lightDir;

static PerPixelPrecs perPixelPrecs;

static void setupViewDirPerPixelLUTs(Screen* buffer)
{
    const int screenWidth = buffer->width;
    const int screenHeight = buffer->height;
    const float aspectRatio = (float)screenHeight / (float)screenWidth;

    const float Xinc = 2.0f / (screenWidth - 1);
    const float Yinc = (2.0f / (screenHeight - 1)) * aspectRatio;
	const float distFromCamera = 1.0f;	// maybe this doesn't need to be high (Xinc,Yinc left right are also normalized)
										// For a moment I thought when I removed viewer pos at -Z something, that it would be some stupid pinhole distorted view from 0,0,0 stretch over the pixelres and maybe it was a mistake to simplify the old, but let's see
										// EDIT: we can control the FOV from here. change 1.0f to 0.5f see what happens or to 1.5f

	Vec3* dir = perPixelPrecs.viewDir;
	float* dirDot = perPixelPrecs.dirDot;

    float Ydir = (screenHeight / 2) * Yinc;
    for (int y = 0; y < screenHeight; y++) {
        float Xdir = (-screenWidth / 2) * Xinc;
        for (int x = 0; x < screenWidth; x++) {
			dir->x = Xdir;
			dir->y = Ydir;
			dir->z = distFromCamera;

			Vnormalize(dir);

			*dirDot++ = 2.0f * (dir->x * dir->x + dir->y * dir->y + dir->z * dir->z);
			dir++;
            Xdir += Xinc;
        }
        Ydir -= Yinc;
    }
}

static void renderViewDir(Screen* buffer)
{
	const int width = buffer->width;
	const int height = buffer->height;
	uint8* dst = (uint8*)buffer->data;

	Vec3* src = perPixelPrecs.viewDir;
	for (int y = 0; y < height; ++y) {
		for (int x = 0; x < width; ++x) {
			uint32 c = (uint32)((src->x * src->x + src->y * src->y) * 255.0f);
			if (c > 255) c = 255;
			*dst++ = (uint8)(c >> 1);
			++src;
		}
	}
}

static void renderViewDirDot(Screen* buffer)
{
	const int width = buffer->width;
	const int height = buffer->height;
	uint8* dst = (uint8*)buffer->data;

	float* src = perPixelPrecs.dirDot;
	for (int y = 0; y < height; ++y) {
		for (int x = 0; x < width; ++x) {
			uint32 c = (uint32)(*src * 16);
			if (c > 255) c = 255;
			*dst++ = (uint8)c;
			++src;
		}
	}
}

static void calcABCD(Object* obj)
{
	Vec3 v;
	switch (obj->type) {

		case OBJ_PLANE:
			Vcross(&obj->dir1, &obj->dir2, &v);
			Vnormalize(&v);
			obj->a = v.x;
			obj->b = v.y;
			obj->c = v.z;
			obj->d = (float)-sqrt((obj->pos.x * obj->pos.x) + (obj->pos.y * obj->pos.y) + (obj->pos.z * obj->pos.z));
			break;

		case OBJ_SPHERE:
			obj->a = obj->pos.x;
			obj->b = obj->pos.y;
			obj->c = obj->pos.z;
			obj->d = 2.0f * (obj->pos.x * obj->pos.x + obj->pos.y * obj->pos.y + obj->pos.z * obj->pos.z - obj->radius * obj->radius);
			break;

		default:
			break;
	}
}

static inline float intersectPlane2(Object *obj, Vec3 *pos, Vec3 *dir)
{
	const float denom = obj->a * dir->x + obj->b * dir->y + obj->c * dir->z;

	if (denom != 0) {
		return -(obj->a * pos->x + obj->b * pos->y + obj->c * pos->z + obj->d) / denom;
	}
	return -1;
}

static inline float intersectSphere2(Object *obj, Vec3 *pos, Vec3 *dir)
{
	const float a = dir->x * dir->x + dir->y * dir->y + dir->z * dir->z;
	const float b = 2 * (dir->x * (pos->x - obj->pos.x) + dir->y * (pos->y - obj->pos.y) + dir->z * (pos->z - obj->pos.z));
	const float c = (pos->x - obj->pos.x) * (pos->x - obj->pos.x) + (pos->y - obj->pos.y) * (pos->y - obj->pos.y) + (pos->z - obj->pos.z) * (pos->z - obj->pos.z) - obj->radius * obj->radius;

	float d = b * b - 4 * a * c;
	if (d >= 0)
	{
		d = sqrt(d);

		float t1 = (-b + d) / (2 * a);
		float t2 = (-b - d) / (2 * a);

		if (t1 <= 0 && t2 <= 0) {
			return -1;
		}

		if (t1 < t2) {
			if (t1 > 0) {
				return t1;
			} else {
				return t2;
			}
		} else {
			if (t2 > 0) {
				return t2;
			} else {
				return t1;
			}
		}
	}
	return -1;
}

static inline float intersectPlane(Object* obj, Vec3* dir)
{
	const float denom = obj->a * dir->x + obj->b * dir->y + obj->c * dir->z;
	if (denom != 0) {
		return -obj->d / denom;
	}
	return -1;
}

static inline float intersectSphere(Object* obj, Vec3* dir, float dirDot)
{
	if (dirDot == 0.0f) return -1;

	const float b = 2 * (dir->x * (-obj->pos.x) + dir->y * (-obj->pos.y) + dir->z * (-obj->pos.z));
	const float c = obj->d;

	float d = b * b - dirDot * c;
	if (d >= 0) {
		d = (float)sqrt(d);

		float t = (-b - d) / dirDot;
		if (t <= 0) {
			t = (-b + d) / dirDot;
		}
		return t;
	}
	return -1;
}

static void traceRayFromReflectPosition(Hit *hit, Vec3 *dir)
{
	float t, tMin = FAR_F;

	Object *prevHitObj = hit->obj;

	Vec3 reflectPos;
	reflectPos.x = hit->pos.x;
	reflectPos.y = hit->pos.y;
	reflectPos.z = hit->pos.z;

	hit->obj = NULL;
	if (prevHitObj != &plane) {
		t = intersectPlane2(&plane, &reflectPos, dir);
		if (t > 0.0f) { tMin = t; hit->obj = &plane; }
	}

	if (prevHitObj != &sphere[0]) {
		t = intersectSphere2(&sphere[0], &reflectPos, dir); if (t > 0.0f && t < tMin) { tMin = t; hit->obj = &sphere[0]; }
	}
	if (prevHitObj != &sphere[1]) {
		t = intersectSphere2(&sphere[1], &reflectPos, dir); if (t > 0.0f && t < tMin) { tMin = t; hit->obj = &sphere[1]; }
	}
	if (prevHitObj != &sphere[2]) {
		t = intersectSphere2(&sphere[2], &reflectPos, dir); if (t > 0.0f && t < tMin) { tMin = t; hit->obj = &sphere[2]; }
	}

	if (tMin < FAR_F) {
		hit->pos.x = reflectPos.x + tMin * dir->x;
		hit->pos.y = reflectPos.y + tMin * dir->y;
		hit->pos.z = reflectPos.z + tMin * dir->z;
	}
}

static void traceRayFromOriginView(Vec3 *dir, float dirDot, Hit *hit)
{
	float t, tMin = FAR_F;

	hit->obj = NULL;
	//if (dir->y > 0.0f) {	// cheat (this plane in this scene is under horizon always)
		t = intersectPlane(&plane, dir);
		if (t > 0.0f) { tMin = t; hit->obj = &plane; }
	//}

	t = intersectSphere(&sphere[0], dir, dirDot); if (t > 0.0f && t < tMin) { tMin = t; hit->obj = &sphere[0]; }
	t = intersectSphere(&sphere[1], dir, dirDot); if (t > 0.0f && t < tMin) { tMin = t; hit->obj = &sphere[1]; }
	t = intersectSphere(&sphere[2], dir, dirDot); if (t > 0.0f && t < tMin) { tMin = t; hit->obj = &sphere[2]; }

	if (tMin < FAR_F) {
		hit->pos.x = tMin * dir->x;
		hit->pos.y = tMin * dir->y;
		hit->pos.z = tMin * dir->z;
	}
}

static void setObjectPlane(Object* obj, Vec3 pos, Vec3 dir1, Vec3 dir2, int index, float reflectivity)
{
	obj->pos = pos;
	obj->dir1 = dir1;
	obj->dir2 = dir2;
	obj->type = OBJ_PLANE;
	obj->reflectivity = reflectivity;
	obj->index = index;
}

static void setObjectSphere(Object* obj, Vec3 pos, float radius, int index, float reflectivity)
{
	obj->pos = pos;
	obj->radius = radius;
	obj->type = OBJ_SPHERE;
	obj->reflectivity = reflectivity;
	obj->index = index;
}

static void setupObjects()
{
	setObjectPlane(&plane, Vec3(0, 0, -1), Vec3(1, 0, 0), Vec3(0, 0, 1), 2, 0.0f);
	setObjectSphere(&sphere[0], Vec3(0, -1, 4), 0.25f, 1, 0.0f);
	setObjectSphere(&sphere[1], Vec3(-1, -1, 4), 0.375f, 2, 1.0f);
	setObjectSphere(&sphere[2], Vec3(1, -1, 4), 0.325f, 3, 0.0f);

	calcABCD(&plane);
	for (int i = 0; i < 3; ++i) {
		calcABCD(&sphere[i]);
	}
}

static void updateObjects(int ticks)
{
	for (int i = 0; i < 3; ++i) {
		float t = (float)(((i + 1) << 10) + ticks);
		float posX = (float)sin(t / 650.0f) * 0.5f;
		float posY = (float)sin(t / 480.0f) * 0.375f;
		float posZ = 1.75f + (float)sin(t / 880.0f) * 0.325f;

		sphere[i].pos = Vec3(posX, posY, posZ);
		calcABCD(&sphere[i]);
	}
}



#define RANGE_SHR 12
#define RANGE 4096
#define RANGE_COL 5
#define RANGE_TO_COL (RANGE_SHR - RANGE_COL)

#define FIX_MUL(a,b) ((a * b) >> RANGE_SHR)
#define RANDV_NUM 256

static bool genRandValues = false;


enum {TRI_AREA_LR_TOP, TRI_AREA_LR_BOTTOM, TRI_AREA_RL_TOP, TRI_AREA_RL_BOTTOM};

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

typedef struct ImggenParams
{
	int width, height;
	int rangeMin, rangeMax;

	int hashX, hashY, hashZ;
	int shrStart;
	int iterations;
} ImggenParams;

static void genCloudTexture(int width, int height, int rangeMin, int rangeMax, ImggenParams *params, unsigned char *imgData)
{
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

static void genGridTexture(int width, int height, unsigned char *imgData)
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

static ImggenParams generateImageParamsCloud(int width, int height, int rangeMin, int rangeMax, int hashX, int hashY, int hashZ, int shrStart, int iterations)
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

static void raytraceInitTextures()
{
	for (int i = 0; i < RANDV_NUM; ++i) {
		float ang = (float)(getRand(0, 32767)) / 32.0f;
		randV[i] = ((int)(sin(ang) * 32768.0f) * ((RANGE << 0) - 1)) >> 16;
	}

	ImggenParams paramsClouds = generateImageParamsCloud(CLOUD_TEX_W, CLOUD_TEX_H, 0,63, 1,255,127,7,3);
	genCloudTexture(CLOUD_TEX_W, CLOUD_TEX_H, 0, 63, &paramsClouds, ceilingTex);

	genGridTexture(CYBER_TEX_W, CYBER_TEX_H, floorTex);
}

void raytraceInit0(Screen* buffer)
{
	perPixelPrecs.viewDir = new Vec3[buffer->width * buffer->height];
	perPixelPrecs.dirDot = new float[buffer->width * buffer->height];

	setupViewDirPerPixelLUTs(buffer);

	setupObjects();

	raytraceInitTextures();
}

static inline int computeColor(Vec3* dir, Hit* hit, int ticks)
{
	Object* obj = hit->obj;
	Vec3 *normal = &hit->normal;
	int ci = 0;

	//uint8 *texture = floorTex;
	uint8 *texture = ceilingTex;
	int planeTexW = CLOUD_TEX_W;
	int planeTexH = CLOUD_TEX_H;

	if (obj) {
		switch (obj->type) {
			case OBJ_PLANE:
			{
				float scaleW = 0.125f;
				float scaleH = 0.25f;
				float zOff = 0.5f;//(float)ticks * 0.0005f;
				normal->x = obj->a;
				normal->y = obj->b;
				normal->z = obj->c;

				ci = (int)(Vdot(dir, normal) * (1 << COL_BITS));
				CLAMP(ci, 0, COLS_RANGE);
				//ci = ((((int)((hit->pos.x + 0.5f) * 16) & 15) ^ ((int)((hit->pos.z + 0.5f + zOff) * 16) & 15)) * ci) >> COL_BITS_OBJ;
				int u = (int)((scaleW * hit->pos.x + 0.5f) * planeTexW) & (planeTexW - 1);
				int v = (int)((scaleH * hit->pos.z + 0.5f + zOff) * planeTexH) & (planeTexH - 1);
				ci = (texture[v * planeTexW + u] * ci) >> COL_BITS_OBJ;
			}
			break;

			case OBJ_SPHERE:
			{
				normal->x = -(hit->pos.x - obj->a);
				normal->y = -(hit->pos.y - obj->b);
				normal->z = -(hit->pos.z - obj->c);

				Vnormalize(normal);

				ci = (int)(Vdot(&lightDir, normal) * (1 << COL_BITS_OBJ));
				CLAMP(ci, 0, COLS_OBJ_RANGE);
				break;
			}
		}
	}
	return ci;
}

static inline int computeColorObjIndex(Hit* hit)
{
	Object* obj = hit->obj;
	if (obj) {
		return (((obj->index+1) << 6) - 1);
	}
	return 0;
}

static void renderObjects(Screen* buffer, int ticks)
{
	int width = buffer->width;
	int height = buffer->height;
	int count = buffer->width * buffer->height;
	uint8* dst = (uint8*)buffer->data;

	Hit hit;
	Vec3* dir = perPixelPrecs.viewDir;
	float* dirDot = perPixelPrecs.dirDot;

	lightDir = Vec3(-0.25f, -0.5f, 0.875f);
	Vnormalize(&lightDir);

	for (int y=0; y<height; ++y) {
		for (int x=0; x<width; ++x) {
			traceRayFromOriginView(dir, *dirDot, &hit);

			uint8 c = computeColor(dir, &hit, ticks);
			uint8 reflectAdd = 0;

			if (hit.obj && hit.obj->reflectivity > 0.0f) {
				Vec3 reflectDir;

				Vscale(&hit.normal, &reflectDir, 2.0f * Vdot(dir, &hit.normal));
				Vsub(dir, &reflectDir, &reflectDir);
				Vnormalize(&reflectDir);

				float reflectDirDot = 2.0f * (reflectDir.x * reflectDir.x + reflectDir.y * reflectDir.y + reflectDir.z * reflectDir.z);

				traceRayFromReflectPosition(&hit, &reflectDir);
				reflectAdd = 4;
				c = reflectAdd + computeColor(dir, &hit, ticks);
			}

			if (hit.obj) {
				CLAMP(c, 0, COLS_OBJ_RANGE);
				c += (hit.obj->index << COL_BITS_OBJ);
			} else {
				c = 208 - (y >> 3) + reflectAdd;
			}

			*dst++ = c;

			dir++;
			dirDot++;
		}
	}
}

void raytraceRun0(Screen* buffer, int ticks)
{
	updateObjects(ticks);

	//renderViewDir(buffer);
	//renderViewDirDot(buffer);

	renderObjects(buffer, ticks);
}
