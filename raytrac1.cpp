#include <math.h>
#include <string.h>
#include <stdio.h>
#include <malloc.h>

#include "types.h"
#include "vectorFp.h"
#include "raytrace.h"
#include "mathutil.h"


#define FAR_FI (1 << 30)

#define FP_SQRT(x) (isqrt(x) << (RP_BITS / 2))

typedef struct Object
{
	int type;
	Vec3fp pos, dir1, dir2;
	int radius;
	int a, b, c, d;
	int index;
} Object;

typedef struct Hit
{
	Object* obj;
	Vec3fp pos;
} Hit;

static Object plane;
static Object sphere;


static Vec3fp* viewDirPerPixel;
static int* viewDirDotPerPixel;

static void setupViewDirPerPixelLUTs(Screen* buffer)
{
	const int screenWidth = buffer->width;
	const int screenHeight = buffer->height;
	const float aspectRatio = (float)screenHeight / (float)screenWidth;

	const int Xinc = FLT_TO_FP(2.0f / (screenWidth - 1));
	const int Yinc = FLT_TO_FP((2.0f / (screenHeight - 1)) * aspectRatio);

	Vec3fp* dir = viewDirPerPixel;
	int* dirDot = viewDirDotPerPixel;

	int Ydir = (screenHeight / 2) * Yinc;
	for (int y = 0; y < screenHeight; y++) {
		int Xdir = (-screenWidth / 2) * Xinc;
		for (int x = 0; x < screenWidth; x++) {
			dir->x = Xdir;
			dir->y = Ydir;
			dir->z = INT_TO_FP(1);

			//Don't normalize yet, if we can get the vis without it..
			//Vnormalize(dir);

			*dirDot++ = 2 * (FP_MUL_T(dir->x, dir->x) + FP_MUL_T(dir->y, dir->y) + FP_MUL_T(dir->z, dir->z));
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

	Vec3fp* src = viewDirPerPixel;
	for (int y = 0; y < height; ++y) {
		for (int x = 0; x < width; ++x) {
			uint32 c = FP_TO_INT((FP_MUL_T(src->x, src->x) + FP_MUL_T(src->y, src->y)) * 256);
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

	int* src = viewDirDotPerPixel;
	for (int y = 0; y < height; ++y) {
		for (int x = 0; x < width; ++x) {
			uint32 c = (uint32)(FP_TO_FLT(*src) * 16);
			if (c > 255) c = 255;
			*dst++ = (uint8)c;
			++src;
		}
	}
}

static int Vdot(Vec3fp* v1, Vec3fp* v2)
{
    return FP_MUL_T(v1->x, v2->x) + FP_MUL_T(v1->y, v2->y) + FP_MUL_T(v1->z, v2->z);
}

static int Vlength(Vec3fp *v)
{
	return FP_SQRT(Vdot(v, v));
}

static void Vnormalize (Vec3fp* v)
{
	int r = Vlength(v);
	if (r!=0) {
		v->x = FP_DIV_T(v->x, r);
		v->y = FP_DIV_T(v->y, r);
		v->z = FP_DIV_T(v->z, r);
	}
}

static void Vcross(Vec3fp* v1, Vec3fp* v2, Vec3fp* v)
{
    v->x = FP_MUL_T(v1->y, v2->z) - FP_MUL_T(v2->y, v1->z);
    v->y = FP_MUL_T(v1->z, v2->x) - FP_MUL_T(v2->z, v1->x);
    v->z = FP_MUL_T(v1->x, v2->y) - FP_MUL_T(v2->x, v1->y);
}

static void calcABCD(Object* obj)
{
	Vec3fp v;
	switch (obj->type) {

		case OBJ_PLANE:
			Vcross(&obj->dir1, &obj->dir2, &v);
			Vnormalize(&v);
			obj->a = v.x;
			obj->b = v.y;
			obj->c = v.z;
			obj->d = -Vlength(&obj->pos);
			break;

		case OBJ_SPHERE:
			obj->a = obj->pos.x;
			obj->b = obj->pos.y;
			obj->c = obj->pos.z;
			obj->d = 2 * (FP_MUL_T(obj->pos.x, obj->pos.x) + FP_MUL_T(obj->pos.y, obj->pos.y) + FP_MUL_T(obj->pos.z, obj->pos.z) - FP_MUL_T(obj->radius, obj->radius));
			break;

		default:
			break;
	}
}

static inline int intersect(Object* obj, Vec3fp* dir, int dirDot)
{
	switch (obj->type) {

		case OBJ_PLANE:
		{
			const int denom = FP_MUL_T(obj->a, dir->x) + FP_MUL_T(obj->b, dir->y) + FP_MUL_T(obj->c, dir->z);

			if (denom != 0) {
				return -FP_DIV_T(obj->d, denom);
			}
			return -1;
		}
		break;

		case OBJ_SPHERE:
		{
			const int a = dirDot;
			if (a == 0) return -1;

			const int b = 2 * (FP_MUL_T(dir->x, -obj->pos.x) + FP_MUL_T(dir->y, -obj->pos.y) + FP_MUL_T(dir->z, -obj->pos.z));
			const int c = obj->d;

			int d = FP_MUL_T(b, b) - FP_MUL_T(a, c);
			if (d >= 0) {
				d = FP_SQRT(d);

				// No div by zero despite warning (a happens to be well above zero)
				int t1 = FP_DIV_T(-b + d, a);
				int t2 = FP_DIV_T(-b - d, a);

				if (t1 <= 0 && t2 <= 0) {
					return -1;
				}

				if (t1 < t2) {
					if (t1 > 0) {
						return t1;
					} else {
						return t2;
					}
				}
				else {
					if (t2 > 0) {
						return t2;
					} else {
						return t1;
					}
				}
			}
		}
		break;
	}
	return -1;
}

static void traceRay(Vec3fp *dir, int *dirDot, Hit *hit)
{
	int t0, t1, tMin = FAR_FI;

	hit->obj = NULL;
	t0 = intersect(&plane, dir, 0);
	t1 = intersect(&sphere, dir, *dirDot);
	if (t0 > 0) { tMin = t0; hit->obj = &plane; }
	if (t1 > 0 && t1 < tMin) { tMin = t1; hit->obj = &sphere; }

	if (tMin < FAR_FI) {
		hit->pos.x = FP_MUL_T(tMin, dir->x);
		hit->pos.y = FP_MUL_T(tMin, dir->y);
		hit->pos.z = FP_MUL_T(tMin, dir->z);
	}
}

static void setObjectPlane(Object* obj, Vec3fp pos, Vec3fp dir1, Vec3fp dir2)
{
	obj->pos = pos;
	obj->dir1 = dir1;
	obj->dir2 = dir2;
	obj->type = OBJ_PLANE;
	obj->index = 0;
}

static void setObjectSphere(Object* obj, Vec3fp pos, int radius, int index)
{
	obj->pos = pos;
	obj->radius = radius;
	obj->type = OBJ_SPHERE;
	obj->index = index;
}

static void printABCD(Object* obj)
{
	printf("R1d_%d) %d %d %d %d\n", obj->type, obj->a, obj->b, obj->c, obj->d);
	printf("R1f_%d) %f %f %f %f\n", obj->type, FP_TO_FLT(obj->a), FP_TO_FLT(obj->b), FP_TO_FLT(obj->c), FP_TO_FLT(obj->d));
}

static void setupObjects()
{
	setObjectPlane(&plane, VEC_FLT_TO_FP(0, 0, -1), VEC_FLT_TO_FP(1, 0, 0), VEC_FLT_TO_FP(0, 0, 1));
	setObjectSphere(&sphere, VEC_FLT_TO_FP(0, -1, 4), FLT_TO_FP(1), 2);

	calcABCD(&plane);
	calcABCD(&sphere);

	printABCD(&plane);
	printABCD(&sphere);
}

static void updateObjects(int ticks)
{
	float posX = (float)sin(ticks / 650.0f) * 0.25f;
	float posY = (float)sin(ticks / 480.0f) * 0.25f;
	float posZ = 1.0f + (float)sin(ticks / 880.0f) * 0.5f;
	float radius = 0.25f;

	setObjectSphere(&sphere, VEC_FLT_TO_FP(posX, posY, posZ), FLT_TO_FP(radius), 2);

	calcABCD(&plane);
	calcABCD(&sphere);
}

void raytraceInit1(Screen* buffer)
{
	viewDirPerPixel = new Vec3fp[buffer->width * buffer->height];
	viewDirDotPerPixel = new int[buffer->width * buffer->height];

	setupViewDirPerPixelLUTs(buffer);

	setupObjects();
}

static inline int computeColor(Vec3fp* dir, Hit* hit)
{
	Object* obj = hit->obj;
	Vec3fp normal;
	int ci = 0;

	if (obj) {
		switch (obj->type) {
		case OBJ_PLANE:
			normal.x = obj->a;
			normal.y = obj->b;
			normal.z = obj->c;

			ci = (int)(FP_TO_FLT(Vdot(dir, &normal)) * 256);
			CLAMP(ci, 0, 255);
			ci = ((((int)(FP_TO_FLT(hit->pos.x) * 16) & 15) ^ ((int)(FP_TO_FLT(hit->pos.z) * 16) & 15)) * ci) >> 6;
			break;

		case OBJ_SPHERE:
			normal.x = -(hit->pos.x - obj->a);
			normal.y = -(hit->pos.y - obj->b);
			normal.z = -(hit->pos.z - obj->c);

			Vnormalize(&normal);

			ci = (int)(FP_TO_FLT(Vdot(dir, &normal)) * 256);
			CLAMP(ci, 0, 255);
			break;
		}
	}
	return ci;
}

static inline int computeColorObjIndex(Hit* hit)
{
	Object* obj = hit->obj;
	int ci = 0;

	if (obj) {
		if (obj->type == OBJ_PLANE) {
			return 31;
		} else {
			return ((obj->index + 1) << 5);
		}
	}
	return ci;
}

static void renderObjects(Screen* buffer)
{
	int count = buffer->width * buffer->height;
	uint8* dst = (uint8*)buffer->data;

	Hit hit;
	Vec3fp* dir = viewDirPerPixel;
	int* dirDot = viewDirDotPerPixel;

	do {
		hit.obj = &sphere;
		traceRay(dir, dirDot, &hit);

		*dst++ = computeColorObjIndex(&hit);
		//*dst++ = computeColor(dir, &hit);

		dir++;
		dirDot++;
	} while (--count != 0);
}

void raytraceRun1(Screen* buffer, int ticks)
{
	updateObjects(ticks);

	//renderViewDir(buffer);
	//renderViewDirDot(buffer);

	renderObjects(buffer);
}
