#include <math.h>
#include <string.h>
#include <stdio.h>
#include <malloc.h>

#include "types.h"
#include "vectorFp.h"
#include "raytrace.h"
#include "mathutil.h"


#define FAR_FI (1 << 30)

typedef struct Object
{
	int type;
	Vec3fp pos, dir1, dir2;
	int radius;
	int invR;
	int a, b, c, d;
	int index;
} Object;

typedef struct Hit
{
	Object* obj;
	Vec3fp pos;
} Hit;

typedef struct PerPixelPrecs
{
	Vec3fp* viewDir;
	int* dirDot;
} PerPixelPrecs;


static Object plane;
static Object sphere[3];
static Vec3fp lightDir;

static PerPixelPrecs perPixelPrecs;



static void setupViewDirPerPixelLUTs(Screen* buffer)
{
	const int screenWidth = buffer->width;
	const int screenHeight = buffer->height;
	const float aspectRatio = (float)screenHeight / (float)screenWidth;

	const int Xinc = FLT_TO_FP(2.0f / (screenWidth - 1));
	const int Yinc = FLT_TO_FP((2.0f / (screenHeight - 1)) * aspectRatio);

	Vec3fp* dir = perPixelPrecs.viewDir;
	int* dirDot = perPixelPrecs.dirDot;

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

	Vec3fp* src = perPixelPrecs.viewDir;
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

	int* src = perPixelPrecs.dirDot;
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

static inline int intersectPlane(Object* obj, Vec3fp* dir)
{
	const int denom = FP_MUL_T(obj->a, dir->x) + FP_MUL_T(obj->b, dir->y) + FP_MUL_T(obj->c, dir->z);
	if (denom!=0) {
		return -FP_DIV_T(obj->d, denom);
	}
	return -1;
}

static inline int intersectSphere(Object* obj, Vec3fp* dir, int dirDot)
{
	if (dirDot == 0) return -1;

	const int b = 2 * (FP_MUL_T(dir->x, -obj->pos.x) + FP_MUL_T(dir->y, -obj->pos.y) + FP_MUL_T(dir->z, -obj->pos.z));
	const int c = obj->d;

	int d = FP_MUL_T(b, b) - FP_MUL_T(dirDot, c);
	if (d >= 0) {
		d = FP_SQRT(d);

		// No div by zero despite warning (a happens to be well above zero)
		int t1 = FP_DIV_T(-b + d, dirDot);
		int t2 = FP_DIV_T(-b - d, dirDot);

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
	return -1;
}

static void traceRay(Vec3fp *dir, int dirDot, Hit *hit)
{
	int t, tMin = FAR_FI;
	Object *obj;

	hit->obj = NULL;
	if (dir->y < 0) {	// cheat (this plane in this scene is under horizon always)
		t = intersectPlane(&plane, dir);
		if (t > 0) { tMin = t; hit->obj = &plane; }
	}

	obj = &sphere[0]; t = intersectSphere(obj, dir, dirDot); if (t > 0 && t < tMin) { tMin = t; hit->obj = obj; }
	obj = &sphere[1]; t = intersectSphere(obj, dir, dirDot); if (t > 0 && t < tMin) { tMin = t; hit->obj = obj; }
	obj = &sphere[2]; t = intersectSphere(obj, dir, dirDot); if (t > 0 && t < tMin) { tMin = t; hit->obj = obj; }

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
	obj->invR = FP_DIV_T(FLT_TO_FP(1.0f), obj->radius);
}

/*static void printABCD(Object* obj)
{
	printf("R1d_%d) %d %d %d %d\n", obj->type, obj->a, obj->b, obj->c, obj->d);
	printf("R1f_%d) %f %f %f %f\n", obj->type, FP_TO_FLT(obj->a), FP_TO_FLT(obj->b), FP_TO_FLT(obj->c), FP_TO_FLT(obj->d));
}*/

static void setupObjects()
{
	setObjectPlane(&plane, VEC_FLT_TO_FP(0, 0, -1), VEC_FLT_TO_FP(1, 0, 0), VEC_FLT_TO_FP(0, 0, 1));
	setObjectSphere(&sphere[0], VEC_FLT_TO_FP(0, -1, 4), FLT_TO_FP(0.25f), 1);
	setObjectSphere(&sphere[1], VEC_FLT_TO_FP(-1, -1, 4), FLT_TO_FP(0.25f), 2);
	setObjectSphere(&sphere[2], VEC_FLT_TO_FP(1, -1, 4), FLT_TO_FP(0.25f), 3);

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
		float posY = (float)sin(t / 480.0f) * 0.25f;
		float posZ = 1.75f + (float)sin(t / 880.0f) * 0.25f;

		sphere[i].pos = VEC_FLT_TO_FP(posX, posY, posZ);
		calcABCD(&sphere[i]);
	}
}

void raytraceInit1(Screen* buffer)
{
	perPixelPrecs.viewDir = new Vec3fp[buffer->width * buffer->height];
	perPixelPrecs.dirDot = new int[buffer->width * buffer->height];

	setupViewDirPerPixelLUTs(buffer);

	setupObjects();
}

static inline int computeColor(Vec3fp* dir, Hit* hit, int ticks)
{
	Object* obj = hit->obj;
	Vec3fp normal;
	int ci = 0;
	int u, v;

	if (obj) {
		switch (obj->type) {
			case OBJ_PLANE:
			{
				int zOff = ticks << 2;
				normal.x = obj->a;
				normal.y = obj->b;
				normal.z = obj->c;

				ci = Vdot(dir, &normal);
				CLAMP(ci, 0, ((1 << RP_BITS) - 1));
				u = (hit->pos.x >> COL_BITS_OBJ) & COLS_OBJ_RANGE;
				v = ((hit->pos.z + zOff) >> COL_BITS_OBJ) & COLS_OBJ_RANGE;
				ci = FP_MUL((u ^ v), ci);
			}
			break;

			case OBJ_SPHERE:
			{
				normal.x = -FP_MUL_T((hit->pos.x - obj->a), obj->invR);
				normal.y = -FP_MUL_T((hit->pos.y - obj->b), obj->invR);
				normal.z = -FP_MUL_T((hit->pos.z - obj->c), obj->invR);

				//Vnormalize(&normal);

				ci = Vdot(&lightDir, &normal) >> (RP_BITS - COL_BITS_OBJ);
				CLAMP(ci, 0, COLS_OBJ_RANGE);
			}
			break;
		}
		ci += (obj->index << COL_BITS_OBJ);
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
	int count = buffer->width * buffer->height;
	uint8* dst = (uint8*)buffer->data;

	Hit hit;
	Vec3fp* dir = perPixelPrecs.viewDir;
	int* dirDot = perPixelPrecs.dirDot;

	lightDir = VEC_FLT_TO_FP(-0.25f, -0.5f, 0.875f);
	Vnormalize(&lightDir);

	do {
		traceRay(dir, *dirDot, &hit);

		//*dst++ = computeColorObjIndex(&hit);
		*dst++ = computeColor(dir, &hit, ticks);

		dir++;
		dirDot++;
	} while (--count != 0);
}

void raytraceRun1(Screen* buffer, int ticks)
{
	updateObjects(ticks);

	//renderViewDir(buffer);
	//renderViewDirDot(buffer);

	renderObjects(buffer, ticks);
}
