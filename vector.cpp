#include <math.h>
#include "vector.h"

float Vdist(Vec3* v1, Vec3* v2)
{
    return (float)sqrt((v2->x - v1->x)*(v2->x - v1->x) + (v2->y - v1->y)*(v2->y - v1->y) + (v2->z - v1->z)*(v2->z - v1->z));
}

void Vadd(Vec3* v1, Vec3* v2, Vec3* v)
{
    v->x = v1->x + v2->x;
    v->y = v1->y + v2->y;
    v->z = v1->z + v2->z;
}

void Vsub(Vec3* v1, Vec3* v2, Vec3* v)
{
    v->x = v1->x - v2->x;
    v->y = v1->y - v2->y;
    v->z = v1->z - v2->z;
}

void Vscale(Vec3* v1, Vec3* v, float z)
{
    v->x = v1->x * z;
    v->y = v1->y * z;
    v->z = v1->z * z;
}

void Vneg(Vec3* v)
{
    v->x = - v->x;
    v->y = - v->y;
    v->z = - v->z;
}

void Vset(Vec3* v1, Vec3* v)
{
    v->x = v1->x;
    v->y = v1->y;
    v->z = v1->z;
}

void Vnormalize (Vec3* v)
{
	float r = (float)sqrt(v->x * v->x + v->y * v->y + v->z * v->z);
	if (r!=0)
	{
		v->x = v->x / r;
		v->y = v->y / r;
		v->z = v->z / r;
	}
}

void Vcross(Vec3* v1, Vec3* v2, Vec3* v)
{
    v->x = v1->y * v2->z - v2->y * v1->z;
    v->y = v1->z * v2->x - v2->z * v1->x;
    v->z = v1->x * v2->y - v2->x * v1->y;
}

float Vdot(Vec3* v1, Vec3* v2)
{
    return v1->x * v2->x + v1->y * v2->y + v1->z * v2->z;
}
