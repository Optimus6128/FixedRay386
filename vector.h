#ifndef VECTOR_H
#define VECTOR_H

typedef struct Vec3
{
    float x, y, z;

    Vec3(float x, float y, float z) : x(x), y(y), z(z) {}
    Vec3() { x=0; y=0; z=0; }
} Vec3;

float Vdist(Vec3 *v1, Vec3 *v2);
void Vadd(Vec3 *v1, Vec3 *v2, Vec3 *v);
void Vsub(Vec3 *v1, Vec3 *v2, Vec3 *v);
void Vscale(Vec3 *v1, Vec3 *v, float z);
void Vneg(Vec3 *v);
void Vset (Vec3 *v1, Vec3 *v);
void Vnormalize (Vec3 *v);
void Vcross(Vec3 *v1, Vec3 *v2, Vec3 *v);
float Vdot(Vec3 *v1, Vec3 *v2);

#endif
