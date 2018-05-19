#ifndef vector_h
#define vector_h

#include <stdint.h>
#include <stdio.h>

// gyro angular velocity readings

struct vector {
	float x;
	float y;
	float z;
};

typedef struct vector vector_t;

// vector functions

void vector_cross(const vector_t *a, const vector_t *b, vector_t *out);
float vector_dot(const vector_t *a, const vector_t *b);
void vector_normalize(vector_t *a);

#endif



