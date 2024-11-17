#ifndef MATERIAL_H
#define MATERIAL_H

#include "Color.h"

struct Material {
	int type; // 0 for DIFF, 1 for SPEC, 2 for FRAC
	Color diffuseColor;
	Color specularColor;
	float reflectionFactor;
	float shineness;
};

#endif // !MATERIAL_H

