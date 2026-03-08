#pragma once

#include "structs.h"

Vec3 calculateGravity();
Vec3 calculateDrag(Vec3 velocity);
Vec3 calculateMagnus(Vec3 velocity, Vec3 spin);
Vec3 returnForceVector(Vec3 currentVelocity, Vec3 spinVector);