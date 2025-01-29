#pragma once

#include "core.h"
#include "vex.h"

const int resX = 320, resY = 240, horizontalFOV = 74, verticalFOV = 59/*Official docs say this is 63, but I think that is a lie*/, diagonalFOV = 87;
const double hPtD = horizontalFOV/(double) resX, vPtD = verticalFOV/(double) resY;

const double cameraHeight = 11.5;

void sensorDistanceTest(int pixelX, int pixelY);