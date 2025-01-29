#pragma once

#include "core.h"
#include "vex.h"

const int resX = 320, resY = 240, horizontalFOV = 74, verticalFOV = 59/*Official docs say this is 63, but I think that is a lie*/, diagonalFOV = 87;
const double hPtD = horizontalFOV/(double) resX, vPtD = verticalFOV/(double) resY;

const double cameraHeight = 11.5; // Height of the camera
const pose_t cameraOffset{5, -3.5, PI}; // x is xloc of camera from odom, y is yloc of camera from odom, and rot is rotation of camera at that point.

void sensorDistanceTest(int pixelX, int pixelY);

point_t cameraRelativeTest(int pixelX, int pixelY);

point_t fieldRelativeTest(int pixelX, int pixelY, OdometryBase* odom);