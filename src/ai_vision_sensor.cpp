#include "ai_vision_sensor.h"
#include <cmath>

void sensorDistanceTest(int pixelX, int pixelY) {
    const int cenX = resX/2, cenY = resY/2;
    int x = pixelX - cenX, y = cenY - pixelY;
    double phi = hPtD*x, theta = vPtD*y;

    double c = cameraHeight * tan(deg2rad(90+theta));
    double distX = c*sin(deg2rad(phi)), distY = c*cos(deg2rad(phi));

    printf("c = %f\n", c);
    printf("(~~~, %f* theta, %f* phi)\n", theta, phi);
    printf("(%fin X, %fin Y)\n", distX, distY);
}

point_t cameraRelativeTest(int pixelX, int pixelY) {
    const int cenX = resX/2, cenY = resY/2;
    int x = pixelX - cenX, y = cenY - pixelY;
    double phi = hPtD*x, theta = vPtD*y;

    double c = cameraHeight * tan(deg2rad(90+theta));
    return point_t{c*sin(deg2rad(phi)), c*cos(deg2rad(phi))};
}

point_t fieldRelativeTest(int pixelX, int pixelY, OdometryBase* odom) {
    point_t point = cameraRelativeTest(pixelX, pixelY) + cameraOffset.get_point();


}