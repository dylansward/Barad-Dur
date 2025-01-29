/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       McGee                                                     */
/*    Created:      10/5/2023, 8:40:34 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"
#include "robot-config.h"
#include "competition/autonomous.h"
#include "competition/opcontrol.h"

using namespace vex;

competition comp;

/**
 * Entry point to the program. No code should be placed here;
 * instead use competition/opcontrol.cpp and
 * competition/autonomous.cpp
*/
int main() {
    comp.autonomous(autonomous);
    comp.drivercontrol(opcontrol);
    robot_init();
}