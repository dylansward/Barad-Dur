#include "competition/opcontrol.h"
#include "competition/autonomous.h"
#include "robot-config.h"
#include "vex.h"
#include <iostream>

const vex::controller::button &intake_button = con.ButtonL1;
const vex::controller::button &outtake_button = con.ButtonL2;
const vex::controller::button &goal_grabber = con.ButtonB;
const vex::controller::button &conveyor_button = con.ButtonR1;
const vex::controller::button &rev_conveyor_button = con.ButtonR2;

/**
 * Main entrypoint for the driver control period
 */
void opcontrol() {

  // ================ INIT ================
  while (imu.isCalibrating()) {
    vexDelay(1);
  }
  // ================ PERIODIC ================

  intake_button.pressed([]() { smart_intake.intake_in(); });

  outtake_button.pressed([]() { smart_intake.intake_in(); });

  conveyor_button.pressed([]() {
    smart_intake.conveyor_start();
    smart_intake.intake_and_sort();
    // smart_intake.intake_in();
    // conveyor.spin(vex::directionType::fwd, 12, vex::volt);
    // intake();
  });

  rev_conveyor_button.pressed([]() {
    smart_intake.intake_out();
    smart_intake.conveyor_out();
  });

  goal_grabber.pressed([]() { goal_grabber_sol.set(!goal_grabber_sol); });

  con.ButtonA.pressed([]() {
        AIVisionSensor.takeSnapshot(aivision::ALL_AIOBJS);
        printf("I took snapshot. Detected %ld object(s).\n\n", AIVisionSensor.objectCount);

        for(int i = 0; i < AIVisionSensor.objectCount; i++) {
            vex::aivision::object* curr = &AIVisionSensor.objects[i];
            point_t point = cameraRelativeTest(curr->centerX, curr->originY+curr->height);
            printf("Detected a %ld at (%f, %f)\n", curr->id, point.x, point.y);
            if(i < AIVisionSensor.objectCount-1) printf("\n");
        }

        printf("-------------------\n");
    });

  while (true) {
    // printf("Current: %f\n", conveyor.current());
    if (!intake_button.pressing() && !outtake_button.pressing() && !conveyor_button.pressing() &&
        !rev_conveyor_button.pressing()) {

      smart_intake.intake_stop();
    }

    if (!conveyor_button.pressing() && !rev_conveyor_button.pressing()) {
      smart_intake.conveyor_stop();
    }

    double l = con.Axis3.position() / 100.0;
    double r = con.Axis2.position() / 100.0;
    drive_sys.drive_tank(l, r, 1, TankDrive::BrakeType::None);

    vexDelay(10);
  }
}

/*
// All AI Objects
    con.ButtonA.pressed([]() {
        sensor.takeSnapshot(aivision::ALL_AIOBJS);
        printf("I took snapshot. Detected %ld object(s).\n\n", sensor.objectCount);

        for(int i = 0; i < sensor.objectCount; i++) {
            vex::aivision::object* curr = &sensor.objects[i];
            printf("Detected a %ld at (%d, %d)\n", curr->id, curr->centerX, curr->centerY);
            sensorDistanceTest(curr->centerX, curr->originY+curr->height);
            if(i < sensor.objectCount-1) printf("\n");
        }

        printf("-------------------\n");
    });

    // All AprilTags
    con.ButtonB.pressed([]() {
        sensor.takeSnapshot(aivision::ALL_TAGS);
        printf("Took snapshot. Detected %ld tag(s).\n", sensor.objectCount);

        for(int i = 0; i < sensor.objectCount; i++) {
            vex::aivision::object* curr = &sensor.objects[i];
            printf("Detected tag %d at (%d, %d)\n", curr->id, curr->centerX, curr->centerY);
        }

        printf("-------------------\n");
    });

    // All Colors
    con.ButtonX.pressed([]() {
        sensor.takeSnapshot(aivision::ALL_COLORS);
        printf("Took snapshot. Detected %ld color(s).\n", sensor.objectCount);

        for(int i = 0; i < sensor.objectCount; i++) {
            vex::aivision::object* curr = &sensor.objects[i];
            printf("Detected color %d at (%d, %d)\n", curr->id, curr->centerX, curr->centerY);
        }

        printf("-------------------\n");
    });*/