/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       unknown                                                   */
/*    Created:      12/1/2024, 11:02:31 AM                                    */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "core.h"
#include "vex.h"
#include "ai_vision_sensor.h"
using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

vex::brain Brain;
vex::controller con;

vex::inertial imu(vex::PORT10, vex::turnType::right);
vex::optical conveyor_optical(vex::PORT17);
vex::triport expander(vex::PORT18);

// Analog sensors
CustomEncoder left_enc{Brain.ThreeWirePort.C, 2048};
CustomEncoder right_enc{Brain.ThreeWirePort.E, 2048};
CustomEncoder front_enc{Brain.ThreeWirePort.G, 2048};

tracking_wheel_cfg_t left_enc_cfg{-0.0625, 3.625, M_PI, 1.0625};
tracking_wheel_cfg_t right_enc_cfg{-0.0625, -3.625, 0, 1.0625};
tracking_wheel_cfg_t front_enc_cfg{4.5, -0.375, (M_PI/2), 1.0625};

aivision sensor(vex::PORT9);

// ================ OUTPUTS ================
// Motors
vex::motor left_front_top(vex::PORT8, vex::gearSetting::ratio6_1, true);
vex::motor left_front_bottom(vex::PORT6, vex::gearSetting::ratio6_1, true);
vex::motor left_back_top(vex::PORT7, vex::gearSetting::ratio6_1, false);
vex::motor left_back_bottom(vex::PORT5, vex::gearSetting::ratio6_1, true);

vex::motor right_front_top(vex::PORT3, vex::gearSetting::ratio6_1, false);
vex::motor right_front_bottom(vex::PORT2, vex::gearSetting::ratio6_1, false);
vex::motor right_back_top(vex::PORT1, vex::gearSetting::ratio6_1, true);
vex::motor right_back_bottom(vex::PORT4, vex::gearSetting::ratio6_1, false);

vex::motor intake_roller(vex::PORT19, vex::gearSetting::ratio6_1, false);
vex::motor intake_ramp(vex::PORT20, vex::gearSetting::ratio6_1, false);
vex::motor conveyor(vex::PORT9, vex::gearSetting::ratio18_1, true);

std::map<std::string, vex::motor &> motor_names{
  {"left front top", left_front_top},   {"left front bottom", left_front_bottom},   {"left back top", left_back_top},   {"left back bottom", left_back_bottom},

  {"right front top", right_front_top},   {"right front bottom", right_front_bottom},   {"right back top", right_back_top},   {"right back bottom", right_back_bottom},

};

vex::motor_group left_motors{left_front_top, left_front_bottom, left_back_top, left_back_bottom};
vex::motor_group right_motors{right_front_top, right_front_bottom, right_back_top, right_back_bottom};

// ================ SUBSYSTEMS ================

PID::pid_config_t drive_pid_cfg{
    .p = 0.045,
    .i = 0.0,
    .d = 0.0003,
    .deadband = 0.5,
    .on_target_time = 0.1,
};

PID drive_pid{drive_pid_cfg};

PID::pid_config_t turn_pid_cfg{
    .p = 0.01625,
    .i = 0.005,
    .d = 0.001,
    //.deadband = 1.5,
    .deadband = 2,
    .on_target_time = 0.1,
};

PID turn_pid{turn_pid_cfg};

PID::pid_config_t drive_correction_pid{};

robot_specs_t robot_cfg{
    .robot_radius = 12.0,
    .odom_wheel_diam = 2.125,
    .odom_gear_ratio = 1.0,
    .dist_between_wheels = 12.5,

    // .drive_correction_cutoff = 0,
    .drive_feedback = &drive_pid,
    .turn_feedback = &turn_pid,
    .correction_pid = drive_correction_pid,
};

OdometryNWheel<3> odom({left_enc, right_enc, front_enc}, {left_enc_cfg, right_enc_cfg, front_enc_cfg}, &imu, true);
// OdometryTank odom(left_enc, right_enc, robot_cfg, &imu);
TankDrive drive_sys(left_motors, right_motors, robot_cfg, &odom);

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {

    // All activities that occur before the competition starts
    // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
    // ..........................................................................
    // Insert autonomous user code here.
    // ..........................................................................
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
    // User control code here, inside the loop
    sensor.modelDetection(true);

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
    });

    while (1) {
        // This is the main execution loop for the user control program.
        // Each time through the loop your program should update motor + servo
        // values based on feedback from the joysticks.

        double straight = (double)con.Axis3.position() / 100;
        double turn = (double)con.Axis1.position() / 100;

        drive_sys.drive_arcade(straight, turn * 0.75, 1, TankDrive::BrakeType::None);

        wait(20, msec); // Sleep the task for a short amount of time to
                        // prevent wasted resources.
    }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
    // Set up callbacks for autonomous and driver control periods.
    Competition.autonomous(autonomous);
    Competition.drivercontrol(usercontrol);

    // Run the pre-autonomous function.
    pre_auton();

    // Prevent main from exiting with an infinite loop.
    while (true) {
        wait(100, msec);
    }
}
