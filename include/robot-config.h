#pragma once
#include "core.h"
#include "intake.h"
#include "ai_vision_sensor.h"
#include "vex.h"

extern vex::brain Brain;
extern vex::controller con;

extern vex::inertial imu;
extern vex::optical conveyor_optical;
extern vex::triport expander;

extern vex::aivision AIVisionSensor;

// Analog sensors
extern CustomEncoder left_enc;
extern CustomEncoder right_enc;
extern CustomEncoder front_enc;

// ================ OUTPUTS ================
// Motors
extern vex::motor left_front_top;
extern vex::motor left_front_bottom;
extern vex::motor left_back_top;
extern vex::motor left_back_bottom;

extern vex::motor right_front_top;
extern vex::motor right_front_bottom;
extern vex::motor right_back_top;
extern vex::motor right_back_bottom;

extern vex::motor intake_roller;
extern vex::motor intake_ramp;
extern vex::motor conveyor;

extern vex::optical conveyor_sensor;

extern Intake smart_intake;

extern vex::motor_group left_motors;
extern vex::motor_group right_motors;
// Pneumatics
extern vex::digital_out goal_grabber_sol;
extern vex::digital_out ring_pusher_sol;

// ================ SUBSYSTEMS ================
extern PID drive_pid;
extern PID turn_pid;

extern PID::pid_config_t drive_correction_pid;

extern robot_specs_t robot_cfg;

extern OdometryNWheel<3> odom;
// extern OdometryTank odom;
extern TankDrive drive_sys;

/**
 * Main robot initialization on startup. Runs before opcontrol and autonomous are started.
 */
void robot_init();