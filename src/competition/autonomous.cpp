#include "competition/autonomous.h"
#include "robot-config.h"

/**
 * Main entrypoint for the autonomous period
 */

void skills();
void game_auto();

void autonomous() {
  while (imu.isCalibrating()) {
    vexDelay(1);
  }

  game_auto();
  // skills();
}

AutoCommand *intake_command(double amt = 12.0) {
  return new FunctionCommand([=]() {
    smart_intake.conveyor_start();
    return true;
  });
}

AutoCommand *outtake_command(double amt = 12.0) {
  return new FunctionCommand([=]() {
    smart_intake.intake_out();
    return true;
  });
}

AutoCommand *stop_intake() {
  return new FunctionCommand([=]() {
    // intake(0);
    smart_intake.intake_in();
    return true;
  });
}

AutoCommand *conveyor_intake_command(double amt = 12.0) {
  return new FunctionCommand([=]() {
    // conveyor_intake(amt);
    smart_intake.conveyor_start();
    return true;
  });
}

AutoCommand *conveyor_stop_command() {
  return new FunctionCommand([=]() {
    smart_intake.conveyor_stop();
    return true;
  });
}

AutoCommand *goal_grabber_command(bool value) {
  return new FunctionCommand([=]() {
    goal_grabber_sol.set(value);
    return true;
  });
}

AutoCommand *ring_pusher_command(bool value) {
  return new FunctionCommand([=]() {
    ring_pusher_sol.set(value);
    return true;
  });
}

class DebugCommand : public AutoCommand {
public:
  bool run() override {
    drive_sys.stop();
    pose_t pos = odom.get_position();
    printf("ODO X: %.2f, Y: %.2f, R:%.2f\n", pos.x, pos.y, pos.rot);
    printf(
      "ENC LEFT POS: %.2f, ENC RIGHT POS: %.2f, ENC BACK POS: %.2f\n", left_enc.position(vex::rotationUnits::deg),
      right_enc.position(vex::rotationUnits::deg), front_enc.position(vex::rotationUnits::deg)
    );
    while (true) {
      double f = con.Axis3.position() / 200.0;
      double s = con.Axis1.position() / 200.0;
      // double left_enc_start_pos = left_enc.position(vex::rotationUnits::rev);
      drive_sys.drive_arcade(f, s, 1, TankDrive::BrakeType::None);
      pose_t pos = odom.get_position();
      printf("ODO X: %.2f, Y: %.2f, R:%.2f\n", pos.x, pos.y, pos.rot);
      // printf("ENC LEFT REV: %.2f, ENC RIGHT POS: %.2f, ENC BACK POS: %.2f\n",
      // left_enc.position(vex::rotationUnits::deg), right_enc.position(vex::rotationUnits::deg),
      // front_enc.position(vex::rotationUnits::deg)); if (left_enc.position(vex::rotationUnits::rev) >= 1.0) {
      //     break;
      // }
      vexDelay(100);
    }
    return false;
  }
};

void game_auto() {
  CommandController cc{
    odom.SetPositionCmd({.x = 20, .y = 96, .rot = 90}),

    // drive_sys.DriveToPointCmd({48, 96}, vex::reverse, .3) ->
    // withTimeout(5.0)->withCancelCondition(drive_sys.DriveStalledCondition(0.05)),
    drive_sys.DriveForwardCmd(28, vex::reverse, .5)->withTimeout(5),

    drive_sys.TurnToPointCmd(72, 72, vex::reverse, .5)->withTimeout(3),
    // drive_sys.TurnToHeadingCmd(45, .5) -> withTimeout(3),

    drive_sys.DriveForwardCmd(34, vex::reverse, .5)->withTimeout(5),

    goal_grabber_command(true),

  };
  cc.run();
}

void skills() {
  CommandController cc{
    odom.SetPositionCmd({.x = 9.5, .y = 72, .rot = 0}),

    new Async(new FunctionCommand([]() {
      while (true) {
        OdometryBase *odombase = &odom;
        pose_t pos = odombase->get_position();
        printf("ODO X: %.2f, Y: %.2f, R:%.2f, Concurr: %f\n", pos.x, pos.y, pos.rot, conveyor.current());
        vexDelay(100);

        if ((conveyor.current() > 2) && conveyor.velocity(rpm) < 0.5) {
          printf("Conveyor Stalling");
          smart_intake.conveyor_out();
          vexDelay(500);
          smart_intake.conveyor_start();
        }
      }
      return true;
    })),

    // First Ring
    intake_command(),
    drive_sys.DriveToPointCmd({24, 72}, vex::forward, .6)->withTimeout(2),

    // Second Ring
    drive_sys.TurnToHeadingCmd(-45, .6)->withTimeout(1),
    conveyor_intake_command(10),
    new DelayCommand(350),
    conveyor_stop_command(),
    intake_command(),
    drive_sys.DriveToPointCmd({48, 48}, vex::forward, .6)->withTimeout(1),
    stop_intake(),

    // First Mogo
    drive_sys.TurnToHeadingCmd(90, .6)->withTimeout(1),
    drive_sys.DriveToPointCmd({48, 24}, vex::reverse, .6)->withTimeout(1.25),
    goal_grabber_command(true),
    new DelayCommand(200),
    conveyor_intake_command(10),
    intake_command(),

    // Third Ring
    drive_sys.TurnToHeadingCmd(0, 0.6)->withTimeout(1),
    drive_sys.DriveToPointCmd({72, 24}, vex::forward, 0.7)->withTimeout(2),

    // Fourth Ring
    drive_sys.TurnToHeadingCmd(-90, 0.7)->withTimeout(0.8),
    drive_sys.DriveToPointCmd({72, 13}, vex::forward, 0.6)->withTimeout(0.5),

    // Fifth Ring
    drive_sys.DriveToPointCmd({72, 24}, vex::reverse, 0.3)->withTimeout(2),
    drive_sys.TurnToPointCmd(24, 24, vex::forward, 0.6)->withTimeout(1),
    drive_sys.DriveToPointCmd({22, 24}, vex::forward, 0.7)->withTimeout(2),

    // Sixth Ring
    drive_sys.TurnToHeadingCmd(-135, 0.6)->withTimeout(1.25),
    drive_sys.DriveToPointCmd({12, 12}, vex::forward, 0.6)->withTimeout(1),
    new DelayCommand(1000),
    drive_sys.DriveForwardCmd(8, vex::reverse, 0.6)->withTimeout(0.5),

    // Drop
    new DelayCommand(1000),
    conveyor_intake_command(-10),
    new DelayCommand(100),
    conveyor_stop_command(),
    drive_sys.TurnToHeadingCmd(45, 0.6)->withTimeout(1.25),
    drive_sys.DriveToPointCmd({14, 14}, vex::reverse, 0.6)->withTimeout(1),
    goal_grabber_command(false),
    new DelayCommand(500),
    drive_sys.DriveToPointCmd({24, 24}, vex::forward, 0.6)->withTimeout(1),
    // SECOND HALF
    drive_sys.TurnToHeadingCmd(45, 0.6)->withTimeout(1),
    // drive_sys.DriveTankCmd(0.5, 0.5),
    drive_sys.DriveForwardCmd(95, vex::forward, 0.8)->withTimeout(4),
    // new DebugCommand,

    // First ring
    conveyor_stop_command(),
    drive_sys.TurnToHeadingCmd(0, 0.6)->withTimeout(1),
    drive_sys.DriveToPointCmd({96, 24}, vex::forward, 0.6)->withTimeout(3),
    odom.SetPositionCmd({96, 24, 0}),

    // Second Mogo
    drive_sys.TurnToHeadingCmd(-90, 0.6)->withTimeout(1),
    drive_sys.DriveToPointCmd({96, 48}, vex::reverse, 0.6)->withTimeout(1),
    goal_grabber_command(true),
    conveyor_intake_command(),

    // Second Ring
    drive_sys.TurnToHeadingCmd(0, 0.6)->withTimeout(1),
    drive_sys.DriveToPointCmd({120, 48}, vex::forward)->withTimeout(1),

    // Third Ring
    drive_sys.TurnToHeadingCmd(-90, 0.6)->withTimeout(1),
    drive_sys.DriveToPointCmd({120, 24}, vex::forward, 0.6)->withTimeout(1),

    // Fourth Ring
    // pickup blue first
    drive_sys.TurnToPointCmd(144, 0, vex::forward, 0.6)->withTimeout(0.6),
    new DelayCommand(500),
    conveyor_stop_command(),
    // go into corner
    drive_sys.DriveForwardCmd(8, vex::forward, 0.6)->withTimeout(1),
    // go outta corner
    drive_sys.DriveForwardCmd(-8, vex::reverse, 0.6)->withTimeout(1),
    // turn sideways to spit
    drive_sys.TurnToHeadingCmd(-135, 0.6)->withTimeout(1),
    outtake_command(),
    // push it a bit
    drive_sys.DriveForwardCmd(6, vex::forward, 0.4)->withTimeout(1),
    drive_sys.DriveForwardCmd(6, vex::reverse, 0.4)->withTimeout(1),
    intake_command(),
    // go into corner for red
    drive_sys.TurnToPointCmd(144, 0, vex::forward, 0.6)->withTimeout(1),
    drive_sys.DriveToPointCmd({132, 12}, vex::forward, 0.6)->withTimeout(1),
    conveyor_intake_command(),
    new DelayCommand(1000),
    // go back out and rotate
    drive_sys.DriveForwardCmd(8, vex::reverse, 0.6)->withTimeout(1),
    drive_sys.TurnToHeadingCmd(135, 0.6)->withTimeout(1),
    // put mogo in corner
    goal_grabber_command(false),
    drive_sys.DriveToPointCmd({132, 12}, vex::reverse, 0.6)->withTimeout(2),
    stop_intake(),
    conveyor_stop_command(),
    new DelayCommand(100),
    // drive into ladder
    // drive_sys.TurnToHeadingCmd(135, 0.6)->withTimeout(1),
    drive_sys.DriveForwardCmd(72, vex::forward, 0.8)->withTimeout(3),

  };
  cc.run();
}