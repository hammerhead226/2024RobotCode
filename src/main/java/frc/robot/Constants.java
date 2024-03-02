// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static enum Mode {
    REAL,
    SIM,
    REPLAY
  }

  public static Mode getMode() {
    return switch (currentMode) {
      case REAL -> Mode.REAL;
      case SIM -> Mode.SIM;
      case REPLAY -> Mode.REPLAY;
    };
  }

  public static final Mode currentMode = Mode.REAL;
  public static final boolean tuningMode = true;
  public static final String CANBUS = "CAN Bus 2";
  public static final double LOOP_PERIOD_SECS = 0.02;

  public static class SwerveConstants {
    public static final double MAX_LINEAR_SPEED = 5.56;
    public static final double TRACK_WIDTH_X = Units.inchesToMeters(26.0);
    public static final double TRACK_WIDTH_Y = Units.inchesToMeters(26.0);
    public static final double DRIVE_BASE_RADIUS =
        Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
    public static final double MAX_ANGULAR_SPEED = 0.6 * MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;
    public static final double OPEN_LOOP_RAMP_SEC = 0.5;
  }

  public static class ModuleConstants {
    public static final double WHEEL_RADIUS = Units.inchesToMeters(2.0);

    public static final double DRIVE_GEAR_RATIO = 6.12;
    public static final double TURN_GEAR_RATIO = 150.0 / 7.0;

    public static final double DRIVE_STATOR_CURRENT_LIMIT = 40.0;
    public static final boolean DRIVE_STATOR_CURRENT_LIMIT_ENABLED = true;
    public static final double TURN_STATOR_CURRENT_LIMIT = 30.0;
    public static final boolean TURN_STATOR_CURRENT_LIMIT_ENABLED = true;
  }

  public static class IntakeConstants {
    public static final int CURRENT_LIMIT = 30;
    public static final int APPLIED_VOLTAGE = 12;
    public static final boolean CURRENT_LIMIT_ENABLED = true;
  }

  public static final class ShooterConstants {
    public static final double FEEDER_CURRENT_LIMIT = 40;
    public static final boolean FEEDER_CURRENT_LIMIT_ENABLED = true;

    public static final double FEEDER_THRESHOLD = 0;

    public static final double FLYWHEEL_CURRENT_LIMIT = 40;
    public static final boolean FLYWHEEL_CURRENT_LIMIT_ENABLED = true;

    public static final double FLYWHEEL_THRESHOLD = 0;

    public static final double FLYWHEEL_AMP_RPM = 600;
    public static final double FLYWHEEL_SHOOT_RPM = 3000;
  }

  public static class ElevatorConstants {
    public static final double CURRENT_LIMIT = 30.0;
    public static final boolean CURRENT_LIMIT_ENABLED = true;

    public static final double RETRACT_SETPOINT_INCH = 0;
    public static final double EXTEND_SETPOINT_INCH = 0;
    public static final double THRESHOLD = 3;

    public static final double[] PID = {0, 0, 0};

    public static final double REDUCTION = (15.0 / 1.0);
  }

  public static class PivotConstants {
    public static final double CURRENT_LIMIT = 35.0;
    public static final boolean CURRENT_LIMIT_ENABLED = true;

    public static final double PODIUM_SETPOINT_DEG = 43.0;
    public static final double STOW_SETPOINT_DEG = 52.7;
    public static final double INTAKE_SETPOINT_DEG = 56.0;
    public static final double AMP_SETPOINT_DEG = 60.0;
    public static final double SUBWOOFER_SETPOINT_DEG = 61.0;
    public static final double REVERSE_SUBWOOFER_SETPOINT_DEG = 118;
    public static final double CLIMB_SETPOINT_ONE_DEG = 50;
    public static final double CLIMB_SETPOINT_TWO_DEG = 110;

    public static final double THRESHOLD = 2;

    public static final double[] PID = {0, 0, 0};

    public static final double REDUCTION =
        (15.0 / 1.0) * (34.0 / 24.0) * (24.0 / 18.0) * (50.0 / 14.0);
  }

  public static class LEDConstants {
    public static final double COLOR_BLUE = 0.87;
    public static final double COLOR_RED = 0.61;
    public static final double COLOR_YELLOW = 0.66;
    public static final double COLOR_VIOLET = 0.91;
  }

  public static enum LED_STATE {
    BLUE,
    RED,
    YELLOW,
    VIOLET,
    OFF
  }
}
