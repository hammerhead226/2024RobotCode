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

  public static final Mode currentMode = Mode.REAL;
  public static final boolean tuningMode = true;
  public static final String CANBUS = "CAN Bus 2";

  public static class SwerveConstants {
    public static final double MAX_LINEAR_SPEED = 5.56;
    public static final double TRACK_WIDTH_X = Units.inchesToMeters(26.0);
    public static final double TRACK_WIDTH_Y = Units.inchesToMeters(26.0);
    public static final double DRIVE_BASE_RADIUS =
        Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
    public static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;
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
    public static final int ROLLER_CURRENT_LIMIT = 30;
    public static final boolean ROLLER_TALON_FX_CURRENT_LIMIT_ENABLED = true;
  }

  public static final class ShooterConstants {
    public static final double FEEDER_CURRENT_LIMIT = 0;
    public static final boolean FEEDER_CURRENT_LIMIT_ENABLED = true;

    public static final double FLYWHEEL_CURRENT_LIMIT = 0;
    public static final boolean FLYWHEEL_CURRENT_LIMIT_ENABLED = true;

    public static final double FLYWHEEL_THRESHOLD = 0;
  }

  public static final double LOOP_PERIOD_SECS = 0.02;

  public static class ElevatorConstants {
    public static final double EXTENDER_CURRENT_LIMIT = 30.0;
    public static final double PIVOT_CURRENT_LIMIT = 30.0;

    public static final boolean EXTENDER_CURRENT_LIMIT_ENABLED = true;
    public static final boolean PIVOT_CURRENT_LIMIT_ENABLED = true;

    public static final double PIVOT_STOW = 0;
    public static final double EXTENDER_RETRACT = 0;
    public static final double EXTENDER_EXTEND = 0;

    public static final double PIVOT_THRESHOLD = 0;
    public static final double EXTENDER_THRESHOLD = 0;

    public static final double[] PIVOT_PID = {0, 0, 0};
    public static final double[] EXTENDER_PID = {0, 0, 0};
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
