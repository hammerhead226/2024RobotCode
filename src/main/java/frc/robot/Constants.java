// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class currentLimitsConstants {
    public static final double EXTENDER_TALON_FX_CURRENT_LIMIT = 30.0;
    public static final double PIVOT_TALON_FX_CURRENT_LIMIT = 30.0;

    public static final boolean EXTENDER_TALON_FX_CURRENT_LIMIT_ENABLED = true;
    public static final boolean PIVOT_TALON_FX_CURRENT_LIMIT_ENABLED = true;
  }

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final Mode currentMode = Mode.SIM;
  public static final boolean tuningMode = true;

  public static final String CANBUS_STRING = "Can Bus 1";
  public static final String CANIVORE_STRING = "Can Bus 2";

  public static final double PIVOT_HEIGHT = 6.25;
  // Still need to find the SHOOTER_LENGTH
  public static final double SHOOTER_LENGTH = 0; // replace with actual shooter length

  // TODO change all IDS here
  public static final int CANCODER_CANBUS_ID = 0;
  public static final int ELEVATOR_PIVOT_ID = 0;
  public static final int ELEVATOR_EXTENDER_ID = 0;

  // Shooter Testing data points
  public static double xDataPoints[] = {96.3, 109.8, 126.3, 145.3, 139};
  public static double yDataPoints[] = {47, 45, 37, 34, 30};

  public static final double LOOP_PERIOD_SECS = 0.02;
}
