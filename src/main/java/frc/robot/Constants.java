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

  public static final String CANBUS = "CAN Bus 2";


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
    public static final double[] EXTENDER_PID = {0, 0 , 0};

  }

  

 
}
