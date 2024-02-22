// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.distanceSensor;

/** Add your docs here. */
public interface SensorIO {
  public static class SensorIOInputs {
    public double distance;
  }

  public default void updateInputs(SensorIOInputs inputs) {}

  public default double getDistance() {return 0.0;}

}
