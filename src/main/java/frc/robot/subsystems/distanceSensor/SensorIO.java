// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.distanceSensor;

/** Add your docs here. */
public interface SensorIO {
  public static class SensorIOInputs {
    public boolean isNoteThere;
  }

  public default void updateInputs(SensorIOInputs inputs) {}

  public default boolean getIfNoteThere(double distanceIn) {return false;}

}
