// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface IntakeRollerIO {
  @AutoLog
  public static class IntakeRollerIOInputs {
    public double rollerVelocityRPM = 0.0;
    public double rollerRotations = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  public default void updateInputs(IntakeRollerIOInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void stop() {}
}
