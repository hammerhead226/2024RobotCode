// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LED;

import frc.robot.Constants.LED_STATE;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface LED_IO {
  @AutoLog
  public static class LED_IOInputs {
    public LED_STATE ledState = LED_STATE.OFF;
  }

  public default void updateInputs(LED_IOInputs inputs) {}

  public default void noBumpersPressed() {}

  public default void setColor(LED_STATE state) {}
}
