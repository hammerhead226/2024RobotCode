// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.LED_STATE;
import frc.robot.subsystems.led.LED;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class setFlashingLEDColor3Seconds extends SequentialCommandGroup {
  /** Creates a new setFlashingLEDColorSeconds. */
  public setFlashingLEDColor3Seconds(LED led, LED_STATE color) {
    addCommands(
     new setLEDColorSeconds(led, color, 0.5,LED_STATE.OFF),
     new setLEDColorSeconds(led, LED_STATE.OFF, 0.5, color),
     new setLEDColorSeconds(led, color, 0.5,LED_STATE.OFF),
     new setLEDColorSeconds(led, LED_STATE.OFF, 0.5, color),
     new setLEDColorSeconds(led, color, 0.5,LED_STATE.OFF),
     new setLEDColorSeconds(led, LED_STATE.OFF, 0.5, LED_STATE.BLUE)
    );
  }
}
