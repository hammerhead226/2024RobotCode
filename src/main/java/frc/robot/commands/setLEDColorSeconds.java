// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.LED_STATE;
import frc.robot.subsystems.led.LED;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class setLEDColorSeconds extends SequentialCommandGroup {
  /** Creates a new setLEDColorSeconds. */
  private final LED led;

  private final LED_STATE color;
  private final LED_STATE final_color;

  private final double seconds;
  public setLEDColorSeconds(LED led, LED_STATE color, double seconds,LED_STATE final_color) {
    this.led = led;
    this.color = color;
    this.seconds = seconds;
    this.final_color = final_color;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> this.led.setColor(this.color), this.led), 
      new WaitCommand(this.seconds), 
      new InstantCommand(() -> this.led.setColor(this.final_color), this.led) 
    );
  }
}
