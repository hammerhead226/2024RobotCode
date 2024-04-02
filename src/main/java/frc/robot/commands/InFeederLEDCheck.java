// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LED_STATE;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.shooter.Shooter;

public class InFeederLEDCheck extends Command {
  /** Creates a new InFeederLEDCheck. */
  Shooter shooter;

  LED led;

  public InFeederLEDCheck(Shooter shooter, LED led) {
    this.shooter = shooter;
    this.led = led;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (shooter.seesNote()) {
      led.setState(LED_STATE.GREEN);
    } else led.setState(LED_STATE.BLUE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
