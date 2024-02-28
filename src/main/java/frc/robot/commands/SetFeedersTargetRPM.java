// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

public class SetFeedersTargetRPM extends Command {
  /** Creates a new SetFeedersRPM. */
  private final Shooter shooter;

  private final double rpm;

  public SetFeedersTargetRPM(double rpm, Shooter shooter) {
    this.rpm = rpm;
    this.shooter = shooter;

    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setFeedersRPM(rpm);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  // TODO needs to be implemented
  @Override
  public boolean isFinished() {
    return shooter.atFeederSetpoint();
  }
}