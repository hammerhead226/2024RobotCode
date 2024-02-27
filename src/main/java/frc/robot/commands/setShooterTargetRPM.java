// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;


public class setShooterTargetRPM extends Command {
  /** Creates a new setShooterTargetRPM. */
  private final RobotContainer container = new RobotContainer();
  public double RightRPMSetpoint;
  public double LeftRPMSetpoint;
  public setShooterTargetRPM(double RightRPMSetpoint, double LeftRPMSetpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(container.getShooter());
    LeftRPMSetpoint = this.LeftRPMSetpoint;
    RightRPMSetpoint = this.RightRPMSetpoint;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    container.getShooter().setFlywheelRPMs(LeftRPMSetpoint, RightRPMSetpoint);;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return container.getShooter().atFlywheelSetpoints();
  }
}
