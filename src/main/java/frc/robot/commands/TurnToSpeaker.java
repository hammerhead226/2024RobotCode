// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;

public class TurnToSpeaker extends Command {
  private final Drive drive;
  private final PIDController pid;
  private double[] gains = new double[3];
  /** Creates a new TurnToSpeaker. */
  public TurnToSpeaker(Drive drive) {
    this.drive = drive;
    addRequirements(drive);

    switch (Constants.currentMode) {
      case REAL:
        gains[0] = 0;
        gains[1] = 0;
        gains[2] = 0;
        break;
      case REPLAY:
        gains[0] = 0;
        gains[1] = 0;
        gains[2] = 0;
        break;
      case SIM:
        gains[0] = 0;
        gains[1] = 0;
        gains[2] = 0;
        break;
      default:
        gains[0] = 0;
        gains[1] = 0;
        gains[2] = 0;
        break;
    }
    
    pid = new PIDController(gains[0], gains[1], gains[2]);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

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
