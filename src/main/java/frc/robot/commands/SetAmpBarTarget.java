// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import org.littletonrobotics.junction.Logger;

public class SetAmpBarTarget extends Command {
  /** Creates a new SetAmpBarTarget. */
  private final Elevator elevator;

  private double threshold;
  private double setPoint;

  public SetAmpBarTarget(double setpoint, double threshold, Elevator elevator) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.setPoint = setpoint;
    this.threshold = threshold;
    this.elevator = elevator;

    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.setBarGoal(setPoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.stopAmpBar();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Logger.recordOutput("bar command ", elevator.ampBarAtGoal());
    return elevator.ampBarAtGoal();
  }
}
