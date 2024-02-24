package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator.Elevator;

public class SetPivotTarget extends Command {
  /** Creates a new SetPivotTarget. */
  private final Elevator elevator;

  private double setPoint;

  public SetPivotTarget(double setPoint, Elevator elevator) {
    setPoint = this.setPoint;
    this.elevator = elevator;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public void initialize() {
    elevator.setPivotGoal(setPoint);
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
    return elevator.pivotAtSetpoint();
  }
}
