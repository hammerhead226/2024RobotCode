package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.pivot.Pivot;

public class SetPivotTarget extends Command {
  /** Creates a new SetPivotTarget. */
  private final Pivot pivot;

  private double setPoint;

  public SetPivotTarget(double setPoint, Pivot pivot) {
    this.setPoint = setPoint;
    this.pivot = pivot;

    addRequirements(pivot);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public void initialize() {
    pivot.setPivotGoal(setPoint);
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
    return Math.abs(pivot.getPivotPositionDegs() - setPoint) <= 2;
  }
}
