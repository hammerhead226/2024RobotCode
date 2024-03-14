// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.FieldConstants;

public class AngleShooter extends Command {
  /** Creates a new AngleShooter. */
  private final Drive drive;
  private final Shooter shooter;
  private final Pivot pivot;

  private DriverStation.Alliance alliance = null;
  private double distanceToSpeakerMeter = 0;
  private double shooterSetpointRPM = 0;
  private double pivotSetpointDeg = 0;

  public AngleShooter(Drive drive, Shooter shooter, Pivot pivot) {
    this.drive = drive;
    this.shooter = shooter;
    this.pivot = pivot;

    addRequirements(shooter, pivot);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (DriverStation.getAlliance().isPresent()) this.alliance = DriverStation.getAlliance().get();

    distanceToSpeakerMeter = 0; // TODO distanceToSpeaker = equation;
    shooter.setFlywheelRPMs(
        5400,
        5400);
    pivot.setPivotGoal(calculatePivotAngleDeg(distanceToSpeakerMeter));
  }


  private double calculatePivotAngleDeg(double distanceToSpeakerMeter) {
    pivotSetpointDeg = (-0.276 * Math.abs(Units.metersToInches(distanceToSpeakerMeter) - 36) + 62);
    return pivotSetpointDeg;
  }

  private double calculateDistanceToSpeaker() {
    double x = 0;
    double y = 0;

    if (alliance == DriverStation.Alliance.Red) {
      x = FieldConstants.fieldLength - drive.getPose().getX();
      y = FieldConstants.Speaker.speakerCenterY - drive.getPose().getY();
    } else {
      x = -drive.getPose().getX();
      y = FieldConstants.Speaker.speakerCenterY - drive.getPose().getY();
    }

    return Math.hypot(x, y);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
