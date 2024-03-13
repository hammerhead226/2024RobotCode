// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.FieldConstants;

public class TurnToSpeaker extends Command {
  private final Drive drive;
  private final Shooter shooter;
  private final Pivot pivot;
  private final CommandXboxController controller;
  private final PIDController pid;
  private double[] gains = new double[3];
  private DriverStation.Alliance alliance = null;
  private double distanceToSpeakerMeter = 0;
  private double shooterSetpointRPM = 0;
  private double pivotSetpointDeg = 0;
  /** Creates a new TurnToSpeaker. */
  public TurnToSpeaker(
      Drive drive, Shooter shooter, Pivot pivot, CommandXboxController controller) {
    this.drive = drive;
    this.shooter = shooter;
    this.pivot = pivot;

    this.controller = controller;
    addRequirements(drive);

    switch (Constants.currentMode) {
      case REAL:
        gains[0] = 0.5;
        gains[1] = 0;
        gains[2] = 0;
        break;
      case REPLAY:
        gains[0] = 0;
        gains[1] = 0;
        gains[2] = 0;
        break;
      case SIM:
        gains[0] = 13;
        gains[1] = 0;
        gains[2] = 01;
        break;
      default:
        gains[0] = 0;
        gains[1] = 0;
        gains[2] = 0;
        break;
    }

    pid = new PIDController(gains[0], gains[1], gains[2], 0.02);
    pid.setTolerance(0);
    pid.enableContinuousInput(-180, 180);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (DriverStation.getAlliance().isPresent()) this.alliance = DriverStation.getAlliance().get();

    if (alliance == DriverStation.Alliance.Red) {
      pid.setSetpoint(
          new Rotation2d(
                  FieldConstants.fieldLength - drive.getPose().getX(),
                  FieldConstants.Speaker.speakerCenterY - drive.getPose().getY())
              .getDegrees());
    } else {
      pid.setSetpoint(
          new Rotation2d(
                  -drive.getPose().getX(),
                  FieldConstants.Speaker.speakerCenterY - drive.getPose().getY())
              .getDegrees());
    }
    double linearMagnitude =
        MathUtil.applyDeadband(Math.hypot(-controller.getLeftY(), -controller.getLeftX()), 0.1);
    Rotation2d linearDirection = new Rotation2d(-controller.getLeftY(), -controller.getLeftX());

    // Square values
    linearMagnitude = linearMagnitude * linearMagnitude;
    // Calcaulate new linear velocity
    Translation2d linearVelocity =
        new Pose2d(new Translation2d(), linearDirection)
            .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
            .getTranslation();
    double angularSpeed = pid.calculate(drive.getPose().getRotation().getDegrees());

    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
            linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
            Math.toRadians(angularSpeed),
            drive.getPose().getRotation()));

    distanceToSpeakerMeter = 0; // TODO distanceToSpeaker = equation;
    shooter.setFlywheelRPMs(
        calculateShooterSpeedRPM(distanceToSpeakerMeter),
        calculateShooterSpeedRPM(distanceToSpeakerMeter));
    pivot.setPivotGoal(calculatePivotAngleDeg(distanceToSpeakerMeter));
  }

  private double calculateShooterSpeedRPM(double distanceToSpeakerMeter) {
    shooterSetpointRPM = 0; // TODO shooterSetpointDeg = equation
    return shooterSetpointRPM;
  }

  private double calculatePivotAngleDeg(double distanceToSpeakerMeter) {
    pivotSetpointDeg = (-0.276 * Math.abs(Units.metersToInches(distanceToSpeakerMeter) - 36) + 62);
    return pivotSetpointDeg;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pid.atSetpoint();
  }
}
