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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.FieldConstants;
import org.littletonrobotics.junction.Logger;

public class TurnToAmpCorner extends Command {
  private final Drive drive;
  private final Pivot pivot;
  private final Shooter shooter;
  private final CommandXboxController controller;
  private final PIDController pid;
  private double[] gains = new double[3];
  private DriverStation.Alliance alliance = null;
  /** Creates a new TurnToSpeaker. */
  public TurnToAmpCorner(
      Drive drive, Pivot pivot, Shooter shooter, CommandXboxController controller) {
    this.drive = drive;
    this.pivot = pivot;
    this.shooter = shooter;

    this.controller = controller;
    addRequirements(drive);

    switch (Constants.currentMode) {
      case REAL:
        gains[0] = 4;
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
    pid.setTolerance(4);
    pid.enableContinuousInput(-180, 180);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pivot.setPivotGoal(45);
    shooter.setFlywheelRPMs(5200, 4900);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double targetAngle;
    if (DriverStation.getAlliance().isPresent()) this.alliance = DriverStation.getAlliance().get();

    if (alliance == DriverStation.Alliance.Red) {
      Logger.recordOutput(
          "trans2",
          new Translation2d(
              FieldConstants.fieldLength - drive.getPose().getX(),
              FieldConstants.fieldWidth - drive.getPose().getY()));
      targetAngle =
          new Rotation2d(
                      FieldConstants.fieldLength - drive.getPose().getX(),
                      FieldConstants.fieldWidth - drive.getPose().getY())
                  .getDegrees()
              + 180;
      pid.setSetpoint(
          new Rotation2d(
                      FieldConstants.fieldLength - drive.getPose().getX(),
                      FieldConstants.fieldWidth - drive.getPose().getY())
                  .getDegrees()
              + 180);
      // pid.setSetpoint(
      //     new Rotation2d(
      //                 FieldConstants.fieldLength,
      //                 FieldConstants.fieldWidth)
      //             .getDegrees()
      //         + 180);
    } else {
      // Logger.recordOutput("trans2", new Translation2d(0, FieldConstants.fieldWidth));
      targetAngle = new Translation2d(0, FieldConstants.fieldWidth).getAngle().getDegrees();
      // new Rotation2d(0, FieldConstants.fieldWidth).getDegrees() + 180;
      // pid.setSetpoint(
      //     new Rotation2d(0, FieldConstants.fieldWidth).getDegrees()
      //         + 180);
      pid.setSetpoint(
          new Rotation2d(
                      -drive.getPose().getX(), FieldConstants.fieldWidth - drive.getPose().getY())
                  .getDegrees()
              + 180);
      // pid.setSetpoint(
      // new Translation2d(0, FieldConstants.fieldWidth).getAngle().getDegrees());
    }

    Logger.recordOutput("target angle", targetAngle);
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
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setFeedersRPM(500);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pid.atSetpoint() && pivot.atGoal() && shooter.atFlywheelSetpoints();
  }
}
