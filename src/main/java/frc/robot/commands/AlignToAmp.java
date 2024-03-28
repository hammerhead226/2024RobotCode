// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.FieldConstants;
import org.littletonrobotics.junction.Logger;

public class AlignToAmp extends Command {
  private Translation2d ampCoord;
  private Rotation2d ampAngle;
  private final Drive drive;
  private final PIDController pid;
  private double[] gains = new double[3];
  private DriverStation.Alliance alliance = null;
  /** Creates a new AlignToAmp. */
  public AlignToAmp(Drive drive) {
    this.drive = drive;
    switch (Constants.currentMode) {
      case REAL:
        gains[0] = 7;
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
    pid.setTolerance(5);
    pid.enableContinuousInput(-180, 180);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (DriverStation.getAlliance().get().equals(Alliance.Red)) {
      ampCoord =
          new Translation2d(
              FieldConstants.fieldLength - FieldConstants.ampCenter.getX(),
              FieldConstants.ampCenter.getY() - 0.5);
      ampAngle = new Rotation2d(FieldConstants.ampCenter.getX(), FieldConstants.ampCenter.getY());
    } else {
      ampCoord =
          new Translation2d(
              FieldConstants.fieldLength - FieldConstants.ampCenter.getX(),
              FieldConstants.ampCenter.getY() - 0.5);
      ampAngle = new Rotation2d(FieldConstants.ampCenter.getX(), FieldConstants.ampCenter.getY());
    }

    Logger.recordOutput("ampCoord", ampCoord);
    Logger.recordOutput("ampAngle", ampAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.driveTo(ampCoord, ampAngle);
    // pid.setSetpoint(ampAngle.getDegrees());

    // double angularSpeed = pid.calculate(drive.getPose().getRotation().getDegrees());

    // drive.runVelocity(
    //     ChassisSpeeds.fromFieldRelativeSpeeds(
    //         0, 0, Math.toRadians(angularSpeed), drive.getPose().getRotation()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(drive.getPose().getTranslation().getDistance(ampCoord)) < 1 && pid.atSetpoint();
  }
}
