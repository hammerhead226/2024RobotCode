// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;

public class AlignToNoteAuto extends Command {
  /** Creates a new AlignToNoteAuto. */
  private final Drive drive;

  private DriverStation.Alliance alliance = null;
  private final PIDController xPID;
  private final PIDController yPID;

  public AlignToNoteAuto(Drive drive) {
    double xKP;
    double xKI;
    double xKD;

    double yKP;
    double yKI;
    double yKD;

    switch (Constants.getMode()) {
      case REAL:
        xKP = 1;
        xKI = 0;
        xKD = 0;

        yKP = 1;
        yKI = 0;
        yKD = 0;
        break;

      default:
        xKP = 1;
        xKI = 0;
        xKD = 0;

        yKP = 1;
        yKI = 0;
        yKD = 0;
        break;
    }

    yPID = new PIDController(yKP, yKI, yKD);
    xPID = new PIDController(xKP, xKI, xKD);

    xPID.setTolerance(5);
    yPID.setTolerance(0.1);
    this.drive = drive;
    addRequirements(drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    yPID.setSetpoint(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (DriverStation.getAlliance().isPresent()) this.alliance = DriverStation.getAlliance().get();

    if (alliance == DriverStation.Alliance.Red) {
      drive.runVelocity(
          ChassisSpeeds.fromFieldRelativeSpeeds(
              xPID.calculate(drive.getNoteError()),
              yPID.calculate(drive.getPose().getY()),
              0,
              drive.getPose().getRotation()));
    } else {
      drive.runVelocity(
          ChassisSpeeds.fromFieldRelativeSpeeds(
              -xPID.calculate(drive.getNoteError()),
              yPID.calculate(drive.getPose().getY()),
              0,
              drive.getPose().getRotation()));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return xPID.atSetpoint() && yPID.atSetpoint();
  }
}
