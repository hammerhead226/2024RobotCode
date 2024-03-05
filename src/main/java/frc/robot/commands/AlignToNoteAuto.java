// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class AlignToNoteAuto extends Command {
  /** Creates a new AlignToNoteAuto. */
  private final Drive drive;

  private DriverStation.Alliance alliance = null;
  private final PIDController xPID;
  private final PIDController yPID;

  private final LoggedTunableNumber xKp = new LoggedTunableNumber("AlignToNoteAuto/xKp");
  private final LoggedTunableNumber xKd = new LoggedTunableNumber("AlignToNoteAuto/xKd");

  private final LoggedTunableNumber yKp = new LoggedTunableNumber("AlignToNoteAuto/yKp");
  private final LoggedTunableNumber yKd = new LoggedTunableNumber("AlignToNoteAuto/yKd");

  public AlignToNoteAuto(Drive drive) {

    switch (Constants.getMode()) {
      case REAL:
        xKp.initDefault(0.045);
        xKd.initDefault(0);

        yKp.initDefault(0.08);
        yKd.initDefault(0);

        break;

      default:
        break;
    }

    yPID = new PIDController(yKp.get(), 0, yKd.get());
    xPID = new PIDController(xKp.get(), 0, xKd.get());

    xPID.setTolerance(5);
    yPID.setTolerance(5);
    this.drive = drive;
    addRequirements(drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    yPID.setSetpoint(-18);
    xPID.setSetpoint(-4);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.recordOutput("TA", LimelightHelpers.getTA(Constants.LL_INTAKE));

    if (DriverStation.getAlliance().isPresent()) this.alliance = DriverStation.getAlliance().get();

    double noteError = drive.getNoteError();
    double distanceError = LimelightHelpers.getTY(Constants.LL_INTAKE);

    double xPIDEffort =
        MathUtil.clamp(
            xPID.calculate(noteError),
            -Constants.SwerveConstants.MAX_LINEAR_SPEED,
            Constants.SwerveConstants.MAX_LINEAR_SPEED);
    double yPIDEffort =
        MathUtil.clamp(
            yPID.calculate(LimelightHelpers.getTY(Constants.LL_INTAKE)),
            -Constants.SwerveConstants.MAX_LINEAR_SPEED,
            Constants.SwerveConstants.MAX_LINEAR_SPEED);

    drive.runVelocity(new ChassisSpeeds(-yPIDEffort, xPIDEffort, 0));

    Logger.recordOutput("Distance Error", distanceError);

    Logger.recordOutput("Note Error", noteError);

    Logger.recordOutput("at xPID", xPID.atSetpoint());
    Logger.recordOutput("at yPID", yPID.atSetpoint());

    Logger.recordOutput("xPIDEffort", xPIDEffort);
    Logger.recordOutput("yPIDEffort", yPIDEffort);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return LimelightHelpers.getTA(Constants.LL_INTAKE) <= 0;
  }
}
