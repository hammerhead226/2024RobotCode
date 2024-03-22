// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.LED_STATE;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.led.LED;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class BellevilleAlignToNoteAuto extends Command {
  /** Creates a new AlignToNoteTeleop. */
  private final Drive drive;

  private final LED led;

  private final PIDController xPID;
  private final PIDController yPID;

  private double startingPositionX;
  private final double threshold;

  private final LoggedTunableNumber xKp = new LoggedTunableNumber("AlignToNoteAuto/xKp");
  private final LoggedTunableNumber xKd = new LoggedTunableNumber("AlignToNoteAuto/xKd");

  private final LoggedTunableNumber yKp = new LoggedTunableNumber("AlignToNoteAuto/yKp");
  private final LoggedTunableNumber yKd = new LoggedTunableNumber("AlignToNoteAuto/yKd");

  public BellevilleAlignToNoteAuto(Drive drive, LED led, double threshold) {

    switch (Constants.getMode()) {
      case REAL:
        xKp.initDefault(0.042);
        xKd.initDefault(0);

        yKp.initDefault(0.08);
        yKd.initDefault(0);

        break;

      default:
        break;
    }

    Logger.recordOutput("startingpos", startingPositionX);
    this.threshold = threshold;

    yPID = new PIDController(0.055, 0, 0);
    xPID = new PIDController(xKp.get(), 0, xKd.get());

    xPID.setTolerance(5);
    yPID.setTolerance(5);
    this.drive = drive;
    this.led = led;
    addRequirements(drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    led.setState(LED_STATE.FLASHING_GREEN);
    startingPositionX = drive.getPose().getX();
    Logger.recordOutput("startingpos", startingPositionX);
    xPID.setSetpoint(-4);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.recordOutput("TA", LimelightHelpers.getTA(Constants.LL_INTAKE));

    double noteError = drive.getNoteError();
    double distanceError = (startingPositionX + threshold) - drive.getPose().getY();

    double xPIDEffort =
        MathUtil.clamp(
            xPID.calculate(noteError),
            -Constants.SwerveConstants.MAX_LINEAR_SPEED,
            Constants.SwerveConstants.MAX_LINEAR_SPEED);
    double yPIDEffort =
        MathUtil.clamp(
            yPID.calculate(distanceError),
            -Constants.SwerveConstants.MAX_LINEAR_SPEED,
            Constants.SwerveConstants.MAX_LINEAR_SPEED);

    drive.runVelocity(new ChassisSpeeds(1, xPIDEffort, 0));

    Logger.recordOutput("Distance Error", distanceError);

    Logger.recordOutput("Note Error", noteError);

    Logger.recordOutput("at xPID", xPID.atSetpoint());
    Logger.recordOutput("at yPID", yPID.atSetpoint());

    Logger.recordOutput("xPIDEffort", xPIDEffort);
    Logger.recordOutput("yPIDEffort", yPIDEffort);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    led.setState(LED_STATE.BLUE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(startingPositionX - drive.getPose().getX()) > threshold;
  }
}
