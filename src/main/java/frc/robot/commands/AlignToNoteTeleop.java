// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.LED_STATE;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class AlignToNoteTeleop extends Command {
  /** Creates a new AlignToNoteTeleop. */
  private final Drive drive;

  private final Pivot pivot;
  private final Intake intake;
  private final Shooter shooter;
  private final LED led;

  private final CommandXboxController controller;

  private final PIDController xPID;
  private final PIDController yPID;

  private final LoggedTunableNumber xKp = new LoggedTunableNumber("AlignToNoteAuto/xKp");
  private final LoggedTunableNumber xKd = new LoggedTunableNumber("AlignToNoteAuto/xKd");

  private final LoggedTunableNumber yKp = new LoggedTunableNumber("AlignToNoteAuto/yKp");
  private final LoggedTunableNumber yKd = new LoggedTunableNumber("AlignToNoteAuto/yKd");

  public AlignToNoteTeleop(
      Drive drive,
      Shooter shooter,
      Pivot pivot,
      Intake intake,
      LED led,
      CommandXboxController controller) {

    switch (Constants.getMode()) {
      case REAL:
        xKp.initDefault(0.012);
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
    this.pivot = pivot;
    this.intake = intake;
    this.shooter = shooter;
    this.led = led;
    this.controller = controller;
    addRequirements(drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pivot.setPivotGoal(Constants.PivotConstants.INTAKE_SETPOINT_DEG);
    intake.runRollers(12);
    shooter.setFeedersRPM(1000);
    led.setColor(LED_STATE.FLASHING_GREEN);
    yPID.setSetpoint(-13);
    xPID.setSetpoint(-9);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.recordOutput("TA", LimelightHelpers.getTA(Constants.LL_INTAKE));
    if (LimelightHelpers.getTV(Constants.LL_INTAKE)) {
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

      // drive.runVelocity(
      //     new ChassisSpeeds(
      //         -controller.getLeftY() * Constants.SwerveConstants.MAX_LINEAR_SPEED, xPIDEffort,
      // 0));

      drive.runVelocity(
          new ChassisSpeeds(
              0
                  + ChassisSpeeds.fromFieldRelativeSpeeds(
                          controller.getLeftX() * Constants.SwerveConstants.MAX_LINEAR_SPEED,
                          controller.getLeftY() * Constants.SwerveConstants.MAX_LINEAR_SPEED,
                          0,
                          drive.getPose().getRotation())
                      .vxMetersPerSecond,
              xPIDEffort
                  + ChassisSpeeds.fromFieldRelativeSpeeds(
                          controller.getLeftX() * Constants.SwerveConstants.MAX_LINEAR_SPEED,
                          controller.getLeftY() * Constants.SwerveConstants.MAX_LINEAR_SPEED,
                          0,
                          drive.getPose().getRotation())
                      .vyMetersPerSecond,
              0));

      // drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(null, null));
      // ChassisSpeeds.from
      Logger.recordOutput("Distance Error", distanceError);

      Logger.recordOutput("Note Error", noteError);

      Logger.recordOutput("at xPID", xPID.atSetpoint());
      Logger.recordOutput("at yPID", yPID.atSetpoint());

      Logger.recordOutput("xPIDEffort", xPIDEffort);
      Logger.recordOutput("yPIDEffort", yPIDEffort);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopRollers();
    shooter.stopFeeders();
    led.setColor(LED_STATE.BLUE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shooter.seesNote() || !LimelightHelpers.getTV(Constants.LL_INTAKE);
  }
}
