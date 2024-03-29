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
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.LED_STATE;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class AlignToNoteTeleop extends Command {
  /** Creates a new AlignToNoteTeleop. */
  private final Drive drive;

  private final Pivot pivot;
  private final Intake intake;
  private final Shooter shooter;
  private final LED led;

  private final PIDController xPID;
  private final PIDController yPID;

  private double startingPositionX;

  private final LoggedTunableNumber xKp = new LoggedTunableNumber("AlignToNoteAuto/xKp");
  private final LoggedTunableNumber xKd = new LoggedTunableNumber("AlignToNoteAuto/xKd");

  private final LoggedTunableNumber yKp = new LoggedTunableNumber("AlignToNoteAuto/yKp");
  private final LoggedTunableNumber yKd = new LoggedTunableNumber("AlignToNoteAuto/yKd");

  private DoubleSupplier xSupplier;
  private DoubleSupplier ySupplier;

  public AlignToNoteTeleop(
      Drive drive,
      Shooter shooter,
      Pivot pivot,
      Intake intake,
      LED led,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier) {

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

    yPID = new PIDController(0.055, 0, 0);
    xPID = new PIDController(0.042, 0, 0);

    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;

    xPID.setTolerance(5);
    yPID.setTolerance(5);
    this.drive = drive;
    this.shooter = shooter;
    this.pivot = pivot;
    this.intake = intake;
    this.led = led;
    addRequirements(drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pivot.setPivotGoal(Constants.PivotConstants.INTAKE_SETPOINT_DEG);
    intake.runRollers(12);
    shooter.setFeedersRPM(500);
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

    double xPIDEffort =
        MathUtil.clamp(
            xPID.calculate(noteError),
            -0.5 * Constants.SwerveConstants.MAX_LINEAR_SPEED,
            0.5 * Constants.SwerveConstants.MAX_LINEAR_SPEED);
    // xSupplier = () -> -controller.getLeftX();
    // ySupplier = () -> -controller.getLeftY();

    double linearMagnitude =
        MathUtil.applyDeadband(Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), 0.1);
    Rotation2d linearDirection = new Rotation2d(ySupplier.getAsDouble(), xSupplier.getAsDouble());

    // Square values
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Calcaulate new linear velocity
    Translation2d linearVelocity =
        new Pose2d(new Translation2d(), linearDirection)
            .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
            .getTranslation();

    Logger.recordOutput("linear velocity auto align", linearVelocity);

    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;

    double forwardSpeed =
        ChassisSpeeds.fromFieldRelativeSpeeds(
                linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                0,
                isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation())
            .vxMetersPerSecond;

    double sidewaysSpeed =
        ChassisSpeeds.fromFieldRelativeSpeeds(
                linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                0,
                isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation())
            .vyMetersPerSecond;

    // double forwardSpeed =
    //     ChassisSpeeds.fromFieldRelativeSpeeds(
    //             -0.5 * controller.getLeftY() * drive.getMaxLinearSpeedMetersPerSec(),
    //             -0.5 * controller.getLeftX() * drive.getMaxLinearSpeedMetersPerSec(),
    //             0,
    //             isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) :
    // drive.getRotation())
    //         .vxMetersPerSecond;

    // double sidewaysSpeed =
    //     ChassisSpeeds.fromFieldRelativeSpeeds(
    //             -0.5 * controller.getLeftY() * drive.getMaxLinearSpeedMetersPerSec(),
    //             -0.5 * controller.getLeftX() * drive.getMaxLinearSpeedMetersPerSec(),
    //             0,
    //             isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) :
    // drive.getRotation())
    //         .vyMetersPerSecond;

    drive.runVelocity(new ChassisSpeeds(forwardSpeed, sidewaysSpeed + xPIDEffort, 0));

    Logger.recordOutput("Forward Speed", forwardSpeed);
    Logger.recordOutput("Sideways Speed", sidewaysSpeed);

    Logger.recordOutput("Note Error", noteError);

    Logger.recordOutput("at xPID", xPID.atSetpoint());

    Logger.recordOutput("xPIDEffort", xPIDEffort);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopRollers();
    shooter.stopFeeders();
    led.setState(intake.getIntakeState());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shooter.seesNote();
  }
}
