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
import frc.robot.Constants.LED_STATE;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.FieldConstants;
import org.littletonrobotics.junction.Logger;

public class Aimbot extends Command {
  private final Drive drive;
  private final Shooter shooter;
  private final Pivot pivot;
  private final LED led;

  private final CommandXboxController controller;
  private final PIDController pid;
  private double[] gains = new double[3];
  private DriverStation.Alliance alliance = null;

  private double distanceToSpeakerMeter = 0;
  private double pivotSetpointDeg = 0;
  /** Creates a new Aimbot. */
  public Aimbot(
      Drive drive, CommandXboxController controller, Shooter shooter, Pivot pivot, LED led) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.shooter = shooter;
    this.pivot = pivot;
    this.led = led;

    this.controller = controller;

    addRequirements(drive, shooter, pivot, led);

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
    pid.setTolerance(0);
    pid.enableContinuousInput(-180, 180);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    led.setColor(LED_STATE.FLASHING_GREEN);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turnToSpeaker();
    angleShooter();
  }

  public void angleShooter() {
    if (DriverStation.getAlliance().isPresent()) this.alliance = DriverStation.getAlliance().get();

    distanceToSpeakerMeter = calculateDistanceToSpeaker();
    shooter.setFlywheelRPMs(5400, 5400);
    pivot.setPivotGoal(calculatePivotAngleDeg(distanceToSpeakerMeter));
  }

  private double calculatePivotAngleDeg(double distanceToSpeakerMeter) {
    pivotSetpointDeg = (-0.32 * Math.abs(Units.metersToInches(distanceToSpeakerMeter) - 36) + 62);
    pivotSetpointDeg = MathUtil.clamp(pivotSetpointDeg, 39, 62);

    Logger.recordOutput("pivot target auto", pivotSetpointDeg);
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

  public void turnToSpeaker() {
    double targetAngle;
    if (DriverStation.getAlliance().isPresent()) this.alliance = DriverStation.getAlliance().get();

    if (alliance == DriverStation.Alliance.Red) {
      targetAngle =
          new Rotation2d(
                      FieldConstants.fieldLength - drive.getPose().getX(),
                      FieldConstants.Speaker.speakerCenterY - drive.getPose().getY())
                  .getDegrees()
              + 180;
      pid.setSetpoint(
          new Rotation2d(
                      FieldConstants.fieldLength - drive.getPose().getX(),
                      FieldConstants.Speaker.speakerCenterY - drive.getPose().getY())
                  .getDegrees()
              + 180);
    } else {
      targetAngle =
          new Rotation2d(
                      -drive.getPose().getX(),
                      FieldConstants.Speaker.speakerCenterY - drive.getPose().getY())
                  .getDegrees()
              + 180;
      pid.setSetpoint(
          new Rotation2d(
                      -drive.getPose().getX(),
                      FieldConstants.Speaker.speakerCenterY - drive.getPose().getY())
                  .getDegrees()
              + 180);
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
    shooter.setFeedersRPM(1000);
    led.setColor(LED_STATE.FLASHING_GREEN);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pid.atSetpoint() && shooter.atFlywheelSetpoints() && pivot.atSetpoint();
  }
}
