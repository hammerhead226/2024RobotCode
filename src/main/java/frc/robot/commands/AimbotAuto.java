// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.LED_STATE;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.FieldConstants;
import org.littletonrobotics.junction.Logger;

public class AimbotAuto extends Command {

  private final Drive drive;
  private final Shooter shooter;
  private final Pivot pivot;
  private final LED led;

  private final PIDController pid;
  private double[] gains = new double[3];
  private DriverStation.Alliance alliance = null;

  private double distanceToSpeakerMeter = 0;
  private double pivotSetpointDeg = 0;
  /** Creates a new Aimbot. */
  public AimbotAuto(Drive drive, Shooter shooter, Pivot pivot, LED led) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.shooter = shooter;
    this.pivot = pivot;
    this.led = led;

    addRequirements(shooter, pivot, led);

    switch (Constants.currentMode) {
      case REAL:
        gains[0] = 2.5;
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
    pid.setTolerance(3);
    pid.enableContinuousInput(-180, 180);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.enabledOverride();
    led.setState(LED_STATE.FLASHING_GREEN);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    angleShooter();

    Logger.recordOutput(
        "distance from speak (ft)", Units.metersToFeet(calculateDistanceToSpeaker()));

    // if (Units.metersToFeet(calculateDistanceToSpeaker()) > 12) {
    //   led.setState(LED_STATE.FLASHING_RED);
    // } else {
    //   led.setState(LED_STATE.GREEN);
    // }
  }

  public void angleShooter() {
    if (DriverStation.getAlliance().isPresent()) this.alliance = DriverStation.getAlliance().get();
    // Logger.recordOutput("distance to speak", Units.metersToFeet(distanceToSpeakerMeter));
    distanceToSpeakerMeter = calculateDistanceToSpeaker();
    if (Units.metersToFeet(distanceToSpeakerMeter) > 9) {
      // double shootingSpeed =
      //     MathUtil.clamp(
      //         calculateShooterSpeed(Units.metersToFeet(distanceToSpeakerMeter)), 3250, 5400);
      double shootingSpeed = calculateShooterSpeed(Units.metersToFeet(distanceToSpeakerMeter));

      shooter.setFlywheelRPMs(shootingSpeed, shootingSpeed);
    } else shooter.setFlywheelRPMs(5400, 5400);
    pivot.setPivotGoal(calculatePivotAngleDeg(distanceToSpeakerMeter));
  }

  private double calculateShooterSpeed(double distanceToSpeakerFeet) {

    if (distanceToSpeakerFeet >= 11) {
      return -430.7 * distanceToSpeakerFeet + 8815;
    } else return -600. * distanceToSpeakerFeet + 10406;
    // return -556.25 * distanceToSpeakerFeet + 10406;

  }

  private double calculatePivotAngleDeg(double distanceToSpeakerMeter) {
    pivotSetpointDeg = (-0.272 * Math.abs(Units.metersToInches(distanceToSpeakerMeter) - 36) + 60);
    pivotSetpointDeg = MathUtil.clamp(pivotSetpointDeg, 39, 62);

    if (Units.metersToFeet(distanceToSpeakerMeter) >= 9.098) {
      return 39;
    }
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

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.disableOverride();
    shooter.setFeedersRPM(1000);
    led.setState(LED_STATE.GREY);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pid.atSetpoint() && shooter.atFlywheelSetpoints() && pivot.atGoal();
  }
}
