// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.LED_STATE;
import frc.robot.Constants.NoteState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class DriveCommands {
  private static final double DEADBAND = 0.1;
  private static final double NOTE_FORWARD_OFFSET = -0.36;
  private static double sideWaysError = 0;
  private static double wantedSidewaysVelocity = 0;
  private static double wantedRotationVelocity = 0;
  private static double sidewaysAssistEffort = 0;
  private static double rotationAssistEffort = 0;
  private static double forwardConstantVelocity = 0;
  private static PIDController sidewaysPID =
      new PIDController(1.5, 0, 0, Constants.LOOP_PERIOD_SECS);
  private static PIDController rotationPID =
      new PIDController(2.54, 0, 0, Constants.LOOP_PERIOD_SECS);

  private static double counter = 0;

  private DriveCommands() {}

  public static Command intakeCommand(
      Drive drive,
      Shooter shooter,
      Pivot pivot,
      Intake intake,
      LED led,
      CommandXboxController controller,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      BooleanSupplier intakeAssistSupplier,
      BooleanSupplier turnToSourceSupplier) {

    if (shooter.seesNote() == NoteState.CURRENT || shooter.seesNote() == NoteState.SENSOR) {
      return joystickDrive(
          drive,
          led,
          xSupplier,
          ySupplier,
          omegaSupplier,
          intakeAssistSupplier,
          turnToSourceSupplier);
    } else {

    }

    return new InstantCommand(() -> led.setState(LED_STATE.RED))
        .andThen(
            new ParallelCommandGroup(
                joystickDrive(
                    drive,
                    led,
                    xSupplier,
                    ySupplier,
                    omegaSupplier,
                    intakeAssistSupplier,
                    turnToSourceSupplier),
                new PivotIntakeTele(pivot, intake, shooter, led, false)));
  }
  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      LED led,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      BooleanSupplier intakeAssistSupplier,
      BooleanSupplier turnToSourceSupplier) {
    return Commands.run(
        () -> {
          rotationPID.setTolerance(5);
          rotationPID.enableContinuousInput(-180, 180);
          sidewaysPID.setTolerance(0.05460);
          // Apply deadband
          double linearMagnitude =
              MathUtil.applyDeadband(
                  Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND);
          Rotation2d linearDirection =
              new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square values
          linearMagnitude = linearMagnitude * linearMagnitude;
          omega = Math.copySign(omega * omega, omega);

          // Calcaulate new linear velocity
          Translation2d linearVelocity =
              new Pose2d(new Translation2d(), linearDirection)
                  .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                  .getTranslation();

          // Convert to field relative speeds & send command
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;

          ChassisSpeeds chassisSpeeds =
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec(),
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation());

          double forwardSpeed = chassisSpeeds.vxMetersPerSecond;

          double sidewaysSpeed = chassisSpeeds.vyMetersPerSecond;

          double rotationSpeed = chassisSpeeds.omegaRadiansPerSecond;

          sideWaysError = 0 - drive.getNotePositionRobotRelative().getY();

          if (turnToSourceSupplier.getAsBoolean()) {
            Rotation2d curreRotation2d = drive.getRotation();
            Rotation2d targeRotation2d;
            if (DriverStation.getAlliance().get() == Alliance.Blue) {
              targeRotation2d = Rotation2d.fromDegrees(-60);
            } else {
              targeRotation2d = Rotation2d.fromDegrees(240);
            }
            rotationPID.setSetpoint(targeRotation2d.getDegrees());

            wantedRotationVelocity =
                Math.toRadians(rotationPID.calculate(curreRotation2d.getDegrees()));

            rotationAssistEffort = wantedRotationVelocity - rotationSpeed * 0.1690;

          } else {
            wantedRotationVelocity = rotationSpeed;
            rotationAssistEffort = 0;
          }

          if ((intakeAssistSupplier.getAsBoolean() && (counter < 10 || drive.canSeeNote()))) {
            if (!drive.canSeeNote()) {
              counter++;
            } else {
              counter = 0;
            }
            forwardConstantVelocity = .14657 * 2;
            led.setState(LED_STATE.FLASHING_RED);
            wantedSidewaysVelocity =
                calculateWantedSidewaysVelocity(drive, sideWaysError, forwardSpeed);
            sidewaysAssistEffort = wantedSidewaysVelocity - sidewaysSpeed * 0.2345;
          } else {
            led.setState(LED_STATE.RED);
            forwardConstantVelocity = 0;
            wantedSidewaysVelocity = sidewaysSpeed;
            sidewaysAssistEffort = 0;
          }

          Logger.recordOutput("Wanted Sideways Velocity", wantedSidewaysVelocity);
          Logger.recordOutput("Note Assist Error", sideWaysError);

          Logger.recordOutput("Sideways Assist Effort", sidewaysAssistEffort);
          Logger.recordOutput("Rotation Assist Effort", rotationAssistEffort);

          drive.runVelocity(
              new ChassisSpeeds(
                  MathUtil.clamp(
                      forwardSpeed + forwardConstantVelocity,
                      -drive.getMaxLinearSpeedMetersPerSec(),
                      drive.getMaxLinearSpeedMetersPerSec()),
                  MathUtil.clamp(
                      sidewaysSpeed + sidewaysAssistEffort,
                      -drive.getMaxLinearSpeedMetersPerSec(),
                      drive.getMaxLinearSpeedMetersPerSec()),
                  MathUtil.clamp(
                      rotationSpeed + rotationAssistEffort,
                      -drive.getMaxAngularSpeedRadPerSec(),
                      drive.getMaxAngularSpeedRadPerSec())));
        },
        drive);
  }

  private static double calculateWantedSidewaysVelocity(
      Drive drive, double sidewaysError, double forwardSpeed) {
    double wantedSidewaysVelocityPID = sidewaysPID.calculate(sideWaysError);

    double forwardDisplacementToNote =
        drive.getNotePositionRobotRelative().getX() + NOTE_FORWARD_OFFSET;
    double maxTime;
    double minVelocity;
    if (forwardSpeed > 0 && forwardDisplacementToNote > 0) {
      maxTime = calculateTime(forwardSpeed, forwardDisplacementToNote);
      minVelocity = calculateVelocity(maxTime, drive.getNotePositionRobotRelative().getY());

      double wantedSidewaysVelocity =
          Math.max(Math.abs(wantedSidewaysVelocityPID), Math.abs(minVelocity))
              * (minVelocity / Math.abs(minVelocity));

      wantedSidewaysVelocity =
          MathUtil.clamp(
              wantedSidewaysVelocity,
              0.7 * -drive.getMaxLinearSpeedMetersPerSec(),
              0.7 * drive.getMaxLinearSpeedMetersPerSec());

      return wantedSidewaysVelocity;
    } else {
      return wantedSidewaysVelocityPID;
    }
  }

  private static double calculateTime(double velocity, double displacement) {
    double time = displacement / velocity;
    Logger.recordOutput("Time to note", time);

    return time;
  }

  private static double calculateVelocity(double time, double displacement) {
    double velocity = displacement / time;
    Logger.recordOutput("Velocity needed to note", velocity);

    return velocity;
  }
}
