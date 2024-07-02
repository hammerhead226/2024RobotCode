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
  private static double sideWaysError = 0;
  private static double wantedSidewaysVelocity = 0;
  private static PIDController pid = new PIDController(1.2, 0, 0);

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
      BooleanSupplier intakeAssistSupplier) {

    if (shooter.seesNote() == NoteState.CURRENT || shooter.seesNote() == NoteState.SENSOR) {
      return joystickDrive(drive, xSupplier, ySupplier, omegaSupplier, intakeAssistSupplier);
    } else {

    }

    return new InstantCommand(() -> led.setState(LED_STATE.RED))
        .andThen(
            new ParallelCommandGroup(
                joystickDrive(drive, xSupplier, ySupplier, omegaSupplier, intakeAssistSupplier),
                new PivotIntakeTele(pivot, intake, shooter, led, false)));
  }
  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      BooleanSupplier intakeAssistSupplier) {
    return Commands.run(
        () -> {

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


          sideWaysError =
              drive.getNotePositionRobotRelative().getY();


          if (intakeAssistSupplier.getAsBoolean()) {
            wantedSidewaysVelocity = calculateWantedSidewaysVelocity(drive, sideWaysError, forwardSpeed);
          } else {
            wantedSidewaysVelocity = sidewaysSpeed;
          }

          Logger.recordOutput("Wanted Sideways Velocity", wantedSidewaysVelocity);
          Logger.recordOutput("Note Assist Error", sideWaysError);

          double assistEffort = wantedSidewaysVelocity - sidewaysSpeed;

          Logger.recordOutput("Assist Effort", assistEffort);

          drive.runVelocity(
              new ChassisSpeeds(forwardSpeed, sidewaysSpeed + assistEffort, rotationSpeed));
        },
        drive);
  }

  private static double calculateWantedSidewaysVelocity(Drive drive, double sidewaysError, double forwardSpeed) {
    double wantedSidewaysVelocity = pid.calculate(sideWaysError);
    // 0.36m is roughly 14 inches
    double forwardDisplacementToNote = drive.getNotePositionRobotRelative().getX() - 0.36;
    double maxTime = calculateTime(forwardSpeed, forwardDisplacementToNote);
    double minVelocity = calculateVelocity(maxTime, drive.getNotePositionRobotRelative().getY());

    wantedSidewaysVelocity = Math.max(Math.abs(wantedSidewaysVelocity), Math.abs(minVelocity));
    wantedSidewaysVelocity = MathUtil.clamp(wantedSidewaysVelocity, -drive.getMaxLinearSpeedMetersPerSec(), drive.getMaxLinearSpeedMetersPerSec());

    return wantedSidewaysVelocity;
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
