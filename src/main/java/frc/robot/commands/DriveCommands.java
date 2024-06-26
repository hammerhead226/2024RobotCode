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
  private static double error = 0;
  private static double assistEffort = 0;
  private static PIDController pid = new PIDController(1, 0, 0);

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
          
          error = getNoteDistancePerpToVel(drive.getNotePositionRobotRelative(), xSupplier.getAsDouble(), ySupplier.getAsDouble());

          if (intakeAssistSupplier.getAsBoolean()) {
            assistEffort = pid.calculate(error);
          } else {
            assistEffort = 0;
          }


          Logger.recordOutput("Assist Effort", assistEffort);
          Logger.recordOutput("Note Assist Error", error);
          
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

          drive.runVelocity(new ChassisSpeeds(forwardSpeed, sidewaysSpeed + assistEffort, rotationSpeed));
        },
        drive);
  }

  private static double getNoteDistancePerpToVel(
      Translation2d noteLocRobotRel, double controllerX, double controllerY) {
    double commandedVelAngle = Math.atan2(controllerY, controllerX);

    Rotation2d commandVelRotation = Rotation2d.fromDegrees(commandedVelAngle);
    return Math.sin(commandVelRotation.minus(new Rotation2d(noteLocRobotRel.getX(), noteLocRobotRel.getY())).getDegrees()) * noteLocRobotRel.getNorm();
  }
}
