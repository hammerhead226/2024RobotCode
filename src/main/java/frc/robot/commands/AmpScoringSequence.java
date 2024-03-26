// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.FieldConstants;
import java.util.List;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AmpScoringSequence extends SequentialCommandGroup {
  /** Creates a new AmpScoringSequence. */
  private final Shooter shooter;

  private final Drive drive;
  private final LED led;
  private final CommandXboxController controller;
  private final Elevator elevator;
  private final Pivot pivot;
  private final PIDController pid;

  private double[] gains = new double[3];
  private DriverStation.Alliance alliance = null;

  public AmpScoringSequence(
      Shooter shooter,
      Drive drive,
      LED led,
      CommandXboxController controller,
      Elevator elevator,
      Pivot pivot) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.shooter = shooter;
    this.drive = drive;
    this.led = led;
    this.controller = controller;
    this.elevator = elevator;
    this.pivot = pivot;
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
        gains[0] = 7;
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

    addCommands(new InstantCommand(() -> turnToAmp()), new AlignToAmp(drive, controller, led));
  }

  public void AlignToAmp() {
    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
      // Rotation2d targetRotation = new Rotation2d(Units.degreesToRadians(270));
      List<Translation2d> pointsToAmp =
          PathPlannerPath.bezierFromPoses(
              new Pose2d(
                  drive.getPose().getX(), drive.getPose().getY(), drive.getPose().getRotation()),
              new Pose2d(
                  FieldConstants.ampCenter.getX(),
                  FieldConstants.ampCenter.getY() - 0.5,
                  drive.getPose().getRotation()));
      PathPlannerPath path =
          new PathPlannerPath(
              pointsToAmp,
              new PathConstraints(3, 3, Units.degreesToRadians(540), Units.degreesToRadians(720)),
              new GoalEndState(0, drive.getPose().getRotation()));

      AutoBuilder.followPath(path).schedule();

      // pathfindingCommand.schedule();
    }

    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      // Rotation2d targetRotation = new Rotation2d(Units.degreesToRadians(90));

      List<Translation2d> pointsToAmpRed =
          PathPlannerPath.bezierFromPoses(
              new Pose2d(
                  FieldConstants.fieldLength - drive.getPose().getX(),
                  drive.getPose().getY(),
                  drive.getPose().getRotation()),
              new Pose2d(
                  FieldConstants.fieldLength - FieldConstants.ampCenter.getX(),
                  FieldConstants.ampCenter.getY() - 0.5,
                  drive.getPose().getRotation()));

      PathPlannerPath pathRed =
          new PathPlannerPath(
              pointsToAmpRed,
              new PathConstraints(3, 3, Units.degreesToRadians(540), Units.degreesToRadians(720)),
              new GoalEndState(0, drive.getPose().getRotation()));

      pathRed.preventFlipping = true;

      AutoBuilder.followPath(pathRed).schedule();
    }
  }

  public void turnToAmp() {
    double targetAngle;
    if (DriverStation.getAlliance().isPresent()) this.alliance = DriverStation.getAlliance().get();

    if (alliance == DriverStation.Alliance.Red) {
      targetAngle =
          new Rotation2d(
                      FieldConstants.fieldLength - drive.getPose().getX(),
                      FieldConstants.ampCenter.getY() - drive.getPose().getY())
                  .getDegrees()
              + 270;
      pid.setSetpoint(
          new Rotation2d(
                      FieldConstants.fieldLength - drive.getPose().getX(),
                      FieldConstants.ampCenter.getY() - drive.getPose().getY())
                  .getDegrees()
              + 300);
    } else {
      targetAngle =
          new Rotation2d(
                      -drive.getPose().getX(),
                      FieldConstants.ampCenter.getY() - drive.getPose().getY())
                  .getDegrees()
              + 360;
      pid.setSetpoint(
          new Rotation2d(
                      -drive.getPose().getX(),
                      FieldConstants.ampCenter.getY() - drive.getPose().getY())
                  .getDegrees()
              + 360);
    }

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
            3, 3, Math.toRadians(angularSpeed), drive.getPose().getRotation()));
  }
}
