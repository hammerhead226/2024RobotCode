// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
public class ScoreAmp extends SequentialCommandGroup {
  /** Creates a new ScoreAmp. */
  private final Drive drive;

  private Rotation2d targetRotation = new Rotation2d();

  public ScoreAmp(Elevator elevator, Pivot pivot, Shooter shooter, Drive drive, LED led) {

    this.drive = drive;
    // new SetElevatorTarget(8, elevator)
    //             .andThen(new SetPivotTarget(Constants.PivotConstants.AMP_SETPOINT_DEG, pivot))
    //             .andThen(new InstantCommand(() -> shooter.setFlywheelRPMs(1200, 1200), shooter))
    //             .andThen(new InstantCommand(() -> shooter.setFeedersRPM(200), shooter)));

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ParallelCommandGroup(
            new InstantCommand(() -> AlignToAmp(), drive),
            new SetPivotTarget(Constants.PivotConstants.AMP_SETPOINT_DEG, pivot),
            new SetElevatorTarget(8, 1, elevator),
            new InstantCommand(() -> shooter.setFlywheelRPMs(1200, 1200), shooter)),
        new WaitCommand(1),
        new InstantCommand(() -> shooter.setFeedersRPM(200), shooter));

    // new InstantCommand(() -> shooter.setFeedersRPM(200), shooter));
    // new WaitUntilCommand(pivot::atSetpoint),

  }

  public void AlignToAmp() {
    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
      targetRotation =
          new Rotation2d(
              (Units.degreesToRadians(
                  new Rotation2d(FieldConstants.ampCenter.getX(), FieldConstants.ampCenter.getY())
                          .getDegrees()
                      + 13)));
      List<Translation2d> pointsToAmp =
          PathPlannerPath.bezierFromPoses(
              new Pose2d(
                  drive.getPose().getX(), drive.getPose().getY(), drive.getPose().getRotation()),
              new Pose2d(
                  FieldConstants.ampCenter.getX(),
                  FieldConstants.ampCenter.getY() - 0.5,
                  targetRotation));
      PathPlannerPath path =
          new PathPlannerPath(
              pointsToAmp,
              new PathConstraints(3, 3, Units.degreesToRadians(540), Units.degreesToRadians(720)),
              new GoalEndState(0, targetRotation, true));

      AutoBuilder.followPath(path).schedule();

      // pathfindingCommand.schedule();
    }

    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      targetRotation =
          new Rotation2d(
              Units.degreesToRadians(
                  new Rotation2d(
                              FieldConstants.fieldLength - FieldConstants.ampCenter.getX(),
                              FieldConstants.ampCenter.getY())
                          .getDegrees()
                      + 62));

      List<Translation2d> pointsToAmpRed =
          PathPlannerPath.bezierFromPoses(
              new Pose2d(
                  FieldConstants.fieldLength - drive.getPose().getX(),
                  drive.getPose().getY(),
                  drive.getPose().getRotation()),
              new Pose2d(
                  FieldConstants.fieldLength - FieldConstants.ampCenter.getX(),
                  FieldConstants.ampCenter.getY() - 0.5,
                  targetRotation));

      PathPlannerPath pathRed =
          new PathPlannerPath(
              pointsToAmpRed,
              new PathConstraints(3, 3, Units.degreesToRadians(540), Units.degreesToRadians(720)),
              new GoalEndState(0, targetRotation, true));

      pathRed.preventFlipping = true;

      AutoBuilder.followPath(pathRed).schedule();
    }
  }
}
