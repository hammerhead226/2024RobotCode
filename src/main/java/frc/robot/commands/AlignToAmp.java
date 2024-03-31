// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.led.LED;
import frc.robot.util.FieldConstants;

public class AlignToAmp extends Command {
  private final Drive drive;

  private List<Translation2d> pointsToAmp;
  private Command pathCommand;
  private PathPlannerPath path;

  private Rotation2d targetRotation = new Rotation2d();
  /** Creates a new AlignToAmp. */
  public AlignToAmp(Drive drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;

    addRequirements(drive);

    targetRotation = drive.getRotation();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
        targetRotation =
            new Rotation2d(
                (Units.degreesToRadians(
                    new Rotation2d(FieldConstants.ampCenter.getX(), FieldConstants.ampCenter.getY())
                            .getDegrees()
                        + 13)));
        pointsToAmp =
            PathPlannerPath.bezierFromPoses(
                new Pose2d(
                    drive.getPose().getX(), drive.getPose().getY(), drive.getPose().getRotation()),
                new Pose2d(
                    FieldConstants.ampCenter.getX(),
                    FieldConstants.ampCenter.getY() - 0.5,
                    targetRotation));
        path =
            new PathPlannerPath(
                pointsToAmp,
                new PathConstraints(3, 3, Units.degreesToRadians(540), Units.degreesToRadians(720)),
                new GoalEndState(0, targetRotation, true));
      } else {
        targetRotation =
            new Rotation2d(
                Units.degreesToRadians(
                    new Rotation2d(
                                FieldConstants.fieldLength - FieldConstants.ampCenter.getX(),
                                FieldConstants.ampCenter.getY())
                            .getDegrees()
                        + 62));
  
        pointsToAmp =
            PathPlannerPath.bezierFromPoses(
                new Pose2d(
                    FieldConstants.fieldLength - drive.getPose().getX(),
                    drive.getPose().getY(),
                    drive.getPose().getRotation()),
                new Pose2d(
                    FieldConstants.fieldLength - FieldConstants.ampCenter.getX(),
                    FieldConstants.ampCenter.getY() - 0.5,
                    targetRotation));
  
        path =
            new PathPlannerPath(
                pointsToAmp,
                new PathConstraints(3, 3, Units.degreesToRadians(540), Units.degreesToRadians(720)),
                new GoalEndState(0, targetRotation, true));
    }

    path.preventFlipping = true;
    pathCommand = AutoBuilder.followPath(path);

    pathCommand.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pathCommand.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pathCommand.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
