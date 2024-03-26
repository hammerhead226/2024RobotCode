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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.LED_STATE;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.led.LED;
import frc.robot.util.FieldConstants;
import java.util.List;

public class AlignToAmp extends Command {

  private final Drive drive;

  private final LED led;

  private final CommandXboxController controller;

  private double[] gains = new double[3];
  private DriverStation.Alliance alliance = null;

  private Rotation2d targetRotation = new Rotation2d();
  private Pose2d TargetPos = new Pose2d(FieldConstants.ampCenter, targetRotation);
  /** Creates a new AlignToAmp. */
  public AlignToAmp(Drive drive, CommandXboxController controller, LED led) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;

    this.led = led;

    this.controller = controller;

    addRequirements(drive, led);

    targetRotation = drive.getRotation();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    led.setState(LED_STATE.GREEN);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
      targetRotation = new Rotation2d(90);
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
              new GoalEndState(0, targetRotation));

      AutoBuilder.followPath(path).schedule();

      // pathfindingCommand.schedule();
    }

    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      targetRotation = new Rotation2d(90);

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
              new GoalEndState(0, targetRotation));

      pathRed.preventFlipping = true;

      AutoBuilder.followPath(pathRed).schedule();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return false;
  }
}
