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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.LED_STATE;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.shooter.Shooter;
import java.util.List;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoPickupNote extends SequentialCommandGroup {
  /** Creates a new ScoreAmp. */
  private final Drive drive;

  private Rotation2d targetRotation = new Rotation2d();

  public AutoPickupNote(Intake intake, Pivot pivot, Shooter shooter, Drive drive, LED led) {

    this.drive = drive;
    // new SetElevatorTarget(8, elevator)
    //             .andThen(new SetPivotTarget(Constants.PivotConstants.AMP_SETPOINT_DEG, pivot))
    //             .andThen(new InstantCommand(() -> shooter.setFlywheelRPMs(1200, 1200), shooter))
    //             .andThen(new InstantCommand(() -> shooter.setFeedersRPM(200), shooter)));

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        // new ParallelCommandGroup(
        new InstantCommand(() -> pivot.setPivotGoal(Constants.PivotConstants.INTAKE_SETPOINT_DEG)),
        new InstantCommand(() -> intake.runRollers(12)),
        new InstantCommand(() -> shooter.setFeedersRPM(500)),
        new InstantCommand(() -> led.setState(LED_STATE.FLASHING_GREEN)),
        new InstantCommand(() -> AlignToNote(), drive)
        //     new SetPivotTarget(Constants.PivotConstants.AMP_SETPOINT_DEG, pivot),
        //     new SetElevatorTarget(8, 1, elevator),
        //     new InstantCommand(() -> shooter.setFlywheelRPMs(1200, 1200), shooter)),
        // new WaitCommand(1),
        // new InstantCommand(() -> shooter.setFeedersRPM(200), shooter));
        );
    // new InstantCommand(() -> shooter.setFeedersRPM(200), shooter));
    // new WaitUntilCommand(pivot::atSetpoint),

  }

  public void AlignToNote() {
    Translation2d cachedNoteT2d = drive.getCachedNoteLocation();
    if (drive.NoteImageIsNew()) {
      targetRotation =
          new Rotation2d(
              cachedNoteT2d.getY() - drive.getPose().getY(),
              cachedNoteT2d.getX() - drive.getPose().getX());
      List<Translation2d> pointsToNote =
          PathPlannerPath.bezierFromPoses(
              new Pose2d(
                  drive.getPose().getX(), drive.getPose().getY(), drive.getPose().getRotation()),
              new Pose2d(cachedNoteT2d.getX(), cachedNoteT2d.getY(), targetRotation));
      PathPlannerPath path =
          new PathPlannerPath(
              pointsToNote,
              new PathConstraints(1, 1, Units.degreesToRadians(180), Units.degreesToRadians(270)),
              new GoalEndState(0.2, targetRotation, true));

      AutoBuilder.followPath(path).schedule();

    } else {
      // Set LEDs to red?
    }
  }
}
