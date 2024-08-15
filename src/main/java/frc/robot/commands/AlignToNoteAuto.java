// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.LED_STATE;
import frc.robot.Constants.NOTE_POSITIONS;
import frc.robot.Constants.NoteState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants;
import java.util.HashMap;
import org.littletonrobotics.junction.Logger;

public class AlignToNoteAuto extends Command {
  /** Creates a new AlignToNote. */
  LED led;

  Drive drive;
  Pivot pivot;
  Intake intake;
  Shooter shooter;
  Command generatedPathCommand;
  Command targetNotePathCommand;
  Translation2d targetNoteLocation;
  Rotation2d targetNoteRotation;

  private boolean useGeneratedPathCommand;

  private boolean finished;
  private HashMap<NOTE_POSITIONS, Translation2d> noteLocations = new HashMap<>();

  public AlignToNoteAuto(LED led, Drive drive, Shooter shooter, Intake intake, Pivot pivot) {
    this.shooter = shooter;
    this.pivot = pivot;
    this.intake = intake;
    this.led = led;
    this.drive = drive;
    finished = false;

    noteLocations.put(NOTE_POSITIONS.C5, FieldConstants.StagingLocations.centerlineTranslations[0]);
    noteLocations.put(NOTE_POSITIONS.C4, FieldConstants.StagingLocations.centerlineTranslations[1]);
    noteLocations.put(NOTE_POSITIONS.C3, FieldConstants.StagingLocations.centerlineTranslations[2]);
    noteLocations.put(NOTE_POSITIONS.C2, FieldConstants.StagingLocations.centerlineTranslations[3]);
    noteLocations.put(NOTE_POSITIONS.C1, FieldConstants.StagingLocations.centerlineTranslations[4]);
    noteLocations.put(
        NOTE_POSITIONS.B1,
        AllianceFlipUtil.apply(FieldConstants.StagingLocations.spikeTranslations[2]));
    noteLocations.put(
        NOTE_POSITIONS.B2,
        AllianceFlipUtil.apply(FieldConstants.StagingLocations.spikeTranslations[1]));
    noteLocations.put(
        NOTE_POSITIONS.B3,
        AllianceFlipUtil.apply(FieldConstants.StagingLocations.spikeTranslations[0]));
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive, shooter, led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.recordOutput("auto pickup init", "true");

    led.setState(LED_STATE.FLASHING_RED);
    intake.runRollers(12);
    shooter.setFeedersRPM(500);
    pivot.setPivotGoal(Constants.PivotConstants.INTAKE_SETPOINT_DEG);
    targetNoteLocation = noteLocations.get(drive.getTargetNote());
    useGeneratedPathCommand =
        drive.getCachedNoteLocation().getDistance(targetNoteLocation) < 2.5
            && drive.getCachedNoteLocation() != null;
    Logger.recordOutput(
        "cached note distance ", drive.getCachedNoteLocation().getDistance(targetNoteLocation));
    Logger.recordOutput("useGeneratedPath command", useGeneratedPathCommand);
    // useGeneratedPathCommand = false;
    // generatedPathCommand = AutoBuilder.followPath(drive.generatePathToNote());
    if (useGeneratedPathCommand) {
      generatedPathCommand = AutoBuilder.followPath(drive.generatePathToNote());

      generatedPathCommand.initialize();
    } else {
      // targetNoteRotation =
      //     new Rotation2d(
      //         targetNoteLocation.getX() - drive.getPose().getX(),
      //         targetNoteLocation.getY() - drive.getPose().getY());
      // targetNotePathCommand =
      //     drive.generateTrajectory(
      //         new Pose2d(targetNoteLocation, targetNoteRotation), 3, 2.45, 100, 180, 0.5);
      targetNotePathCommand =
          AutoBuilder.followPath(drive.generatePathToNoteBlind(targetNoteLocation));

      targetNotePathCommand.initialize();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.recordOutput("useGeneratedPathCommand", useGeneratedPathCommand);
    finished = shooter.seesNote() == NoteState.SENSOR;
    if (useGeneratedPathCommand) {
      generatedPathCommand.execute();
    } else {
      targetNotePathCommand.execute();
    }

    Logger.recordOutput("path is finished", finished);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    led.setState(LED_STATE.BLUE);
    intake.stopRollers();
    shooter.stopFeeders();
    // pathCommand.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Logger.recordOutput("isFinished align note", shooter.seesNote());
    // return false;
    return shooter.seesNote() == NoteState.SENSOR
        || shooter.seesNote() == NoteState.CURRENT
        || finished;
  }
}
